"""Simulation manager. This is the center of the simulator. It does not use a GUI.

Tasks:
  * Keep the physics engine, the world (environment objects) and the robots.
  * Run the fixed-timestep physics loop. `update(wall_dt)` starts each cycle.
  * Update each sensor at its own rate.
  * Execute the commands of the thread-safe command queue (ROS 2, scripts).
  * Send snapshots to the listeners (for example, the ROS 2 bridge).

Sequence for each physics step:
    read commands -> apply motor commands -> step PyBullet ->
    update robot state -> sensors that are due -> state snapshots and clock
"""
from __future__ import annotations

import hashlib
import logging
import math
import time
from dataclasses import replace
from enum import Enum
from pathlib import Path
from typing import Callable, Optional, Protocol


from ..environment.environment_objects import EnvObjectConfig
from ..environment.ground_plane import default_ground
from ..environment.world_manager import WorldManager
from ..physics.collision_manager import CollisionManager
from ..physics.physics_engine import PhysicsEngine
from ..robot.robot_loader import CACHE_DIR, RobotLoader
from ..robot.urdf_parser import URDFError
from ..sensors.camera import Camera
from ..utils.transforms import quat_to_rpy
from .commands import (CmdVelCommand, Command, CommandQueue, DeleteRobotCommand, JointTargetsCommand,
                       SimControlCommand, SpawnRobotCommand)
from .configuration import AppConfig
from .event_manager import EventManager
from .robot_instance import RobotConfig, RobotInstance, sanitize_name
from .simulation_clock import SimulationClock
from .snapshots import JointDescriptor, RobotDescriptor, RobotStateSnapshot, SensorDescriptor

log = logging.getLogger(__name__)

SPEED_PRESETS = (0.25, 0.5, 1.0, 2.0, 4.0)


class SimState(str, Enum):
    STOPPED = "stopped"
    PLAYING = "playing"
    PAUSED = "paused"


class SimulationListener(Protocol):
    def on_robot_added(self, desc: RobotDescriptor) -> None: ...
    def on_robot_changed(self, desc: RobotDescriptor) -> None: ...
    def on_robot_removed(self, name: str) -> None: ...
    def on_robot_state(self, snap: RobotStateSnapshot) -> None: ...
    def on_sensor_data(self, robot: str, sensor: str, sensor_type: str, data) -> None: ...
    def on_clock(self, sim_time: float) -> None: ...
    def on_reset(self) -> None: ...


class SimulationManager:
    def __init__(self, config: Optional[AppConfig] = None, add_ground: bool = True):
        self.config = config or AppConfig()
        self.events = EventManager()
        self.engine = PhysicsEngine(self.config.physics)
        self.collisions = CollisionManager(self.engine.client)
        self.world = WorldManager(self.engine, self.collisions)
        self.loader = RobotLoader(self.config.package_paths)
        self.robots: dict[str, RobotInstance] = {}
        self.clock = SimulationClock(self.engine.timestep, self.config.physics.max_steps_per_frame)
        self.commands = CommandQueue()
        self.listeners: list[SimulationListener] = []
        self.state = SimState.STOPPED
        self.speed = 1.0
        self.image_backend: Optional[Callable] = None
        self.quit_requested = False

        # publication rates; listeners such as the ROS 2 bridge set them
        self.state_rate_hz = 0.0
        self.clock_rate_hz = 0.0
        self._next_state_time = 0.0
        self._next_clock_time = 0.0
        self._notified_versions: dict[str, int] = {}

        # statistics
        self.last_update_steps = 0
        self.last_physics_wall_time = 0.0
        self.cmd_vel_timeout = self.config.ros2.cmd_vel_timeout_s
        if add_ground:
            self.world.add(default_ground())

    @property
    def sim_time(self) -> float:
        return self.engine.sim_time

    @property
    def timestep(self) -> float:
        return self.engine.timestep

    @property
    def is_playing(self) -> bool:
        return self.state is SimState.PLAYING

    def add_listener(self, listener: SimulationListener) -> None:
        self.listeners.append(listener)
        for r in self.robots.values():
            listener.on_robot_added(self.describe(r))
            self._notified_versions[r.name] = r.version

    def remove_listener(self, listener: SimulationListener) -> None:
        if listener in self.listeners:
            self.listeners.remove(listener)

    def _notify(self, method: str, *args) -> None:
        for l in self.listeners:
            try:
                getattr(l, method)(*args)
            except Exception:
                log.exception("Simulation listener %s.%s failed", type(l).__name__, method)

    def play(self) -> None:
        if self.state is not SimState.PLAYING:
            self.clock.reset()
            self.state = SimState.PLAYING
            self.events.emit("sim_state", self.state)

    def pause(self) -> None:
        if self.state is SimState.PLAYING:
            self.state = SimState.PAUSED
            self.events.emit("sim_state", self.state)

    def stop(self) -> None:
        self.state = SimState.STOPPED
        self.reset()
        self.events.emit("sim_state", self.state)

    def set_speed(self, speed: float) -> None:
        if not math.isfinite(speed) or speed <= 0:
            raise ValueError("simulation speed must be > 0")
        self.speed = min(max(float(speed), 0.01), 16.0)
        self.events.emit("sim_speed", self.speed)

    def reset(self) -> None:
        self.engine.reset_time()
        self.clock.reset()
        self.world.reset()
        for r in self.robots.values():
            r.reset()
        self._next_state_time = 0.0
        self._next_clock_time = 0.0
        self._notify("on_reset")
        self.events.emit("sim_reset")
        log.info("Simulation reset")

    def set_physics_config(self) -> None:
        """Apply the changed PhysicsConfig (timestep, gravity, solver and other values)."""
        self.engine.apply_config()
        self.clock.timestep = self.engine.timestep
        self.clock.max_steps = self.config.physics.max_steps_per_frame
        self.collisions.set_global_enabled(self.config.physics.collision_detection)

    def update(self, wall_dt: float) -> int:
        """Move the simulation forward by `wall_dt` seconds of wall-clock time.

        The simulation speed multiplies this time. Return the number of physics steps.
        """
        self.process_commands()
        self._check_structure_changes()
        n = self.clock.advance(wall_dt, self.speed) if self.is_playing else 0
        t0 = time.perf_counter()
        for _ in range(n):
            self.step_once()
        self.last_update_steps = n
        self.last_physics_wall_time = time.perf_counter() - t0
        return n

    def single_step(self) -> None:
        if self.state is SimState.PLAYING:
            self.pause()
        if self.state is SimState.STOPPED:
            self.state = SimState.PAUSED
        self.step_once()

    def step_once(self) -> None:
        t_prev = self.engine.sim_time
        for r in self.robots.values():
            if r.config.diff_drive.enabled:
                r.drive.check_timeout(t_prev, self.cmd_vel_timeout)
            r.motors.apply()
        self.engine.step()
        t = self.engine.sim_time
        for r in self.robots.values():
            for s in r.sensors.values():
                if s.due(t):
                    data = s.tick(t, self.engine.timestep)
                    if data is not None and self.listeners:
                        self._notify("on_sensor_data", r.name, s.name, s.TYPE, data)
        if self.listeners:
            if self.state_rate_hz > 0 and t + 1e-9 >= self._next_state_time:
                self._next_state_time = max(self._next_state_time + 1.0 / self.state_rate_hz, t)
                for r in self.robots.values():
                    self._notify("on_robot_state", self.snapshot(r))
            if self.clock_rate_hz > 0 and t + 1e-9 >= self._next_clock_time:
                self._next_clock_time = max(self._next_clock_time + 1.0 / self.clock_rate_hz, t)
                self._notify("on_clock", t)

    def publish_state_now(self) -> None:
        """Publish the state now. While the simulation is paused, tools then still see the robot."""
        for r in self.robots.values():
            self._notify("on_robot_state", self.snapshot(r))
        self._notify("on_clock", self.engine.sim_time)

    def process_commands(self) -> None:
        for cmd in self.commands.drain():
            try:
                self._execute(cmd)
            except Exception as exc:  # send the error to the caller; the loop continues
                log.error("Command %s failed: %s", type(cmd).__name__, exc)
                if cmd.reply:
                    cmd.reply.set(False, str(exc))

    def _execute(self, cmd: Command) -> None:
        ok, msg, value = True, "", None
        if isinstance(cmd, JointTargetsCommand):
            ok, msg = self.apply_joint_targets(cmd)
        elif isinstance(cmd, CmdVelCommand):
            r = self.robots.get(cmd.robot)
            if r is None:
                ok, msg = False, f"unknown robot '{cmd.robot}'"
            elif not r.config.diff_drive.enabled:
                ok, msg = False, f"robot '{cmd.robot}' has no differential drive configured"
            else:
                errors = r.drive.validate()
                if errors:
                    ok, msg = False, "; ".join(errors)
                else:
                    r.drive.command(cmd.linear_x, cmd.angular_z, self.sim_time)
        elif isinstance(cmd, SimControlCommand):
            a = cmd.action
            if a == "play":
                self.play()
            elif a == "pause":
                self.pause()
            elif a == "stop":
                self.stop()
            elif a == "reset":
                self.reset()
            elif a == "step":
                n = max(1, int(cmd.value or 1))
                if self.state is SimState.PLAYING:
                    ok, msg = False, "stepping requires a paused simulation"
                else:
                    for _ in range(n):
                        self.single_step()
            elif a == "quit":
                self.quit_requested = True
            elif a == "speed":
                self.set_speed(cmd.value)
            else:
                ok, msg = False, f"unknown action '{a}'"
        elif isinstance(cmd, SpawnRobotCommand):
            value = self._spawn_from_command(cmd)
        elif isinstance(cmd, DeleteRobotCommand):
            if cmd.name not in self.robots:
                ok, msg = False, f"robot '{cmd.name}' does not exist"
            else:
                self.remove_robot(cmd.name)
        else:
            ok, msg = False, f"unsupported command {type(cmd).__name__}"
        if not ok:
            log.warning("%s", msg)
        if cmd.reply:
            cmd.reply.set(ok, msg, value)

    def apply_joint_targets(self, cmd: JointTargetsCommand) -> tuple[bool, str]:
        r = self.robots.get(cmd.robot)
        if r is None:
            return False, f"unknown robot '{cmd.robot}'"
        problems = []

        def field(arr, i):
            if i < len(arr):
                v = arr[i]
                return None if v is None or math.isnan(v) else float(v)
            return None

        for i, name in enumerate(cmd.names):
            m = r.motors.get(name)
            if m is None:
                problems.append(f"joint '{name}' has no motor")
                continue
            pos, vel, eff = field(cmd.positions, i), field(cmd.velocities, i), field(cmd.efforts, i)
            if pos is not None:
                m.set_position(pos)
            elif vel is not None:
                m.set_velocity(vel)
            elif eff is not None:
                m.set_effort(eff)
            else:
                problems.append(f"no target given for joint '{name}'")
        return (not problems), "; ".join(problems)

    def _spawn_from_command(self, cmd: SpawnRobotCommand) -> str:
        path = cmd.urdf_path
        if not path and cmd.urdf_string:
            CACHE_DIR.mkdir(parents=True, exist_ok=True)
            digest = hashlib.sha1(cmd.urdf_string.encode()).hexdigest()[:16]
            p = CACHE_DIR / f"spawned_{digest}.urdf"
            p.write_text(cmd.urdf_string)
            path = str(p)
        if not path:
            raise ValueError("no URDF given")
        name = cmd.name
        if name and name in self.robots and not cmd.allow_renaming:
            raise ValueError(f"robot name '{name}' is already taken")
        cfg = RobotConfig(name=name or "", urdf_path=path, position=tuple(cmd.position),
                          rpy=quat_to_rpy(cmd.orientation))
        r = self.spawn_robot(cfg)
        return r.name

    def unique_robot_name(self, base: str) -> str:
        base = sanitize_name(base)
        if base not in self.robots:
            return base
        i = 1
        while f"{base}_{i}" in self.robots:
            i += 1
        return f"{base}_{i}"

    def spawn_robot(self, config: RobotConfig) -> RobotInstance:
        """Import a URDF robot. If this fails, raise URDFError or RuntimeError with a clear message."""
        if not config.urdf_path:
            raise URDFError("No URDF path given")
        loaded = self.loader.load(config.urdf_path)
        config.urdf_path = str(Path(config.urdf_path).expanduser().resolve())
        config.name = self.unique_robot_name(config.name or loaded.model.name)
        if loaded.model.root_link == "world" and not config.fixed_base:
            config.fixed_base = True
            log.info("Robot '%s': root link is 'world'; loading with a fixed base", config.name)
        all_frames = set(loaded.model.links) | {s.name + "_link" for s in config.sensors}
        if not config.tf_prefix:
            for other in self.robots.values():
                other_frames = {other.tf_prefix + l for l in other.model.links}
                if all_frames & other_frames:
                    config.tf_prefix = f"{config.name}/"
                    log.warning("Robot '%s' shares frame names with '%s'; using TF prefix '%s' to avoid "
                                "conflicting TF frames", config.name, other.name, config.tf_prefix)
                    break
        robot = RobotInstance(config, loaded, self.engine, frame_checker=self.frame_owner)
        robot.image_backend = self.image_backend
        self.robots[config.name] = robot
        for w in loaded.warnings:
            log.warning("[%s] %s", config.name, w)
        log.info("Imported robot '%s' from %s: %d links, %d joints (%d movable)", config.name,
                 config.urdf_path, len(robot.model.links), len(robot.model.joints),
                 len(robot.model.movable_joints))
        self._notified_versions[robot.name] = robot.version
        self._notify("on_robot_added", self.describe(robot))
        self.events.emit("robot_added", robot)
        return robot

    def frame_owner(self, frame: str, exclude_robot: str = "") -> Optional[str]:
        """Return the name of the robot that publishes TF frame `frame`, or None."""
        for r in self.robots.values():
            if r.name == exclude_robot:
                continue
            if frame.startswith(r.tf_prefix) and frame[len(r.tf_prefix):] in r.model.links:
                return r.name
            if any(s.frame_id == frame for s in r.sensors.values()):
                return r.name
        return None

    def remove_robot(self, name: str) -> None:
        r = self.robots.pop(name, None)
        if r is None:
            return
        r.remove()
        self._notified_versions.pop(name, None)
        self._notify("on_robot_removed", name)
        self.events.emit("robot_removed", name)
        log.info("Removed robot '%s'", name)

    def reload_robot(self, name: str) -> RobotInstance:
        """Load the robot again and keep its configuration.

        Do this after a change to a load-time option, for example fixed base or self-collision.
        """
        r = self.robots[name]
        cfg = replace(r.config, motors=list(r.config.motors), sensors=list(r.config.sensors))
        self.remove_robot(name)
        cfg.name = name
        return self.spawn_robot(cfg)

    def set_image_backend(self, backend: Optional[Callable]) -> None:
        self.image_backend = backend
        for r in self.robots.values():
            r.image_backend = backend

    def _check_structure_changes(self) -> None:
        for r in self.robots.values():
            if self._notified_versions.get(r.name) != r.version:
                self._notified_versions[r.name] = r.version
                self._notify("on_robot_changed", self.describe(r))

    def notify_robot_changed(self, robot: RobotInstance) -> None:
        robot.version += 1

    def add_object(self, config: EnvObjectConfig):
        obj = self.world.add(config)
        self.events.emit("object_added", obj)
        return obj

    def remove_object(self, name: str) -> None:
        self.world.remove(name)
        self.events.emit("object_removed", name)

    def describe(self, r: RobotInstance) -> RobotDescriptor:
        joints = tuple(JointDescriptor(j.name, j.type, j.parent, j.child, j.origin.matrix())
                       for j in r.model.iter_joints_depth_first())
        sensors = tuple(SensorDescriptor(s.name, s.TYPE, s.config.link, s.frame_id, s.mount_transform.copy(),
                                         s.optical_frame_id if isinstance(s, Camera) else None, s.config.topic,
                                         s.frame_override)
                        for s in r.sensors.values())
        oc = r.config.odometry
        rc = r.config.ros
        ns = f"/{r.name}" if rc.namespace is None else ("/" + rc.namespace.strip("/") if rc.namespace.strip("/") else "")
        return RobotDescriptor(
            name=r.name, version=r.version, tf_prefix=r.tf_prefix, root_link=r.model.root_link,
            links=tuple(r.model.links), joints=joints,
            movable_joints=tuple(r.physics.movable_joints), sensors=sensors,
            fixed_base=r.config.fixed_base, odom_enabled=oc.enabled, odom_frame=oc.odom_frame or "odom",
            base_frame=r.base_frame_link, odom_publish_world_tf=oc.publish_world_tf,
            odom_origin=r.odom_origin.copy(), has_drive=r.config.diff_drive.enabled,
            ros_enabled=rc.enabled, namespace=ns, publish_tf=rc.publish_tf,
            joint_states_topic=rc.joint_states_topic, cmd_vel_topic=rc.cmd_vel_topic,
            joint_commands_topic=rc.joint_commands_topic, odom_topic=oc.topic or "odom",
            odom_child_frame=oc.child_frame)

    def snapshot(self, r: RobotInstance) -> RobotStateSnapshot:
        js = r.physics.joint_states()
        eff = js.effort.copy()
        for k, name in enumerate(js.names):
            eff[k] = r.motors.effort_of(name, eff[k])
        wt = r.physics.update_link_transforms()
        v, w = r.base_twist()
        return RobotStateSnapshot(robot=r.name, stamp=self.engine.sim_time, joint_names=list(js.names),
                                  position=js.position.copy(), velocity=js.velocity.copy(), effort=eff,
                                  link_transforms={k: T.copy() for k, T in wt.items()},
                                  base_linear_velocity=v, base_angular_velocity=w)

    def clear(self) -> None:
        for name in list(self.robots):
            self.remove_robot(name)
        self.world.clear()

    def shutdown(self) -> None:
        self.listeners.clear()
        self.engine.disconnect()

    def run_for(self, seconds: float) -> None:
        """Run the physics for `seconds` of simulation time, as fast as possible."""
        if self.state is not SimState.PLAYING:
            self.play()
        n = int(round(seconds / self.engine.timestep))
        self.process_commands()
        for _ in range(n):
            self.step_once()
