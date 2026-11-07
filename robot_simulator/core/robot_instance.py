"""A robot in the simulation: description, physics, motors and sensors."""
from __future__ import annotations

import logging
import re
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ..control.diff_drive import DiffDriveConfig, DiffDriveController
from ..motors.motor_base import MotorConfig
from ..physics.motor_controller import MotorController
from ..physics.physics_engine import PhysicsEngine
from ..physics.robot_physics import RobotPhysics
from ..robot.robot_loader import LoadedRobotDescription
from ..sensors import Sensor, SensorConfig, create_sensor
from ..utils.transforms import invert_transform, make_transform, pose_to_matrix, rpy_to_quat

log = logging.getLogger(__name__)

NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


def sanitize_name(name: str) -> str:
    """Return a robot name that ROS accepts. Topic namespaces use this name."""
    s = re.sub(r"[^A-Za-z0-9_]", "_", name.strip()) or "robot"
    if not s[0].isalpha():
        s = "r_" + s
    return s


@dataclass
class OdometryConfig:
    enabled: bool = False
    odom_frame: str = "odom"
    base_frame: str = ""                # "" = URDF root link
    publish_world_tf: bool = False      # also publish world -> odom (set false if SLAM or AMCL publishes map -> odom)
    child_frame: str = ""               # child_frame_id of the message; "" = base_frame with the TF prefix
    topic: str = "odom"                 # relative to the robot namespace, or absolute ("/...")
    origin: str = "spawn"               # "spawn" = odom frame at the initial pose; "world" = world frame


@dataclass
class RosRobotConfig:
    """ROS 2 interface of the robot. Relative topic names are in the robot namespace."""
    enabled: bool = True                # False = the robot has no ROS 2 interface
    namespace: Optional[str] = None     # None = "/<robot name>", "" = root namespace, "/x" or "x" = /x
    publish_tf: bool = True             # set false if robot_state_publisher and a localizer publish the TF tree
    joint_states_topic: str = "joint_states"
    cmd_vel_topic: str = "cmd_vel"
    joint_commands_topic: str = "joint_commands"


@dataclass
class RobotConfig:
    name: str = ""                                                # "" = URDF robot name
    urdf_path: str = ""
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)      # m (URDF root link frame)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)           # rad
    fixed_base: bool = False
    self_collision: bool = False
    lateral_friction: Optional[float] = None                     # applied to all links
    link_friction: dict[str, float] = field(default_factory=dict)  # per-link override
    tf_prefix: str = ""
    initial_joint_positions: dict[str, float] = field(default_factory=dict)
    motors: list[MotorConfig] = field(default_factory=list)
    sensors: list[SensorConfig] = field(default_factory=list)
    diff_drive: DiffDriveConfig = field(default_factory=DiffDriveConfig)
    odometry: OdometryConfig = field(default_factory=OdometryConfig)
    ros: RosRobotConfig = field(default_factory=RosRobotConfig)


class RobotInstance:
    def __init__(self, config: RobotConfig, loaded: LoadedRobotDescription, engine: PhysicsEngine,
                 frame_checker=None):
        self.config = config
        self.loaded = loaded
        self.model = loaded.model
        self.engine = engine
        self.image_backend = None   # the simulation manager sets this when a renderer is available
        self.frame_checker = frame_checker   # callable(frame, robot_name) -> owner robot name or None
        self.physics = RobotPhysics(engine, self.model, str(loaded.pybullet_urdf), config.position,
                                    rpy_to_quat(config.rpy), config.fixed_base, config.self_collision,
                                    None)
        self.apply_link_friction()
        self.motors = MotorController(self.physics)
        self.sensors: dict[str, Sensor] = {}
        self.drive = DiffDriveController(config.diff_drive, self.motors)
        self.odom_origin = self._odom_origin()
        self.version = 0     # increases when sensors, motors or frames change

        # Apply the configuration from the project file. Log errors and continue.
        stored_motors, config.motors = list(config.motors), []
        for mc in stored_motors:
            try:
                self.assign_motor(mc)
            except ValueError as exc:
                log.error("Robot '%s': motor configuration for '%s' rejected: %s", config.name, mc.joint, exc)
        stored_sensors, config.sensors = list(config.sensors), []
        for sc in stored_sensors:
            try:
                self.add_sensor(sc)
            except ValueError as exc:
                log.error("Robot '%s': sensor '%s' rejected: %s", config.name, sc.name, exc)
        self.reset()

    @property
    def name(self) -> str:
        return self.config.name

    @property
    def tf_prefix(self) -> str:
        return self.config.tf_prefix

    @property
    def gravity(self):
        return self.engine.config.gravity

    @property
    def base_frame_link(self) -> str:
        bf = self.config.odometry.base_frame
        return bf if bf in self.model.links else self.model.root_link

    def apply_link_friction(self) -> None:
        if self.config.lateral_friction is not None:
            self.physics.set_friction(self.config.lateral_friction)
        for link, mu in self.config.link_friction.items():
            if link in self.physics.link_index:
                self.physics.set_link_friction(link, mu)
            else:
                log.warning("Robot '%s': friction override for unknown link '%s' ignored", self.config.name, link)

    def assign_motor(self, mc: MotorConfig):
        motor = self.motors.assign(mc)
        self.config.motors = [m for m in self.config.motors if m.joint != mc.joint] + [mc]
        motor.config.target_position = self.config.initial_joint_positions.get(mc.joint, mc.target_position)
        self.version += 1
        return motor

    def remove_motor(self, joint: str) -> None:
        self.motors.remove(joint)
        self.config.motors = [m for m in self.config.motors if m.joint != joint]
        self.version += 1

    def add_sensor(self, sc: SensorConfig) -> Sensor:
        if sc.link not in self.model.links:
            raise ValueError(f"link '{sc.link}' does not exist")
        if not NAME_RE.match(sc.name):
            raise ValueError(f"sensor name '{sc.name}' must match [A-Za-z][A-Za-z0-9_]*")
        if sc.name in self.sensors:
            raise ValueError(f"a sensor named '{sc.name}' already exists")
        if not sc.frame_id and sc.name + "_link" in self.model.links:
            raise ValueError(f"sensor frame '{sc.name}_link' would duplicate a URDF link frame "
                             f"(set the sensor frame_id to use that link)")
        frame = f"{self.tf_prefix}{sc.name}_link"
        owner = self.frame_checker(frame, self.name) if (self.frame_checker and not sc.frame_id) else None
        if owner:
            raise ValueError(f"TF frame '{frame}' is already used by robot '{owner}'; "
                             f"choose another sensor name or set a TF prefix")
        sensor = create_sensor(sc, self)
        sensor.next_update_time = self.engine.sim_time
        self.sensors[sc.name] = sensor
        self.config.sensors.append(sc)
        self.version += 1
        return sensor

    def remove_sensor(self, name: str) -> None:
        s = self.sensors.pop(name, None)
        if s is not None:
            self.config.sensors = [c for c in self.config.sensors if c is not s.config]
            self.version += 1

    def unique_sensor_name(self, base: str) -> str:
        name, i = base, 1
        while (name in self.sensors or name + "_link" in self.model.links or
               (self.frame_checker and self.frame_checker(f"{self.tf_prefix}{name}_link", self.name))):
            name = f"{base}_{i}"
            i += 1
        return name

    def initial_joint_positions(self) -> dict[str, float]:
        out = {}
        for j in self.model.movable_joints:
            q = float(self.config.initial_joint_positions.get(j.name, 0.0))
            if j.has_position_limits:
                q = min(max(q, j.limit.lower), j.limit.upper)  # type: ignore[union-attr]
            out[j.name] = q
        return out

    def reset(self) -> None:
        q0 = self.initial_joint_positions()
        self.physics.reset_state(self.config.position, rpy_to_quat(self.config.rpy), q0)
        self.motors.reset(q0)
        self.drive.stop()
        self.drive.last_command_time = -1.0
        for s in self.sensors.values():
            s.reset()
            s.next_update_time = self.engine.sim_time
        self.odom_origin = self._odom_origin()
        self.physics.update_link_transforms(force=True)

    def _odom_origin(self) -> np.ndarray:
        if self.config.odometry.origin == "world":
            return np.eye(4)            # ground truth in the world frame
        return make_transform(self.config.position, self.config.rpy)

    def base_twist(self) -> tuple[np.ndarray, np.ndarray]:
        """Return the linear and angular velocity of the base frame origin, in the world frame."""
        link = self.base_frame_link
        com, v_com, w = self.physics.link_velocity(link)
        p_link = self.physics.link_transform(link)[:3, 3]
        return v_com + np.cross(w, p_link - com), w

    def odometry_pose(self) -> np.ndarray:
        return invert_transform(self.odom_origin) @ self.physics.link_transform(self.base_frame_link)

    def remove(self) -> None:
        self.physics.remove()


def default_pose_matrix(cfg: RobotConfig) -> np.ndarray:
    return pose_to_matrix(cfg.position, rpy_to_quat(cfg.rpy))
