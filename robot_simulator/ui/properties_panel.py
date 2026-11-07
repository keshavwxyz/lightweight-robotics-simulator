"""Right panel: properties of the selected item.

robot  -> information, pose, load options, odometry, differential drive, sensors, motors
link   -> pose, inertial properties, geometry, attach sensor
joint  -> description, live state, motor configuration
sensor -> sensor configuration
object -> environment object editor
none   -> world / physics / ROS 2 settings
"""
from __future__ import annotations

import math

import imgui
import numpy as np

from ..motors.motor_base import ControlMode, MotorConfig
from ..physics.motor_controller import motor_type_for_joint
from ..sensors import SENSOR_TYPES
from ..utils.transforms import matrix_to_rpy
from .widgets import (ERROR, MUTED, OK, WARN, combo_value, danger_button, float_input, fmt_vec, heading,
                      help_marker, kv, vec3)

SENSOR_DEFAULT_NAMES = {"camera": "camera", "imu": "imu", "lidar_2d": "lidar"}


class PropertiesPanel:
    def __init__(self, ui):
        self.ui = ui
        self.new_sensor_type = 0
        self.new_sensor_link = ""

    def draw(self) -> None:
        imgui.text_colored("PROPERTIES", *MUTED)
        imgui.separator()
        imgui.push_item_width(imgui.get_window_width() * 0.48)
        try:
            self._dispatch()
        finally:
            imgui.pop_item_width()

    def _dispatch(self) -> None:
        ui = self.ui
        sel = ui.selection
        robot = ui.selected_robot()
        if sel.kind == "object" and sel.name in ui.sim.world.objects:
            self._object(sel.name)
        elif robot is None:
            if sel.kind:
                ui.select()
            self._world()
        elif sel.kind == "robot":
            self._robot(robot)
        elif sel.kind == "link" and sel.name in robot.model.links:
            self._link(robot, sel.name)
        elif sel.kind == "joint" and sel.name in robot.model.joints:
            self._joint(robot, robot.model.joints[sel.name])
        elif sel.kind == "sensor" and sel.name in robot.sensors:
            self._sensor(robot, robot.sensors[sel.name])
        else:
            ui.select("robot", robot.name)

    def _world(self) -> None:
        sim = self.ui.sim
        pc = sim.config.physics
        heading("Physics")
        changed = False
        c, v = float_input("Frequency [Hz]", pc.frequency_hz, "%.1f", 10.0, 2000.0)
        if c:
            pc.frequency_hz, changed = v, True
        imgui.text_colored(f"timestep = {pc.timestep * 1000:.3f} ms (fixed)", *MUTED)
        c, g = vec3("Gravity [m/s^2]", pc.gravity, 0.05)
        if c:
            pc.gravity, changed = g, True
        c, it = imgui.input_int("Solver iterations", pc.solver_iterations)
        if c:
            pc.solver_iterations, changed = max(1, min(it, 500)), True
        c, on = imgui.checkbox("Collision detection", pc.collision_detection)
        if c:
            pc.collision_detection, changed = on, True
        c, v = float_input("Default friction", pc.default_friction, "%.3f", 0.0, 10.0)
        if c:
            pc.default_friction, changed = v, True
        c, v = float_input("Default restitution", pc.default_restitution, "%.3f", 0.0, 1.0)
        if c:
            pc.default_restitution, changed = v, True
        c, v = float_input("Contact ERP", pc.contact_erp, "%.3f", 0.0, 1.0)
        if c:
            pc.contact_erp, changed = v, True
        c, it = imgui.input_int("Max steps / frame", pc.max_steps_per_frame)
        if c:
            pc.max_steps_per_frame, changed = max(1, min(it, 1000)), True
        help_marker("Upper bound on physics steps per rendered frame. If physics cannot keep up, "
                    "the simulation runs slower than real time instead of freezing the UI.")
        if changed:
            sim.set_physics_config()
            for o in sim.world.objects:
                sim.world.apply_config(o)
            self.ui.project.dirty = True

        heading("ROS 2")
        b = self.ui.bridge
        col = OK if b.status == "connected" else (ERROR if b.status == "error" else MUTED)
        kv("Status", b.status, col)
        if b.error and b.status != "connected":
            imgui.push_text_wrap_pos(0)
            imgui.text_colored(b.error, *WARN)
            imgui.pop_text_wrap_pos()
        rc = sim.config.ros2
        running = b.status == "connected"
        if not running:
            c, rc.node_name = imgui.input_text("Node name", rc.node_name, 128)
            c, did = imgui.input_int("Domain ID (-1 = env)", -1 if rc.domain_id is None else rc.domain_id)
            if c:
                rc.domain_id = None if did < 0 else min(did, 232)
            _, rc.cmd_vel_stamped = imgui.checkbox("cmd_vel as TwistStamped", rc.cmd_vel_stamped)
        c, v = float_input("State rate [Hz]", rc.state_publish_rate_hz, "%.1f", 1.0, 1000.0)
        if c:
            rc.state_publish_rate_hz = v
            if running:
                sim.state_rate_hz = v
        help_marker("Publication rate of joint_states, tf and odom (simulation time).")
        c, rc.publish_clock = imgui.checkbox("Publish /clock", rc.publish_clock)
        if c and running:
            sim.clock_rate_hz = rc.clock_publish_rate_hz if rc.publish_clock else 0.0
        c, v = float_input("cmd_vel timeout [s]", rc.cmd_vel_timeout_s, "%.2f", 0.0, 60.0)
        if c:
            rc.cmd_vel_timeout_s = sim.cmd_vel_timeout = v
        if b.available():
            if imgui.button("Disable bridge" if running else "Enable bridge"):
                self.ui.set_ros_enabled(not running)
        else:
            imgui.text_colored("rclpy not found - source your ROS 2 setup.bash", *WARN)

        heading("Scene")
        kv("Robots", str(len(sim.robots)))
        kv("Objects", str(len(sim.world.objects)))
        imgui.text_colored("Select an item in the scene hierarchy or click it in the viewport.", *MUTED)

    def _robot(self, r) -> None:
        ui, sim, cfg = self.ui, self.ui.sim, r.config
        heading("Robot")
        kv("Name", r.name)
        kv("URDF", r.model.name)
        imgui.push_text_wrap_pos(0)
        imgui.text_colored(cfg.urdf_path, *MUTED)
        imgui.pop_text_wrap_pos()
        kv("Links / joints", f"{len(r.model.links)} / {len(r.model.joints)} ({len(r.model.movable_joints)} movable)")
        kv("Total mass", f"{r.model.total_mass():.3f} kg")
        kv("Root link", r.model.root_link)
        kv("TF prefix", cfg.tf_prefix or "(none)")
        if r.loaded.warnings:
            if imgui.tree_node(f"Import warnings ({len(r.loaded.warnings)})"):
                imgui.push_text_wrap_pos(0)
                for w in r.loaded.warnings:
                    imgui.bullet_text(w)
                imgui.pop_text_wrap_pos()
                imgui.tree_pop()

        heading("Initial pose")
        c1, cfg.position = vec3("Position [m]", cfg.position, 0.01)
        c2, cfg.rpy = vec3("RPY [rad]", cfg.rpy, 0.01)
        if c1 or c2:
            ui.project.dirty = True
        if imgui.button("Move robot to initial pose"):
            r.reset()
        imgui.same_line()
        if imgui.button("Use current pose"):
            T = r.physics.link_transform(r.model.root_link)
            cfg.position = tuple(float(x) for x in T[:3, 3])
            cfg.rpy = matrix_to_rpy(T[:3, :3])
            ui.project.dirty = True

        heading("Physics options")
        c, cfg.fixed_base = imgui.checkbox("Fixed base", cfg.fixed_base)
        reload_needed = c
        c, cfg.self_collision = imgui.checkbox("Self collision", cfg.self_collision)
        reload_needed |= c
        if reload_needed:
            ui.project.dirty = True
            name = r.name
            try:
                sim.reload_robot(name)
                ui.select("robot", name)
            except Exception as exc:
                ui.report_error(f"Reload failed: {exc}")
            return
        use_mu = cfg.lateral_friction is not None
        c, use_mu = imgui.checkbox("Override friction (all links)", use_mu)
        if c:
            cfg.lateral_friction = 0.8 if use_mu else None
            r.apply_link_friction()
        if cfg.lateral_friction is not None:
            c, v = float_input("Friction coefficient", cfg.lateral_friction, "%.3f", 0.0, 10.0)
            if c:
                cfg.lateral_friction = v
                r.apply_link_friction()

        heading("Odometry")
        oc = cfg.odometry
        c, oc.enabled = imgui.checkbox("Publish odometry", oc.enabled)
        changed = c
        if oc.enabled:
            c, oc.odom_frame = imgui.input_text("Odom frame", oc.odom_frame, 64)
            changed |= c
            c, oc.base_frame = combo_value("Base frame", oc.base_frame or r.model.root_link, list(r.model.links))
            changed |= c
            c, oc.publish_world_tf = imgui.checkbox("Publish world -> odom TF", oc.publish_world_tf)
            changed |= c
            help_marker("Disable when SLAM / AMCL publishes map -> odom, so 'odom' keeps a single parent.")
        if changed:
            r.version += 1
            ui.project.dirty = True

        self._drive(r)

        heading("Sensors")
        for sname, s in list(r.sensors.items()):
            if imgui.selectable(f"{s.TYPE}: {sname} @ {s.config.link} ({s.config.rate_hz:g} Hz)")[0]:
                ui.select("sensor", r.name, sname)
        self._add_sensor_widget(r, None)

        heading("Motors")
        if not r.motors.motors:
            imgui.text_colored("No motors. Select a joint to assign one.", *MUTED)
        for jname, m in r.motors.motors.items():
            if imgui.selectable(f"{jname}: {m.TYPE}, {m.config.mode}")[0]:
                ui.select("joint", r.name, jname)

        imgui.spacing()
        imgui.separator()
        if danger_button("Remove robot"):
            ui.remove_robot(r.name)

    def _drive(self, r) -> None:
        dc = r.config.diff_drive
        heading("Differential drive (cmd_vel)")
        c, dc.enabled = imgui.checkbox("Enable", dc.enabled)
        if c:
            r.version += 1
            self.ui.project.dirty = True
        if not dc.enabled:
            return
        rot_joints = [j.name for j in r.model.movable_joints if j.is_rotational]
        for side, lst in (("Left", dc.left_joints), ("Right", dc.right_joints)):
            imgui.text(f"{side} wheels:")
            for jn in list(lst):
                imgui.same_line()
                if imgui.small_button(f"{jn} x##{side}{jn}"):
                    lst.remove(jn)
            avail = [j for j in rot_joints if j not in dc.left_joints + dc.right_joints]
            if avail:
                imgui.push_item_width(160)
                c, pick = imgui.combo(f"##add{side}", 0, ["add joint..."] + avail)
                imgui.pop_item_width()
                if c and pick > 0:
                    jn = avail[pick - 1]
                    lst.append(jn)
                    if r.motors.get(jn) is None:
                        j = r.model.joints[jn]
                        eff = j.limit.effort if j.limit and j.limit.effort > 0 else 10.0
                        vel = j.limit.velocity if j.limit and j.limit.velocity > 0 else 20.0
                        r.assign_motor(MotorConfig(joint=jn, type="rotational", mode="velocity",
                                                   max_effort=eff, max_velocity=vel))
        c, dc.wheel_radius = float_input("Wheel radius [m]", dc.wheel_radius, "%.4f", 1e-4)
        c, dc.wheel_separation = float_input("Wheel separation [m]", dc.wheel_separation, "%.4f", 1e-4)
        _, dc.invert_left = imgui.checkbox("Invert left", dc.invert_left)
        imgui.same_line()
        _, dc.invert_right = imgui.checkbox("Invert right", dc.invert_right)
        _, dc.max_linear_velocity = float_input("Max linear [m/s]", dc.max_linear_velocity, "%.3f", 0.0)
        _, dc.max_angular_velocity = float_input("Max angular [rad/s]", dc.max_angular_velocity, "%.3f", 0.0)
        errors = r.drive.validate()
        for e in errors:
            imgui.text_colored(e, *WARN)
        if not errors:
            imgui.text_colored(f"cmd: v = {r.drive.v:.2f} m/s, w = {r.drive.w:.2f} rad/s", *MUTED)
            if imgui.button("Test: forward 0.3 m/s"):
                r.drive.command(0.3, 0.0, self.ui.sim.sim_time)
            imgui.same_line()
            if imgui.button("Stop##drive"):
                r.drive.stop()

    def _add_sensor_widget(self, r, link: str | None) -> None:
        types = list(SENSOR_TYPES)
        imgui.push_item_width(110)
        _, self.new_sensor_type = imgui.combo("##stype", self.new_sensor_type, ["Camera", "IMU", "LiDAR 2D"])
        imgui.pop_item_width()
        if link is None:
            imgui.same_line()
            imgui.push_item_width(150)
            links = list(r.model.links)
            if self.new_sensor_link not in links:
                self.new_sensor_link = r.model.root_link
            _, self.new_sensor_link = combo_value("##slink", self.new_sensor_link, links)
            imgui.pop_item_width()
            link = self.new_sensor_link
        imgui.same_line()
        if imgui.button("Add sensor"):
            stype = types[self.new_sensor_type]
            cls = SENSOR_TYPES[stype][1]
            name = r.unique_sensor_name(SENSOR_DEFAULT_NAMES[stype])
            try:
                r.add_sensor(cls(name=name, link=link))
                self.ui.select("sensor", r.name, name)
                self.ui.project.dirty = True
            except ValueError as exc:
                self.ui.report_error(str(exc))

    def _link(self, r, name: str) -> None:
        link = r.model.links[name]
        heading(f"Link: {name}")
        pj = r.model.parent_joint_of.get(name)
        kv("Parent joint", pj.name if pj else "(root)")
        kv("Child joints", ", ".join(j.name for j in r.model.child_joints_of[name]) or "-")
        T = r.physics.link_transform(name)
        kv("World position", fmt_vec(T[:3, 3]))
        kv("World RPY", fmt_vec(matrix_to_rpy(T[:3, :3])))
        heading("Inertial")
        if link.inertial:
            kv("Mass", f"{link.inertial.mass:.4f} kg")
            kv("COM origin", fmt_vec(link.inertial.origin.xyz))
            ixx, ixy, ixz, iyy, iyz, izz = link.inertial.inertia
            kv("Ixx Iyy Izz", f"{ixx:.3g} {iyy:.3g} {izz:.3g}")
            kv("Ixy Ixz Iyz", f"{ixy:.3g} {ixz:.3g} {iyz:.3g}")
        else:
            imgui.text_colored("No <inertial>: treated as a massless frame (1 g).", *MUTED)
        heading("Geometry")
        for i, v in enumerate(link.visuals):
            g = v.geometry
            desc = g.type if g.type != "mesh" else f"mesh {g.filename}"
            col = None if g.type != "mesh" or g.resolved_path else ERROR
            kv(f"Visual {i}", desc, col)
        for i, c in enumerate(link.collisions):
            g = c.geometry
            kv(f"Collision {i}", g.type if g.type != "mesh" else f"mesh {g.filename}")
        if not link.visuals and not link.collisions:
            imgui.text_colored("No geometry", *MUTED)
        heading("Attach sensor")
        self._add_sensor_widget(r, name)

    def _joint(self, r, j) -> None:
        ui = self.ui
        heading(f"Joint: {j.name}")
        kv("Type", j.type if j.type == j.original_type else f"{j.type} (URDF: {j.original_type}, unsupported)")
        kv("Parent", j.parent)
        kv("Child", j.child)
        kv("Origin xyz", fmt_vec(j.origin.xyz))
        kv("Origin rpy", fmt_vec(j.origin.rpy))
        if j.is_movable:
            kv("Axis", fmt_vec(j.axis))
        unit = "rad" if j.is_rotational else "m"
        if j.limit:
            if j.type != "continuous":
                kv("Limits", f"[{j.limit.lower:.4f}, {j.limit.upper:.4f}] {unit}")
            kv("Velocity limit", f"{j.limit.velocity:g} {unit}/s")
            kv("Effort limit", f"{j.limit.effort:g} {'N m' if j.is_rotational else 'N'}")
        if j.dynamics.damping or j.dynamics.friction:
            kv("Damping / friction", f"{j.dynamics.damping:g} / {j.dynamics.friction:g}")
        if j.mimic:
            kv("Mimics", f"{j.mimic.joint} x {j.mimic.multiplier:g} + {j.mimic.offset:g}")

        if not j.is_movable:
            imgui.spacing()
            imgui.text_colored("Fixed joints cannot receive motors.", *MUTED)
            return

        js = r.physics.joint_states()
        k = r.physics.index_of_movable[j.name]
        heading("State")
        kv("Position", f"{js.position[k]: .5f} {unit}")
        kv("Velocity", f"{js.velocity[k]: .5f} {unit}/s")
        kv("Effort", f"{r.motors.effort_of(j.name, js.effort[k]): .4f} {'N m' if j.is_rotational else 'N'}")
        q0 = r.config.initial_joint_positions.get(j.name, 0.0)
        c, q0 = float_input(f"Initial position [{unit}]", q0, "%.4f")
        if c:
            r.config.initial_joint_positions[j.name] = q0
            ui.project.dirty = True

        heading("Motor")
        if j.mimic:
            imgui.text_colored("Driven by its mimic leader joint.", *MUTED)
            return
        m = r.motors.get(j.name)
        mtype = motor_type_for_joint(j.type)
        if m is None:
            imgui.text_colored("No motor assigned (joint is passive).", *MUTED)
            if imgui.button(f"Assign {mtype} motor"):
                eff = j.limit.effort if j.limit and j.limit.effort > 0 else (10.0 if mtype == "rotational" else 100.0)
                vel = j.limit.velocity if j.limit and j.limit.velocity > 0 else 1.0
                mc = MotorConfig(joint=j.name, type=mtype, max_effort=eff, max_velocity=vel,
                                 target_position=float(js.position[k]))
                try:
                    r.assign_motor(mc).config.target_position = float(js.position[k])
                    ui.project.dirty = True
                except ValueError as exc:
                    ui.report_error(str(exc))
            return
        c = m.config
        eff_unit = "N m" if m.TYPE == "rotational" else "N"
        kv("Motor type", f"{m.TYPE} ({'revolute/continuous' if m.TYPE == 'rotational' else 'prismatic'})")
        modes = [cm.value for cm in ControlMode]
        changed, mode = combo_value("Control mode", c.mode, modes)
        if changed:
            m.set_mode(mode)
        ch = False
        cc, c.max_effort = float_input(f"Max {m.EFFORT_NAME} [{eff_unit}]", c.max_effort, "%.3f", 0.0)
        ch |= cc
        cc, c.max_velocity = float_input(f"Max velocity [{unit}/s]", c.max_velocity, "%.3f", 0.0)
        ch |= cc
        cc, c.position_gain = float_input("Position gain", c.position_gain, "%.4f", 0.0)
        ch |= cc
        cc, c.velocity_gain = float_input("Velocity gain", c.velocity_gain, "%.4f", 0.0)
        ch |= cc
        if ch:
            m.mark_dirty()
            ui.project.dirty = True
        imgui.spacing()
        if c.mode == "position":
            lo, hi = (j.limit.lower, j.limit.upper) if j.has_position_limits else (-math.pi, math.pi)
            if j.is_linear and not j.has_position_limits:
                lo, hi = -1.0, 1.0
            cc, v = imgui.slider_float(f"Target position [{unit}]", c.target_position, lo, hi, "%.4f")
            if cc:
                m.set_position(v)
        elif c.mode == "velocity":
            vm = c.max_velocity if c.max_velocity > 0 else 10.0
            cc, v = imgui.slider_float(f"Target velocity [{unit}/s]", c.target_velocity, -vm, vm, "%.4f")
            if cc:
                m.set_velocity(v)
        else:
            em = c.max_effort if c.max_effort > 0 else 10.0
            cc, v = imgui.slider_float(f"Target {m.EFFORT_NAME} [{eff_unit}]", c.target_effort, -em, em, "%.4f")
            if cc:
                m.set_effort(v)
        if imgui.button("Zero target"):
            {"position": m.set_position, "velocity": m.set_velocity, "effort": m.set_effort}[c.mode](0.0)
        imgui.same_line()
        if danger_button("Remove motor"):
            r.remove_motor(j.name)
            ui.project.dirty = True

    def _sensor(self, r, s) -> None:
        ui = self.ui
        c = s.config
        heading(f"Sensor: {s.name}")
        kv("Type", {"camera": "RGB camera", "imu": "Perfect IMU", "lidar_2d": "2D LiDAR"}[s.TYPE])
        kv("Frame", s.frame_id)
        kv("Updates", str(s.update_count))
        structural = False
        _, c.enabled = imgui.checkbox("Enabled", c.enabled)
        ch, link = combo_value("Parent link", c.link, list(r.model.links))
        if ch:
            c.link, structural = link, True
        ch1, c.xyz = vec3("Mount xyz [m]", c.xyz, 0.005)
        ch2, c.rpy = vec3("Mount rpy [rad]", c.rpy, 0.01)
        if ch1 or ch2:
            s.refresh_mount()
            structural = True
        cc, v = float_input("Rate [Hz]", c.rate_hz, "%.2f", 0.1, 1000.0)
        if cc:
            c.rate_hz = v
        cc, c.topic = imgui.input_text("Topic (blank = default)", c.topic, 256, imgui.INPUT_TEXT_ENTER_RETURNS_TRUE)
        structural |= cc

        if s.TYPE == "camera":
            heading("Camera")
            cc1, c.width = imgui.input_int("Width [px]", c.width)
            cc2, c.height = imgui.input_int("Height [px]", c.height)
            c.width, c.height = max(8, min(c.width, 4096)), max(8, min(c.height, 4096))
            cc, hfov = imgui.slider_float("Horizontal FOV [rad]", c.horizontal_fov, 0.1, 3.0, "%.3f")
            if cc:
                c.horizontal_fov = hfov
            imgui.text_colored(f"= {math.degrees(c.horizontal_fov):.1f} deg", *MUTED)
            _, c.near = float_input("Near clip [m]", c.near, "%.3f", 1e-4)
            _, c.far = float_input("Far clip [m]", c.far, "%.2f", c.near + 1e-3)
            key = (r.name, s.name)
            showing = key in ui.camera_windows
            if imgui.button("Hide image" if showing else "Show image"):
                (ui.camera_windows.discard if showing else ui.camera_windows.add)(key)
        elif s.TYPE == "lidar_2d":
            heading("LiDAR")
            cc, res = imgui.slider_float("Angular resolution [rad]", c.angular_resolution, 0.001, 0.1, "%.4f")
            if cc:
                c.angular_resolution = res
            imgui.text_colored(f"= {math.degrees(c.angular_resolution):.2f} deg, {s.num_beams} beams", *MUTED)
            _, c.range_min = float_input("Range min [m]", c.range_min, "%.3f", 0.0)
            _, c.range_max = float_input("Range max [m]", c.range_max, "%.2f", c.range_min + 1e-3)
            d = s.latest
            if d is not None:
                finite = np.isfinite(d.ranges)
                kv("Returns", f"{int(finite.sum())} / {len(d.ranges)}")
                if finite.any():
                    kv("Closest", f"{float(d.ranges[finite].min()):.3f} m")
        elif s.TYPE == "imu":
            d = s.latest
            heading("Measurement")
            if d is not None:
                kv("Orientation", fmt_vec(d.orientation, 4))
                kv("Ang. velocity", fmt_vec(d.angular_velocity) + " rad/s")
                kv("Lin. accel.", fmt_vec(d.linear_acceleration) + " m/s^2")
            else:
                imgui.text_colored("Run the simulation to produce measurements.", *MUTED)

        if structural:
            r.version += 1
            ui.project.dirty = True
        imgui.spacing()
        imgui.separator()
        if danger_button("Remove sensor"):
            r.remove_sensor(s.name)
            ui.camera_windows.discard((r.name, s.name))
            ui.select("robot", r.name)
            ui.project.dirty = True

    def _object(self, name: str) -> None:
        ui, sim = self.ui, self.ui.sim
        obj = sim.world.objects[name]
        c = obj.config
        heading(f"Object: {name}")
        new_name = imgui.input_text("Name", c.name, 64, imgui.INPUT_TEXT_ENTER_RETURNS_TRUE)
        if new_name[0] and new_name[1].strip():
            ui.select("object", "", sim.world.rename(name, new_name[1].strip()))
            return
        kv("Shape", c.shape)
        changed = False
        if c.shape != "plane":
            if c.shape == "box":
                cc, size = vec3("Size [m]", c.size, 0.01)
                if cc:
                    c.size, changed = tuple(max(1e-3, s) for s in size), True
            if c.shape in ("cylinder", "sphere"):
                cc, v = imgui.drag_float("Radius [m]", c.radius, 0.005, 1e-3, 100.0)
                if cc:
                    c.radius, changed = v, True
            if c.shape == "cylinder":
                cc, v = imgui.drag_float("Length [m]", c.length, 0.005, 1e-3, 100.0)
                if cc:
                    c.length, changed = v, True
        cc, c.position = vec3("Position [m]", c.position, 0.01)
        changed |= cc
        cc, c.rpy = vec3("RPY [rad]", c.rpy, 0.01)
        changed |= cc
        if c.shape != "plane":
            cc, m = float_input("Mass [kg] (0 = static)", c.mass, "%.3f", 0.0)
            if cc:
                c.mass, changed = m, True
        cc, col = imgui.color_edit4("Colour", *c.color)
        if cc:
            c.color = tuple(col)
        cc, c.collision_enabled = imgui.checkbox("Collision", c.collision_enabled)
        changed |= cc
        imgui.same_line()
        _, c.visual_enabled = imgui.checkbox("Visible", c.visual_enabled)
        use_mu = c.friction is not None
        cc, use_mu = imgui.checkbox("Override friction", use_mu)
        if cc:
            c.friction, changed = (sim.config.physics.default_friction if use_mu else None), True
        if c.friction is not None:
            cc, c.friction = float_input("Friction", c.friction, "%.3f", 0.0, 10.0)
            changed |= cc
        if changed:
            sim.world.apply_config(name)
            ui.project.dirty = True
        if not c.is_static:
            pos, _ = sim.engine.body_pose(obj.body)
            kv("Current position", fmt_vec(pos))
        imgui.spacing()
        imgui.separator()
        if danger_button("Delete object"):
            ui.remove_object(name)
