"""Physics engine, joints, limits and the two motor types."""
import numpy as np
import pybullet as p
import pytest

from robot_simulator.core.robot_instance import RobotConfig
from robot_simulator.environment.environment_objects import EnvObjectConfig
from robot_simulator.motors.motor_base import MotorConfig
from robot_simulator.physics.motor_controller import MotorError

from .conftest import ARM_URDF, DIFF_URDF, MANIP_URDF, MESH_URDF, diff_drive_config


def joint_pos(robot, name):
    js = robot.physics.joint_states()
    return float(js.position[robot.physics.index_of_movable[name]])


def joint_vel(robot, name):
    js = robot.physics.joint_states()
    return float(js.velocity[robot.physics.index_of_movable[name]])


def test_direct_mode_only(sim):
    info = p.getConnectionInfo(sim.engine.client)
    assert info["isConnected"] == 1 and info["connectionMethod"] == p.DIRECT


def test_import_differential_drive_robot(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=DIFF_URDF))
    assert r.name == "diff_bot"
    assert set(r.physics.movable_joints) == {"left_wheel_joint", "right_wheel_joint"}
    sim.run_for(1.0)
    z = r.physics.link_transform("base_link")[2, 3]
    assert z == pytest.approx(0.08, abs=0.005)          # resting on its wheels
    assert r.motors.motors == {}                          # no automatic motors


def test_import_six_dof_arm_and_fk_agrees_with_physics(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=ARM_URDF, fixed_base=True,
                                    initial_joint_positions={"joint2": 0.4, "joint3": -0.7, "joint5": 0.3}))
    assert len(r.physics.movable_joints) == 6
    js = r.physics.joint_states()
    q = dict(zip(js.names, js.position))
    fk = r.model.forward_kinematics(q, r.physics.link_transform("base_link"))
    for link in r.model.links:
        np.testing.assert_allclose(fk[link], r.physics.link_transform(link), atol=1e-6)


def test_import_robot_with_multiple_visual_meshes(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=MESH_URDF, fixed_base=True))
    assert len(r.model.links["base_link"].visuals) == 3
    assert r.loaded.warnings == []


def test_revolute_joint_position_control(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=ARM_URDF, fixed_base=True))
    m = r.assign_motor(MotorConfig(joint="joint1", type="rotational", max_effort=80, max_velocity=2.0))
    m.set_position(1.2)
    sim.run_for(2.0)
    assert joint_pos(r, "joint1") == pytest.approx(1.2, abs=1e-3)
    # the child link actually rotated about Z
    x_axis = r.physics.link_transform("shoulder_link")[:3, 0]
    assert np.arctan2(x_axis[1], x_axis[0]) == pytest.approx(1.2, abs=1e-3)


def test_prismatic_joint_linear_motor(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=MANIP_URDF, fixed_base=True))
    m = r.assign_motor(MotorConfig(joint="lift", type="linear", max_effort=100, max_velocity=0.3))
    z0 = r.physics.link_transform("carriage_link")[2, 3]
    m.set_position(0.2)
    sim.run_for(2.0)
    assert joint_pos(r, "lift") == pytest.approx(0.2, abs=1e-3)
    assert r.physics.link_transform("carriage_link")[2, 3] - z0 == pytest.approx(0.2, abs=1e-3)


def test_joint_limits_are_enforced(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=MANIP_URDF, fixed_base=True))
    m = r.assign_motor(MotorConfig(joint="lift", type="linear", max_effort=100, max_velocity=1.0))
    m.set_position(5.0)                       # clamped to the URDF upper limit
    assert m.config.target_position == pytest.approx(0.35)
    m.set_effort(100.0)                       # push hard against the limit
    sim.run_for(2.0)
    assert joint_pos(r, "lift") <= 0.35 + 0.01
    e = r.assign_motor(MotorConfig(joint="elbow", type="rotational", max_effort=10, max_velocity=5.0))
    e.set_effort(-10.0)
    sim.run_for(2.0)
    assert joint_pos(r, "elbow") >= -1.8 - 0.02


def test_motor_velocity_control(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=ARM_URDF, fixed_base=True))
    m = r.assign_motor(MotorConfig(joint="joint1", type="rotational", mode="velocity", max_effort=80,
                                   max_velocity=2.0))
    m.set_velocity(0.8)
    sim.run_for(1.0)
    assert joint_vel(r, "joint1") == pytest.approx(0.8, abs=1e-3)
    m.set_velocity(10.0)                      # saturated by the motor's maximum velocity
    sim.run_for(1.0)
    assert joint_vel(r, "joint1") == pytest.approx(2.0, abs=1e-3)


def test_motor_effort_limits(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=ARM_URDF, fixed_base=True,
                                    initial_joint_positions={"joint2": 1.2}))
    # The extended arm needs much more than 2 N m. A weak motor cannot hold it.
    weak = r.assign_motor(MotorConfig(joint="joint2", type="rotational", max_effort=2.0, max_velocity=2.0,
                                      target_position=1.2))
    weak.set_position(1.2)
    sim.run_for(1.5)
    assert joint_pos(r, "joint2") > 1.25
    # max_effort limits the effort commands.
    weak.set_effort(50.0)
    sim.run_for(0.1)
    assert weak.applied_effort == pytest.approx(2.0)
    # A strong motor holds the same pose.
    r2 = sim.spawn_robot(RobotConfig(urdf_path=ARM_URDF, fixed_base=True, position=(2, 0, 0),
                                     initial_joint_positions={"joint2": 1.2}))
    strong = r2.assign_motor(MotorConfig(joint="joint2", type="rotational", max_effort=80.0, max_velocity=2.0))
    strong.set_position(1.2)
    sim.run_for(1.5)
    assert joint_pos(r2, "joint2") == pytest.approx(1.2, abs=0.01)


def test_motor_rules(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=MANIP_URDF, fixed_base=True))
    with pytest.raises(MotorError, match="fixed"):
        r.assign_motor(MotorConfig(joint="gripper_mount", type="rotational"))
    with pytest.raises(MotorError, match="linear motor cannot drive revolute"):
        r.assign_motor(MotorConfig(joint="elbow", type="linear"))
    with pytest.raises(MotorError, match="rotational motor cannot drive prismatic"):
        r.assign_motor(MotorConfig(joint="lift", type="rotational"))
    with pytest.raises(MotorError, match="mimics"):
        r.assign_motor(MotorConfig(joint="right_finger_joint", type="linear"))


def test_mimic_joint_follows_leader(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=MANIP_URDF, fixed_base=True))
    m = r.assign_motor(MotorConfig(joint="left_finger_joint", type="linear", max_effort=20, max_velocity=0.1))
    m.set_position(0.03)
    sim.run_for(1.5)
    assert joint_pos(r, "left_finger_joint") == pytest.approx(0.03, abs=1e-3)
    assert joint_pos(r, "right_finger_joint") == pytest.approx(0.03, abs=2e-3)


def test_differential_drive_moves_forward_and_turns(sim):
    r = sim.spawn_robot(diff_drive_config())
    sim.cmd_vel_timeout = 0.0
    r.drive.command(0.5, 0.0, sim.sim_time)
    sim.run_for(3.0)
    x = r.physics.link_transform("base_link")[0, 3]
    assert 1.2 < x < 1.6                      # ~0.5 m/s after a short acceleration phase
    r.drive.command(0.0, 1.0, sim.sim_time)
    sim.run_for(1.0)
    _, w = r.base_twist()
    assert w[2] == pytest.approx(1.0, abs=0.1)


def test_cmd_vel_timeout_stops_robot(sim):
    r = sim.spawn_robot(diff_drive_config())
    sim.cmd_vel_timeout = 0.3
    r.drive.command(0.5, 0.0, sim.sim_time)
    sim.run_for(2.0)
    v, _ = r.base_twist()
    assert abs(v[0]) < 0.05


def test_environment_objects_static_dynamic_and_collision_toggle(sim):
    box = sim.add_object(EnvObjectConfig(name="box", shape="box", size=(0.2, 0.2, 0.2), position=(0, 0, 1.0),
                                         mass=1.0))
    ghost = sim.add_object(EnvObjectConfig(name="ghost", shape="sphere", radius=0.1, position=(1, 0, 1.0),
                                           mass=1.0, collision_enabled=False))
    wall = sim.add_object(EnvObjectConfig(name="wall", shape="box", size=(1, 1, 1), position=(3, 0, 0.5)))
    sim.run_for(1.5)
    assert sim.engine.body_pose(box.body)[0][2] == pytest.approx(0.1, abs=0.01)   # rests on the ground
    assert sim.engine.body_pose(ghost.body)[0][2] < -1.0                           # falls through
    assert sim.engine.body_pose(wall.body)[0][2] == pytest.approx(0.5)            # static
    sim.reset()
    assert sim.engine.body_pose(box.body)[0][2] == pytest.approx(1.0)


def test_pause_and_reset(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=ARM_URDF, fixed_base=True))
    m = r.assign_motor(MotorConfig(joint="joint1", type="rotational", mode="velocity", max_effort=80,
                                   max_velocity=2))
    m.set_velocity(1.0)
    sim.play()
    for _ in range(30):               # 0.5 s of wall time at 60 FPS
        sim.update(1 / 60)
    t = sim.sim_time
    assert t == pytest.approx(0.5, abs=sim.timestep)
    sim.pause()
    q = joint_pos(r, "joint1")
    for _ in range(30):
        sim.update(1 / 60)
    assert sim.sim_time == t and joint_pos(r, "joint1") == q
    sim.reset()
    assert sim.sim_time == 0.0 and joint_pos(r, "joint1") == 0.0
    assert m.config.target_velocity == 0.0


def test_duplicate_sensor_frames_across_robots_are_rejected(sim, tmp_path):
    from robot_simulator.sensors import ImuConfig
    other = tmp_path / "other.urdf"
    other.write_text('<robot name="other"><link name="other_base"><inertial><mass value="1"/>'
                     '<inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link></robot>')
    a = sim.spawn_robot(RobotConfig(urdf_path=ARM_URDF, fixed_base=True))
    b = sim.spawn_robot(RobotConfig(urdf_path=str(other), fixed_base=True, position=(2, 0, 0)))
    assert a.tf_prefix == "" and b.tf_prefix == ""          # no shared link names -> no prefix needed
    a.add_sensor(ImuConfig(name="imu", link="base_link"))
    with pytest.raises(ValueError, match="already used by robot"):
        b.add_sensor(ImuConfig(name="imu", link="other_base"))
    assert b.unique_sensor_name("imu") == "imu_1"
