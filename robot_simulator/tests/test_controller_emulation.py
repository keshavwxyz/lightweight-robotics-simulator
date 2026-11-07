"""Controller models of examples/ros2_integration (no ROS graph necessary)."""
import math

import pytest

from robot_simulator.examples.ros2_integration.controller_logic import (
    AxisLimits, controller_sections, DiffDriveModel, flatten, TrajectoryFollower)

CONTROLLERS = {
    "controller_manager": {"ros__parameters": {"update_rate": 50}},
    "base": {"ros__parameters": {
        "left_wheel_names": ["l"], "right_wheel_names": ["r"],
        "wheel_separation": 0.8, "wheel_radius": 0.18, "cmd_vel_timeout": 0.5,
        "linear": {"x": {"max_velocity": 1.0, "min_velocity": -0.5, "max_acceleration": 0.5}},
        "angular.z.max_velocity": 0.8}},
    "arm": {"ros__parameters": {"joints": ["j1"], "gains": {"j1": {"p": 2.0}}}},
}


def test_sections_with_and_without_namespace():
    plain = controller_sections(CONTROLLERS)
    wrapped = controller_sections({"robot_1": CONTROLLERS})
    assert plain == wrapped
    assert plain["controller_manager"]["update_rate"] == 50


def test_nested_and_dotted_keys_give_the_same_limits():
    p = flatten(controller_sections(CONTROLLERS)["base"])
    lim = AxisLimits.from_params(p, "linear.x")
    assert (lim.max_velocity, lim.min_velocity, lim.max_acceleration) == (1.0, -0.5, 0.5)
    assert AxisLimits.from_params(p, "angular.z").max_velocity == 0.8


def test_acceleration_and_velocity_limits():
    lim = AxisLimits(max_velocity=1.0, max_acceleration=0.5, max_deceleration=0.5)
    v = 0.0
    for _ in range(50):
        v = lim.limit(2.0, v, 0.02)
    assert v == pytest.approx(0.5)
    for _ in range(200):
        v = lim.limit(2.0, v, 0.02)
    assert v == pytest.approx(1.0)


def test_wheel_velocities_and_command_timeout():
    d = DiffDriveModel.from_params(flatten(controller_sections(CONTROLLERS)["base"]))
    d.linear, d.angular = AxisLimits(), AxisLimits()
    assert d.update_command(0.36, 0.0, 0.0, 0.02) == pytest.approx((2.0, 2.0))
    wl, wr = d.update_command(0.0, 0.5, 0.0, 0.02)
    assert (wl, wr) == pytest.approx((-0.5 * 0.4 / 0.18, 0.5 * 0.4 / 0.18))
    assert d.update_command(0.36, 0.0, 0.6, 0.02) == (0.0, 0.0)
    assert d.update_command(0.36, 0.0, None, 0.02) == (0.0, 0.0)


def test_encoder_odometry():
    d = DiffDriveModel(wheel_separation=0.8, wheel_radius=0.18)
    d.update_odometry(0.0, 0.0, 0.0)
    d.update_odometry(10.0, 10.0, 1.0)
    assert (d.x, d.y, d.yaw) == pytest.approx((1.8, 0.0, 0.0))
    turn = math.pi / 2 * 0.4 / 0.18
    d.update_odometry(10.0 - turn, 10.0 + turn, 1.0)
    assert d.yaw == pytest.approx(math.pi / 2) and d.x == pytest.approx(1.8)


def test_trajectory_follower():
    f = TrajectoryFollower(p_gain=2.0, max_velocity=0.2)
    assert f.command(0.0, 0.05) == pytest.approx(0.0)
    f.set_trajectory([(1.0, 0.10, 0.05), (2.0, 0.15, 0.0)], start_time=10.0, current_pos=0.05)
    assert f.sample(11.0) == pytest.approx((0.10, 0.05))
    assert f.command(11.0, 0.09) == pytest.approx(0.05 + 2.0 * 0.01)
    assert f.command(20.0, 0.15) == pytest.approx(0.0)
    assert f.command(20.0, 0.00) == pytest.approx(0.2)
