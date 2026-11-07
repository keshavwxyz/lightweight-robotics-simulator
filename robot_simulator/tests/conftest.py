import os
from pathlib import Path

# The ROS tests must not join the ROS graph of the developer. Other projects can use the
# ROS_DOMAIN_ID of the shell. Set a separate domain and local discovery before rclpy loads.
os.environ["ROS_DOMAIN_ID"] = os.environ.get("ROBOT_SIM_TEST_DOMAIN_ID", "77")
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ.pop("ROS_STATIC_PEERS", None)

import pytest

from robot_simulator.control.diff_drive import DiffDriveConfig
from robot_simulator.core.robot_instance import OdometryConfig, RobotConfig
from robot_simulator.core.simulation_manager import SimulationManager
from robot_simulator.motors.motor_base import MotorConfig

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"
DIFF_URDF = str(EXAMPLES / "differential_drive_robot" / "diff_drive_robot.urdf")
ARM_URDF = str(EXAMPLES / "robotic_arm" / "six_dof_arm.urdf")
MANIP_URDF = str(EXAMPLES / "simple_manipulator" / "simple_manipulator.urdf")
MESH_URDF = str(EXAMPLES / "mesh_robot_description" / "urdf" / "mesh_robot.urdf")


def ros_available() -> bool:
    try:
        import importlib
        importlib.import_module("rclpy")
        return True
    except Exception:
        return False


def display_available() -> bool:
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


requires_ros = pytest.mark.skipif(not ros_available(), reason="rclpy not available (source ROS 2)")
requires_display = pytest.mark.skipif(not display_available(), reason="no display for an OpenGL context")


@pytest.fixture
def sim():
    s = SimulationManager()
    yield s
    s.shutdown()


def diff_drive_config(**kw) -> RobotConfig:
    cfg = RobotConfig(
        urdf_path=DIFF_URDF,
        link_friction={"front_caster": 0.0, "rear_caster": 0.0},
        motors=[MotorConfig(joint=j, type="rotational", mode="velocity", max_effort=5.0, max_velocity=20.0)
                for j in ("left_wheel_joint", "right_wheel_joint")],
        diff_drive=DiffDriveConfig(enabled=True, left_joints=["left_wheel_joint"],
                                   right_joints=["right_wheel_joint"], wheel_radius=0.08, wheel_separation=0.36),
        odometry=OdometryConfig(enabled=True, base_frame="base_footprint"),
    )
    for k, v in kw.items():
        setattr(cfg, k, v)
    return cfg
