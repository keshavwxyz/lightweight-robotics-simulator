"""Perfect IMU, 2D LiDAR and RGB camera."""
import math

import numpy as np
import pytest

from robot_simulator.core.robot_instance import RobotConfig
from robot_simulator.environment.environment_objects import EnvObjectConfig
from robot_simulator.motors.motor_base import MotorConfig
from robot_simulator.sensors import CameraConfig, ImuConfig, Lidar2DConfig
from robot_simulator.utils.transforms import quat_to_rpy

from .conftest import ARM_URDF, DIFF_URDF, requires_display

G = 9.81


def test_imu_orientation_matches_link(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=DIFF_URDF, rpy=(0, 0, 0.7)))
    imu = r.add_sensor(ImuConfig(name="imu", link="base_link", rate_hz=100))
    imu_rot = r.add_sensor(ImuConfig(name="imu_rot", link="base_link", rpy=(0, 0, math.pi / 2), rate_hz=100))
    sim.run_for(0.5)
    roll, pitch, yaw = quat_to_rpy(imu.latest.orientation)
    assert yaw == pytest.approx(0.7, abs=1e-3) and abs(roll) < 0.02 and abs(pitch) < 0.02
    assert quat_to_rpy(imu_rot.latest.orientation)[2] == pytest.approx(0.7 + math.pi / 2, abs=1e-3)


def test_imu_angular_velocity(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=ARM_URDF, fixed_base=True))
    m = r.assign_motor(MotorConfig(joint="joint1", type="rotational", mode="velocity", max_effort=80,
                                   max_velocity=2))
    imu = r.add_sensor(ImuConfig(name="imu", link="shoulder_link", rate_hz=200))
    tilted = r.add_sensor(ImuConfig(name="tilted", link="shoulder_link", rpy=(0, math.pi / 2, 0), rate_hz=200))
    m.set_velocity(1.5)
    sim.run_for(1.0)
    np.testing.assert_allclose(imu.latest.angular_velocity, [0, 0, 1.5], atol=1e-3)
    # a frame with 90 deg pitch sees a rotation about world Z on its -X axis
    np.testing.assert_allclose(tilted.latest.angular_velocity, [-1.5, 0, 0], atol=1e-3)


def test_imu_acceleration_and_gravity(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=ARM_URDF, fixed_base=True))
    level = r.add_sensor(ImuConfig(name="level", link="base_link", rate_hz=100))
    rolled = r.add_sensor(ImuConfig(name="rolled", link="base_link", rpy=(math.pi / 2, 0, 0), rate_hz=100))
    sim.run_for(0.5)
    np.testing.assert_allclose(level.latest.linear_acceleration, [0, 0, G], atol=1e-3)
    np.testing.assert_allclose(rolled.latest.linear_acceleration, [0, G, 0], atol=1e-3)

    # Centripetal acceleration of an IMU 0.2 m from the rotation axis: w^2 r = 1 * 0.2
    m = r.assign_motor(MotorConfig(joint="joint1", type="rotational", mode="velocity", max_effort=80,
                                   max_velocity=2))
    offset = r.add_sensor(ImuConfig(name="offset", link="shoulder_link", xyz=(0.2, 0, 0), rate_hz=200))
    m.set_velocity(1.0)
    sim.run_for(1.5)
    np.testing.assert_allclose(offset.latest.linear_acceleration, [-0.2, 0, G], atol=5e-3)


def test_imu_reads_zero_in_free_fall(sim):
    import pybullet as p
    r = sim.spawn_robot(RobotConfig(urdf_path=DIFF_URDF, position=(0, 0, 50)))
    # PyBullet applies a small default velocity damping. Remove it to test only the sensor.
    for i in [-1] + r.physics.child_link_indices:
        p.changeDynamics(r.physics.body, i, linearDamping=0.0, angularDamping=0.0,
                         physicsClientId=sim.engine.client)
    imu = r.add_sensor(ImuConfig(name="imu", link="base_link", rate_hz=100))
    sim.run_for(0.3)
    assert np.linalg.norm(imu.latest.linear_acceleration) < 1e-6


def test_lidar_ranges_against_known_obstacles(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=DIFF_URDF))
    lidar = r.add_sensor(Lidar2DConfig(name="lidar", link="lidar_mount", xyz=(0, 0, 0.03), rate_hz=10,
                                       angular_resolution=math.radians(0.5), range_max=8.0))
    sim.add_object(EnvObjectConfig(name="wall_x", shape="box", size=(0.2, 6, 1), position=(3.1, 0, 0.5)))
    sim.add_object(EnvObjectConfig(name="wall_y", shape="box", size=(6, 0.2, 1), position=(0, 2.1, 0.5)))
    sim.run_for(0.5)
    scan = lidar.latest
    assert len(scan.ranges) == 720
    origin = lidar.world_transform()[:3, 3]

    def range_at(angle):
        k = int(round((angle - scan.angle_min) / scan.angle_increment))
        return float(scan.ranges[k])
    assert range_at(0.0) == pytest.approx(3.0 - origin[0], abs=2e-3)               # +X wall
    assert range_at(math.pi / 2) == pytest.approx(2.0 - origin[1], abs=2e-3)       # +Y wall
    assert range_at(math.atan2(2.0 - origin[1], 3.0 - origin[0]) - 0.3) == pytest.approx(
        (3.0 - origin[0]) / math.cos(math.atan2(2.0 - origin[1], 3.0 - origin[0]) - 0.3), abs=5e-3)
    assert math.isinf(range_at(-math.pi / 2))                                       # nothing at -Y
    assert lidar.update_count == pytest.approx(0.5 * 10, abs=1)                     # runs at 10 Hz


def test_sensor_rates_are_respected(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=DIFF_URDF))
    imu = r.add_sensor(ImuConfig(name="imu", link="base_link", rate_hz=50))
    lidar = r.add_sensor(Lidar2DConfig(name="lidar", link="lidar_mount", rate_hz=5))
    sim.run_for(2.0)
    assert imu.update_count == pytest.approx(100, abs=1)
    assert lidar.update_count == pytest.approx(10, abs=1)


@requires_display
def test_camera_image_dimensions_and_content(sim):
    from robot_simulator.rendering.offscreen_context import OffscreenContext
    from robot_simulator.rendering.renderer import Renderer
    ctx = OffscreenContext()
    try:
        renderer = Renderer()
        sim.set_image_backend(renderer.make_image_backend(sim))
        r = sim.spawn_robot(RobotConfig(urdf_path=DIFF_URDF))
        cam = r.add_sensor(CameraConfig(name="cam", link="camera_mount", width=320, height=200, rate_hz=10,
                                        horizontal_fov=math.radians(90)))
        sim.add_object(EnvObjectConfig(name="red", shape="box", size=(0.2, 3, 3), position=(2, 0, 1.5),
                                       color=(1.0, 0.0, 0.0, 1.0)))
        sim.run_for(0.3)
        img = cam.latest
        assert img.data.shape == (200, 320, 3) and img.data.dtype == np.uint8
        assert (img.width, img.height) == (320, 200)
        assert img.frame_id == "cam_optical_frame"
        fx = img.K[0]
        assert fx == pytest.approx(160.0)       # (w/2) / tan(hfov/2) with hfov = 90 deg
        center = img.data[100, 160].astype(int)
        assert center[0] > 2 * center[1] and center[0] > 2 * center[2]   # looking at the red wall
        renderer.release()
    finally:
        ctx.close()
