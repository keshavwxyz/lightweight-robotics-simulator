"""Features for existing ROS 2 projects (refer to docs/porting_guide.md)."""
import math

import numpy as np
import pytest

from robot_simulator.core.robot_instance import OdometryConfig, RobotConfig, RosRobotConfig
from robot_simulator.environment.environment_objects import EnvObjectConfig
from robot_simulator.sensors import ImuConfig, Lidar2DConfig

from .conftest import DIFF_URDF, requires_display


def test_partial_fov_lidar(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=DIFF_URDF))
    fov = math.radians(190)
    lidar = r.add_sensor(Lidar2DConfig(name="front", link="lidar_mount", xyz=(0, 0, 0.03),
                                       horizontal_fov=fov, angular_resolution=fov / 380, rate_hz=10))
    sim.add_object(EnvObjectConfig(name="wall", shape="box", size=(0.2, 6, 1), position=(2.1, 0, 0.5)))
    sim.add_object(EnvObjectConfig(name="behind", shape="box", size=(0.2, 6, 1), position=(-2.0, 0, 0.5)))
    sim.run_for(0.3)
    scan = lidar.latest
    assert len(scan.ranges) == 381
    assert scan.angle_min == pytest.approx(-fov / 2) and scan.angle_max == pytest.approx(fov / 2)
    mid = scan.ranges[190]
    origin_x = lidar.world_transform()[0, 3]
    assert mid == pytest.approx(2.0 - origin_x, abs=2e-3)       # straight ahead
    # the wall behind is outside the 190 deg field of view
    assert np.isinf(scan.ranges[0]) or scan.ranges[0] > 1.5


def test_sensor_frame_override_uses_existing_frame(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=DIFF_URDF))
    # Without frame_id, the sensor frame is "lidar_mount_link". frame_id can be a URDF link name.
    s = r.add_sensor(Lidar2DConfig(name="lidar_mount", link="lidar_mount", frame_id="lidar_mount"))
    assert s.frame_id == "lidar_mount" and s.frame_override
    from robot_simulator.ros2_bridge.tf_manager import TFManager
    tf = TFManager()
    tf.set_robot(sim.describe(r))
    # Only the URDF edge base_link -> lidar_mount exists. The sensor adds no frame.
    assert tf.static[r.name]["lidar_mount"][0] == "base_link"
    assert "lidar_mount_link" not in tf.static[r.name]


def test_publish_tf_false_and_namespace_resolution(sim):
    from robot_simulator.ros2_bridge.publishers import resolve_topic, sensor_topic
    from robot_simulator.ros2_bridge.tf_manager import TFManager
    cfg = RobotConfig(urdf_path=DIFF_URDF, ros=RosRobotConfig(namespace="", publish_tf=False,
                                                              cmd_vel_topic="sim/cmd_vel"))
    r = sim.spawn_robot(cfg)
    r.add_sensor(ImuConfig(name="imu", link="base_link", topic="imu/data"))
    d = sim.describe(r)
    assert d.namespace == "" and resolve_topic(d, "joint_states") == "/joint_states"
    assert resolve_topic(d, d.cmd_vel_topic) == "/sim/cmd_vel"
    assert sensor_topic(d, "imu") == "/imu/data"
    tf = TFManager()
    tf.set_robot(d)
    assert tf.all_static() == [] and tf.dynamic(sim.snapshot(r)) == []
    r.config.ros.namespace = "fleet/robot_7"
    assert sim.describe(r).namespace == "/fleet/robot_7"
    r.config.ros.namespace = None
    assert sim.describe(r).namespace == "/diff_bot"


def test_world_origin_odometry(sim):
    r = sim.spawn_robot(RobotConfig(urdf_path=DIFF_URDF, position=(3, -2, 0), rpy=(0, 0, 1.0),
                                    odometry=OdometryConfig(enabled=True, origin="world")))
    sim.run_for(0.2)
    pose = r.odometry_pose()
    np.testing.assert_allclose(pose[:2, 3], [3, -2], atol=0.01)
    spawn = sim.spawn_robot(RobotConfig(urdf_path=DIFF_URDF, position=(3, 2, 0),
                                        odometry=OdometryConfig(enabled=True)))
    np.testing.assert_allclose(spawn.odometry_pose()[:2, 3], [0, 0], atol=0.01)


def test_textured_obj_is_loaded_with_uvs(tmp_path):
    from PIL import Image
    from robot_simulator.rendering.mesh_renderer import load_mesh_file
    img = np.zeros((8, 8, 3), np.uint8)
    img[:4, :4] = 255
    Image.fromarray(img).save(tmp_path / "tex.png")
    (tmp_path / "q.mtl").write_text("newmtl m\nKd 1 1 1\nmap_Kd tex.png\n")
    (tmp_path / "q.obj").write_text("mtllib q.mtl\nusemtl m\nv 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n"
                                    "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nf 1/1 2/2 3/3\nf 1/1 3/3 4/4\n")
    v, n, i, c, uv, tex = load_mesh_file(str(tmp_path / "q.obj"))
    assert len(v) == 6 and uv is not None and uv.shape == (6, 2)
    assert tex is not None and tex.shape == (8, 8, 4) and tex[0, 0, 0] == 255


@requires_display
def test_textured_mesh_renders_its_texture(sim, tmp_path):
    from PIL import Image
    from robot_simulator.rendering.offscreen_context import OffscreenContext
    from robot_simulator.rendering.renderer import Renderer
    from robot_simulator.sensors import CameraConfig
    Image.fromarray(np.full((4, 4, 3), (255, 0, 0), np.uint8)).save(tmp_path / "red.png")
    (tmp_path / "q.mtl").write_text("newmtl m\nKd 1 1 1\nmap_Kd red.png\n")
    # 2 m x 2 m quad facing -x at x = 0 (seen by a camera at negative x looking along +x)
    (tmp_path / "q.obj").write_text("mtllib q.mtl\nusemtl m\nv 0 1 -1\nv 0 -1 -1\nv 0 -1 1\nv 0 1 1\n"
                                    "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nf 1/1 2/2 3/3\nf 1/1 3/3 4/4\n")
    (tmp_path / "board.urdf").write_text(
        '<robot name="board"><link name="b"><visual><geometry><mesh filename="q.obj"/></geometry>'
        '</visual></link></robot>')
    ctx = OffscreenContext()
    try:
        ren = Renderer()
        sim.set_image_backend(ren.make_image_backend(sim))
        sim.spawn_robot(RobotConfig(urdf_path=str(tmp_path / "board.urdf"), fixed_base=True,
                                    position=(2, 0, 1)))
        cam_robot = sim.spawn_robot(RobotConfig(urdf_path=DIFF_URDF))
        cam = cam_robot.add_sensor(CameraConfig(name="cam", link="camera_mount", width=64, height=48))
        sim.run_for(0.2)
        px = cam.latest.data[24, 32].astype(int)
        assert px[0] > 100 and px[0] > 3 * px[1] and px[0] > 3 * px[2]   # lit texture red, not grey
        ren.release()
    finally:
        ctx.close()
