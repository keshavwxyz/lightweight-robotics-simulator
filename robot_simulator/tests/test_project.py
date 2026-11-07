"""Project save / load round trip."""
import json
import math
import shutil

import pytest

from robot_simulator.core.robot_instance import RobotConfig
from robot_simulator.environment.environment_objects import EnvObjectConfig
from robot_simulator.project.project_loader import ProjectError
from robot_simulator.project.project_manager import ProjectManager
from robot_simulator.sensors import CameraConfig, ImuConfig, Lidar2DConfig

from .conftest import DIFF_URDF, EXAMPLES, diff_drive_config


def test_save_and_load_round_trip(sim, tmp_path):
    cfg = diff_drive_config()
    cfg.sensors = [ImuConfig(name="imu", link="base_link", rate_hz=200),
                   Lidar2DConfig(name="lidar", link="lidar_mount", angular_resolution=math.radians(0.5)),
                   CameraConfig(name="cam", link="camera_mount", width=320, height=240)]
    r = sim.spawn_robot(cfg)
    r.config.initial_joint_positions["left_wheel_joint"] = 0.25
    sim.add_object(EnvObjectConfig(name="crate", shape="box", size=(0.3, 0.4, 0.5), position=(1, 2, 0.25),
                                   mass=2.0, collision_enabled=False))
    sim.config.physics.frequency_hz = 500.0
    sim.set_speed(2.0)

    pm = ProjectManager(sim)
    path = pm.save(tmp_path / "proj")
    assert path.name == "project.json"
    for sub in ("robots", "environments", "assets", "logs"):
        assert (tmp_path / "proj" / sub).is_dir()

    pm.new()
    assert not sim.robots
    report = pm.open(tmp_path / "proj")
    assert report.errors == []
    r2 = sim.robots["diff_bot"]
    assert set(r2.sensors) == {"imu", "lidar", "cam"}
    assert r2.sensors["imu"].config.rate_hz == 200
    assert r2.sensors["cam"].config.width == 320
    assert {m for m in r2.motors.motors} == {"left_wheel_joint", "right_wheel_joint"}
    assert r2.motors.get("left_wheel_joint").config.mode == "velocity"
    assert r2.config.diff_drive.enabled and r2.config.diff_drive.wheel_radius == 0.08
    assert r2.config.odometry.enabled and r2.config.odometry.base_frame == "base_footprint"
    assert r2.config.initial_joint_positions == {"left_wheel_joint": 0.25}
    crate = sim.world.objects["crate"].config
    assert crate.size == (0.3, 0.4, 0.5) and crate.mass == 2.0 and not crate.collision_enabled
    assert sim.config.physics.frequency_hz == 500.0 and sim.timestep == pytest.approx(1 / 500)
    assert sim.speed == 2.0


def test_project_inside_dir_uses_relative_urdf_paths(sim, tmp_path):
    proj = tmp_path / "p"
    (proj / "robots").mkdir(parents=True)
    shutil.copy(DIFF_URDF, proj / "robots" / "bot.urdf")
    sim.spawn_robot(RobotConfig(urdf_path=str(proj / "robots" / "bot.urdf")))
    ProjectManager(sim).save(proj)
    doc = json.loads((proj / "project.json").read_text())
    assert doc["robots"][0]["urdf_path"] == "robots/bot.urdf"
    # the folder can be moved
    moved = tmp_path / "moved"
    shutil.move(str(proj), moved)
    report = ProjectManager(sim).open(moved)
    assert report.errors == [] and "diff_bot" in sim.robots


def test_missing_urdf_is_a_clear_error(sim, tmp_path):
    proj = tmp_path / "p"
    sim.spawn_robot(RobotConfig(urdf_path=DIFF_URDF))
    ProjectManager(sim).save(proj)
    doc = json.loads((proj / "project.json").read_text())
    doc["robots"][0]["urdf_path"] = "/nonexistent/robot.urdf"
    (proj / "project.json").write_text(json.dumps(doc))
    report = ProjectManager(sim).open(proj)
    assert len(report.errors) == 1 and "URDF not found" in report.errors[0]


def test_invalid_project_file(sim, tmp_path):
    (tmp_path / "project.json").write_text("{not json")
    with pytest.raises(ProjectError, match="not valid JSON"):
        ProjectManager(sim).open(tmp_path)


@pytest.mark.parametrize("example", ["differential_drive_robot", "robotic_arm", "simple_manipulator"])
def test_example_projects_load(sim, example):
    report = ProjectManager(sim).open(EXAMPLES / example / "project.json")
    assert report.errors == [] and len(sim.robots) == 1
    sim.run_for(0.5)
