"""Load a project JSON document into a SimulationManager.

If the URDF or the meshes of a robot are missing, the loader logs a clear
error and does not load that robot. The loader then continues with the
remaining items of the project.
"""
from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from ..control.diff_drive import DiffDriveConfig
from ..core.configuration import PhysicsConfig, Ros2Config, from_dict
from ..core.robot_instance import OdometryConfig, RobotConfig, RosRobotConfig
from ..environment.environment_objects import EnvObjectConfig
from ..motors.motor_base import MotorConfig
from ..sensors import sensor_config_from_dict
from .project_saver import FORMAT, VERSION

log = logging.getLogger(__name__)


class ProjectError(Exception):
    pass


@dataclass
class LoadReport:
    name: str = ""
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    view: Optional[dict] = None


def robot_config_from_dict(d: dict, project_dir: Path) -> RobotConfig:
    d = dict(d)
    motors = [from_dict(MotorConfig, m) for m in d.pop("motors", [])]
    sensors = [sensor_config_from_dict(s) for s in d.pop("sensors", [])]
    drive = from_dict(DiffDriveConfig, d.pop("diff_drive", None))
    odom = from_dict(OdometryConfig, d.pop("odometry", None))
    ros = from_dict(RosRobotConfig, d.pop("ros", None))
    cfg = from_dict(RobotConfig, d)
    cfg.motors, cfg.sensors, cfg.diff_drive, cfg.odometry, cfg.ros = motors, sensors, drive, odom, ros
    p = Path(cfg.urdf_path).expanduser()
    if not p.is_absolute():
        p = project_dir / p
    cfg.urdf_path = str(p)
    return cfg


def read_document(project_file: str | Path) -> dict:
    project_file = Path(project_file)
    if not project_file.is_file():
        raise ProjectError(f"Project file not found: {project_file}")
    try:
        doc = json.loads(project_file.read_text())
    except json.JSONDecodeError as exc:
        raise ProjectError(f"Project file {project_file} is not valid JSON: {exc}") from exc
    if doc.get("format") != FORMAT:
        raise ProjectError(f"{project_file} is not a robot simulator project")
    if int(doc.get("version", 0)) > VERSION:
        raise ProjectError(f"Project version {doc.get('version')} is newer than supported ({VERSION})")
    return doc


def load_project(sim, project_file: str | Path) -> LoadReport:
    project_file = Path(project_file).resolve()
    doc = read_document(project_file)
    project_dir = project_file.parent
    report = LoadReport(name=doc.get("name", project_file.parent.name), view=doc.get("view"))

    sim.stop()
    sim.clear()
    sim.config.physics = from_dict(PhysicsConfig, doc.get("physics"))
    sim.engine.config = sim.config.physics
    sim.config.ros2 = from_dict(Ros2Config, doc.get("ros2"))
    sim.config.package_paths = list(doc.get("package_paths", []))
    sim.loader.package_paths = tuple(sim.config.package_paths)
    sim.loader.parser.package_paths = tuple(sim.config.package_paths)
    sim.set_physics_config()
    try:
        sim.set_speed(float(doc.get("simulation", {}).get("speed", 1.0)))
    except ValueError:
        pass

    for od in doc.get("world", {}).get("objects", []):
        try:
            sim.add_object(from_dict(EnvObjectConfig, od))
        except Exception as exc:
            report.errors.append(f"environment object '{od.get('name', '?')}': {exc}")

    for rd in doc.get("robots", []):
        name = rd.get("name", "?")
        try:
            cfg = robot_config_from_dict(rd, project_dir)
        except Exception as exc:
            report.errors.append(f"robot '{name}': invalid configuration: {exc}")
            continue
        if not Path(cfg.urdf_path).is_file():
            report.errors.append(f"robot '{name}': URDF not found at '{cfg.urdf_path}'. "
                                 f"Move the file back or edit 'urdf_path' in {project_file.name}.")
            continue
        try:
            r = sim.spawn_robot(cfg)
            missing = r.model.description.unresolved_meshes
            if missing:
                report.warnings.append(f"robot '{name}': {len(missing)} mesh(es) could not be located: "
                                       + ", ".join(sorted(set(missing))[:5]))
        except Exception as exc:
            report.errors.append(f"robot '{name}': {exc}")

    sim.reset()
    for e in report.errors:
        log.error("Project: %s", e)
    for w in report.warnings:
        log.warning("Project: %s", w)
    log.info("Loaded project '%s' (%d robots, %d objects)", report.name, len(sim.robots), len(sim.world.objects))
    return report
