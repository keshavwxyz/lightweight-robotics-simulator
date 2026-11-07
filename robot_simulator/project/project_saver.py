"""Write the simulation to a project JSON document."""
from __future__ import annotations

import json
import os
from pathlib import Path

from ..core.configuration import to_dict

FORMAT = "robot_simulator_project"
VERSION = 1


def _portable_path(path: str, project_dir: Path) -> str:
    """Return a relative path for files in the project directory. Return an absolute path for other files."""
    p = Path(path).resolve()
    try:
        return str(p.relative_to(project_dir.resolve()))
    except ValueError:
        return str(p)


def project_document(sim, name: str, project_dir: Path, view_options=None) -> dict:
    robots = []
    for r in sim.robots.values():
        d = to_dict(r.config)
        d["urdf_path"] = _portable_path(r.config.urdf_path, project_dir)
        robots.append(d)
    doc = {
        "format": FORMAT,
        "version": VERSION,
        "name": name,
        "physics": to_dict(sim.config.physics),
        "ros2": to_dict(sim.config.ros2),
        "simulation": {"speed": sim.speed},
        "world": {"objects": [to_dict(o.config) for o in sim.world.objects.values()]},
        "robots": robots,
        "package_paths": list(sim.config.package_paths),
    }
    if view_options is not None:
        doc["view"] = to_dict(view_options)
    return doc


def save_project(sim, project_file: str | Path, name: str, view_options=None) -> Path:
    project_file = Path(project_file)
    project_dir = project_file.parent
    for sub in ("robots", "environments", "assets", "logs"):
        (project_dir / sub).mkdir(parents=True, exist_ok=True)
    doc = project_document(sim, name, project_dir, view_options)
    tmp = project_file.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(doc, indent=2))
    os.replace(tmp, project_file)   # atomic replace: a failure does not leave a partial file
    return project_file
