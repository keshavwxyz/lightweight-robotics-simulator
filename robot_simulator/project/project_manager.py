"""Project lifecycle: new, open and save. A project is a directory:

    <project>/
        project.json
        robots/         (optional place for project-local URDFs)
        environments/
        assets/
        logs/

The project refers to URDFs and meshes. It does not copy them. Paths in the
project directory are relative. Thus, you can move the directory.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

from ..environment.ground_plane import default_ground
from .project_loader import LoadReport, load_project
from .project_saver import save_project

log = logging.getLogger(__name__)

PROJECT_FILE = "project.json"


class ProjectManager:
    def __init__(self, sim):
        self.sim = sim
        self.name = "untitled"
        self.path: Optional[Path] = None     # project.json
        self.dirty = False

    @staticmethod
    def resolve_project_file(path: str | Path) -> Path:
        p = Path(path).expanduser()
        if p.is_dir() or p.suffix.lower() != ".json":
            p = p / PROJECT_FILE
        return p.resolve()

    def new(self, name: str = "untitled") -> None:
        self.sim.stop()
        self.sim.clear()
        self.sim.add_object(default_ground())
        self.sim.reset()
        self.name, self.path, self.dirty = name, None, False
        log.info("New project '%s'", name)

    def open(self, path: str | Path) -> LoadReport:
        pf = self.resolve_project_file(path)
        report = load_project(self.sim, pf)
        self.name, self.path, self.dirty = report.name, pf, False
        return report

    def save(self, path: Optional[str | Path] = None, view_options=None) -> Path:
        if path is not None:
            self.path = self.resolve_project_file(path)
            if self.name == "untitled":
                self.name = self.path.parent.name
        if self.path is None:
            raise ValueError("No project path set; use 'Save As'")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        save_project(self.sim, self.path, self.name, view_options)
        self.dirty = False
        log.info("Saved project '%s' to %s", self.name, self.path)
        return self.path
