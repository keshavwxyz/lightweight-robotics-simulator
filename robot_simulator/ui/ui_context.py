"""Shared state and user actions for the UI panels.

The panels read the state from this class and call its actions. The actions
use the simulation manager (same thread), the project manager and the ROS 2
bridge.
"""
from __future__ import annotations

import logging
import math
from typing import Optional

import numpy as np

from ..core.robot_instance import RobotConfig
from ..environment.environment_objects import preset
from ..rendering.camera_controller import OrbitCamera
from ..rendering.scene_builder import Selection, ViewOptions
from .file_dialog import FileDialog

log = logging.getLogger(__name__)


class UIContext:
    def __init__(self, app):
        self.app = app
        self.sim = app.sim
        self.bridge = app.bridge
        self.project = app.project
        self.selection = Selection()
        self.view = ViewOptions()
        self.camera = OrbitCamera()
        self.file_dialog = FileDialog()
        self.camera_windows: set[tuple[str, str]] = set()   # (robot, sensor) image windows
        self.show_topics = False
        self.show_about = False
        self.show_controls = False
        self.import_fixed_base = False
        self.status_message = ""
        self.error_popup: Optional[str] = None
        self.last_dir: Optional[str] = None

    def select(self, kind: str = "", robot: str = "", name: str = "") -> None:
        self.selection = Selection(kind, robot, name)

    def selected_robot(self):
        return self.sim.robots.get(self.selection.robot) if self.selection.robot else None

    def focus_selection(self) -> None:
        sel = self.selection
        r = self.selected_robot()
        if r is not None:
            wt = r.physics.update_link_transforms()
            if sel.kind == "link" and sel.name in wt:
                self.camera.focus(wt[sel.name][:3, 3], 0.2)
            elif sel.kind == "sensor" and sel.name in r.sensors:
                self.camera.focus(r.sensors[sel.name].world_transform()[:3, 3], 0.2)
            elif sel.kind == "joint" and sel.name in r.model.joints:
                self.camera.focus(wt[r.model.joints[sel.name].child][:3, 3], 0.2)
            else:
                pts = np.array([T[:3, 3] for T in wt.values()])
                center = pts.mean(axis=0)
                radius = float(np.max(np.linalg.norm(pts - center, axis=1))) + 0.25
                self.camera.focus(center, radius)
        elif sel.kind == "object" and sel.name in self.sim.world.objects:
            obj = self.sim.world.objects[sel.name]
            pos, _ = self.sim.engine.body_pose(obj.body)
            c = obj.config
            size = max(c.size) if c.shape == "box" else max(c.radius * 2, c.length)
            self.camera.focus(pos, max(size, 0.3))

    def frame_scene(self) -> None:
        """Show all objects and robots. The camera looks down at an angle."""
        pts = []
        for obj in self.sim.world.objects.values():
            if obj.config.shape != "plane":
                pts.append(self.sim.engine.body_pose(obj.body)[0])
        for r in self.sim.robots.values():
            pts.append(r.physics.link_transform(r.model.root_link)[:3, 3])
        if not pts:
            return
        pts = np.asarray(pts, dtype=float)
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        center = (lo + hi) / 2.0
        center[2] = 0.0
        extent = float(np.max(hi[:2] - lo[:2]))
        self.camera.target = center
        self.camera.distance = max(extent * 0.95, 3.0)
        self.camera.yaw = math.radians(-90.0)        # from the -Y side, looking along +Y
        self.camera.pitch = math.radians(55.0)

    def report_error(self, message: str) -> None:
        log.error("%s", message)
        self.error_popup = message

    def import_urdf(self, path: str) -> None:
        try:
            r = self.sim.spawn_robot(RobotConfig(urdf_path=path, fixed_base=self.import_fixed_base))
        except Exception as exc:
            self.report_error(f"URDF import failed:\n{exc}")
            return
        self.project.dirty = True
        self.select("robot", r.name)
        self.focus_selection()
        if r.loaded.warnings:
            self.status_message = f"Imported '{r.name}' with {len(r.loaded.warnings)} warning(s) - see console"
        else:
            self.status_message = f"Imported '{r.name}'"

    def remove_robot(self, name: str) -> None:
        self.sim.remove_robot(name)
        self.camera_windows = {k for k in self.camera_windows if k[0] != name}
        self.select()
        self.project.dirty = True

    def add_object(self, kind: str) -> None:
        cfg = preset(kind, kind)
        if kind != "ground_plane":
            # put the object on the ground at the camera target
            t = self.camera.target
            cfg.position = (float(t[0]), float(t[1]), cfg.position[2])
        obj = self.sim.add_object(cfg)
        self.select("object", "", obj.config.name)
        self.project.dirty = True

    def remove_object(self, name: str) -> None:
        self.sim.remove_object(name)
        self.select()
        self.project.dirty = True

    def new_project(self) -> None:
        self.project.new()
        self.camera_windows.clear()
        self.select()

    def open_project(self, path: str) -> None:
        try:
            report = self.project.open(path)
        except Exception as exc:
            self.report_error(f"Could not open project:\n{exc}")
            return
        self.camera_windows.clear()
        self.select()
        if report.view:
            for k, v in report.view.items():
                if hasattr(self.view, k):
                    setattr(self.view, k, v)
        if self.sim.robots:
            self.select("robot", next(iter(self.sim.robots)))
        self.frame_scene()
        if report.errors:
            self.report_error("Project loaded with errors:\n- " + "\n- ".join(report.errors))
        self.status_message = f"Opened project '{report.name}'"

    def save_project(self, path: Optional[str] = None) -> None:
        try:
            p = self.project.save(path, self.view)
            self.status_message = f"Saved {p}"
        except Exception as exc:
            self.report_error(f"Could not save project:\n{exc}")

    def ask_save_as(self) -> None:
        self.file_dialog.open("Save project (choose or type a folder / project.json)", (".json",),
                              self.save_project, mode="save", start_dir=self.last_dir,
                              default_name="project.json")

    def ask_open_project(self) -> None:
        self.file_dialog.open("Open project", (".json",), self.open_project, start_dir=self.last_dir)

    def ask_import_urdf(self) -> None:
        def done(path: str):
            from pathlib import Path
            self.last_dir = str(Path(path).parent)
            self.import_urdf(path)
        self.file_dialog.open("Import URDF", (".urdf",), done, start_dir=self.last_dir)

    def set_ros_enabled(self, enabled: bool) -> None:
        if enabled:
            if not self.bridge.start():
                self.report_error(f"ROS 2 bridge could not start:\n{self.bridge.error}")
        else:
            self.bridge.stop()
        self.sim.config.ros2.enabled = enabled and self.bridge.status == "connected"

    def pick(self, u: float, v: float, aspect: float) -> None:
        origin, d = self.camera.ray(u, v, aspect)
        body, link, frac, _ = self.sim.engine.ray_test(origin, origin + d * self.camera.far)
        if body < 0:
            self.select()
            return
        for r in self.sim.robots.values():
            if r.physics.body == body:
                lname = r.physics.index_to_link.get(link, r.model.root_link)
                if self.selection.robot == r.name and self.selection.kind in ("robot", "link", "joint", "sensor"):
                    self.select("link", r.name, lname)
                else:
                    self.select("robot", r.name)
                return
        name = self.sim.world.find_by_body(body)
        if name:
            self.select("object", "", name)


def deg(x: float) -> float:
    return math.degrees(x)
