"""Makes the draw lists for each frame from the simulation state.

The renderer does not use PyBullet directly. This module reads the link world
transforms from the physics layer (`RobotModel.world_transforms`). It reads
the geometry from the URDF model. The geometry of each robot (visual origins,
scales and colours) is calculated one time and kept. For each frame, each
visual needs only one 4x4 matrix multiplication.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ..utils.transforms import pose_to_matrix, scale_matrix

DEFAULT_COLOR = (0.72, 0.72, 0.75, 1.0)
COLLISION_COLOR = (0.15, 0.95, 0.35, 1.0)


@dataclass
class DrawItem:
    mesh_key: str
    model: np.ndarray
    color: tuple
    use_vcolor: bool = False
    highlight: float = 0.0
    wireframe: bool = False


@dataclass
class GroundItem:
    model: np.ndarray        # plane frame
    color: tuple


@dataclass
class RenderScene:
    items: list[DrawItem] = field(default_factory=list)
    transparent: list[DrawItem] = field(default_factory=list)
    grounds: list[GroundItem] = field(default_factory=list)
    lines: list[tuple] = field(default_factory=list)     # (p0, p1, rgba)
    points: list[tuple] = field(default_factory=list)    # (Nx3 array, rgba)


@dataclass
class Selection:
    kind: str = ""          # "robot" | "link" | "joint" | "sensor" | "object" | ""
    robot: str = ""
    name: str = ""          # link / joint / sensor / object name


@dataclass
class ViewOptions:
    show_grid: bool = True
    show_frames: bool = False
    show_collision: bool = False
    show_sensors: bool = True
    show_joint_axes: bool = False
    show_visuals: bool = True


def _geom_entry(geom, origin_matrix: np.ndarray) -> Optional[tuple[str, np.ndarray]]:
    if geom.type in ("box", "cylinder", "sphere"):
        return geom.type, origin_matrix @ scale_matrix(geom.scale_vector())
    if geom.type == "mesh" and geom.resolved_path:
        return geom.resolved_path, origin_matrix @ scale_matrix(geom.scale)
    return None


class SceneBuilder:
    def __init__(self, mesh_library=None):
        self.meshes = mesh_library     # optional; finds meshes with vertex colours
        self._visual_cache: dict[tuple, list] = {}
        self._collision_cache: dict[tuple, list] = {}

    def _robot_visuals(self, robot) -> list:
        key = (id(robot.model),)
        cached = self._visual_cache.get(key)
        if cached is None:
            cached = []
            for link in robot.model.links.values():
                for v in link.visuals:
                    entry = _geom_entry(v.geometry, v.origin.matrix())
                    if entry is None:
                        continue
                    color = v.material.rgba if (v.material and v.material.rgba) else None
                    cached.append((link.name, entry[0], entry[1], color))
            self._visual_cache[key] = cached
        return cached

    def _robot_collisions(self, robot) -> list:
        key = (id(robot.model),)
        cached = self._collision_cache.get(key)
        if cached is None:
            cached = []
            for link in robot.model.links.values():
                for c in link.collisions:
                    entry = _geom_entry(c.geometry, c.origin.matrix())
                    if entry is not None:
                        cached.append((link.name, entry[0], entry[1]))
            self._collision_cache[key] = cached
        return cached

    def forget(self) -> None:
        """Remove the cached geometry of robots that were removed."""
        self._visual_cache.clear()
        self._collision_cache.clear()

    def build(self, sim, opts: ViewOptions, selection: Optional[Selection] = None,
              overlays: bool = True) -> RenderScene:
        scene = RenderScene()
        sel = selection or Selection()

        for robot in sim.robots.values():
            wt = robot.physics.update_link_transforms()
            robot_selected = overlays and sel.robot == robot.name and sel.kind in ("robot",)
            if opts.show_visuals or not overlays:
                for link, mesh_key, local, color in self._robot_visuals(robot):
                    hl = 0.0
                    if overlays and sel.robot == robot.name and (robot_selected or (sel.kind == "link" and sel.name == link)):
                        hl = 0.6 if sel.kind == "link" else 0.25
                    use_vc = False
                    if color is None:
                        mesh = self.meshes.get(mesh_key) if (self.meshes and mesh_key not in ("box", "cylinder", "sphere")) else None
                        use_vc = bool(mesh is not None and (mesh.has_vertex_colors or mesh.texture is not None))
                        color = DEFAULT_COLOR
                    scene.items.append(DrawItem(mesh_key, wt[link] @ local, color, use_vc, hl))
            if overlays and opts.show_collision:
                for link, mesh_key, local in self._robot_collisions(robot):
                    scene.transparent.append(DrawItem(mesh_key, wt[link] @ local, COLLISION_COLOR, wireframe=True))
            if overlays:
                self._robot_overlays(scene, robot, wt, opts, sel)

        for name, obj in sim.world.objects.items():
            c = obj.config
            pos, orn = sim.engine.body_pose(obj.body)
            T = pose_to_matrix(pos, orn)
            if c.shape == "plane":
                if c.visual_enabled or not overlays:
                    scene.grounds.append(GroundItem(T, c.color))
                continue
            hl = 0.5 if (overlays and sel.kind == "object" and sel.name == name) else 0.0
            geom_scale = {"box": c.size, "cylinder": (c.radius, c.radius, c.length),
                          "sphere": (c.radius, c.radius, c.radius)}[c.shape]
            M = T @ scale_matrix(geom_scale)
            if c.visual_enabled:
                scene.items.append(DrawItem(c.shape, M, c.color, False, hl))
            if overlays and opts.show_collision and c.collision_enabled:
                scene.transparent.append(DrawItem(c.shape, M, COLLISION_COLOR, wireframe=True))
            if overlays and hl > 0 and not c.visual_enabled:
                scene.transparent.append(DrawItem(c.shape, M, (1.0, 0.8, 0.2, 1.0), wireframe=True))
        return scene

    @staticmethod
    def _axes(scene: RenderScene, T: np.ndarray, length: float, alpha: float = 1.0) -> None:
        o = T[:3, 3]
        for k, col in enumerate(((1.0, 0.2, 0.2, alpha), (0.2, 1.0, 0.2, alpha), (0.3, 0.5, 1.0, alpha))):
            scene.lines.append((o, o + T[:3, k] * length, col))

    def _robot_overlays(self, scene: RenderScene, robot, wt, opts: ViewOptions, sel: Selection) -> None:
        if opts.show_frames:
            for link in robot.model.links:
                self._axes(scene, wt[link], 0.08, 0.9)
        if sel.robot == robot.name and sel.kind == "link" and sel.name in wt:
            self._axes(scene, wt[sel.name], 0.15)
        show_axis_for = None
        if sel.robot == robot.name and sel.kind == "joint":
            show_axis_for = sel.name
        for j in robot.model.joints.values():
            if not j.is_movable:
                continue
            if not (opts.show_joint_axes or j.name == show_axis_for):
                continue
            Tj = wt[j.parent] @ j.origin.matrix()
            o = Tj[:3, 3]
            axis = Tj[:3, :3] @ np.asarray(j.axis)
            col = (1.0, 0.85, 0.1, 1.0) if j.is_rotational else (0.1, 0.9, 0.9, 1.0)
            scene.lines.append((o - axis * 0.12, o + axis * 0.12, col))
        if not opts.show_sensors:
            return
        for s in robot.sensors.values():
            T = wt[s.config.link] @ s.mount_transform
            selected = sel.robot == robot.name and sel.kind == "sensor" and sel.name == s.name
            if s.TYPE == "lidar_2d":
                data = s.latest
                if data is not None and len(data.world_points):
                    scene.points.append((data.world_points, (1.0, 0.25, 0.2, 1.0)))
                    if selected:
                        for p in data.world_points[::4]:
                            scene.lines.append((data.world_origin, p, (1.0, 0.3, 0.2, 0.35)))
                self._axes(scene, T, 0.06)
            elif s.TYPE == "camera":
                self._frustum(scene, T, s.config, (0.3, 0.8, 1.0, 1.0 if selected else 0.6))
            elif s.TYPE == "imu":
                self._axes(scene, T, 0.1 if selected else 0.06)

    @staticmethod
    def _frustum(scene: RenderScene, T: np.ndarray, c, col) -> None:
        d = 0.25
        hw = d * math.tan(c.horizontal_fov / 2)
        hh = hw * c.height / max(c.width, 1)
        o = T[:3, 3]
        corners = [T[:3, :3] @ np.array([d, sy * hw, sz * hh]) + o for sy, sz in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
        for k in range(4):
            scene.lines.append((o, corners[k], col))
            scene.lines.append((corners[k], corners[(k + 1) % 4], col))
        up = T[:3, :3] @ np.array([d, 0, hh * 1.4]) + o
        scene.lines.append((corners[0], up, col))
        scene.lines.append((corners[1], up, col))
