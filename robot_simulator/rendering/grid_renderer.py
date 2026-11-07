"""Ground plane and grid.

The fragment shader draws the checker pattern and the grid of the ground plane.
If the scene has no ground plane, a grid of lines is drawn.
"""
from __future__ import annotations

import numpy as np

from ..utils.transforms import invert_transform, scale_matrix
from .mesh_renderer import MeshLibrary
from .shader_manager import ShaderManager


class GridRenderer:
    def __init__(self, shaders: ShaderManager, meshes: MeshLibrary):
        self.shaders = shaders
        self.meshes = meshes

    def draw_grounds(self, grounds, view, proj, eye, show_grid: bool, far: float) -> None:
        if not grounds:
            return
        prog = self.shaders.ground
        prog.use()
        prog.mat4("u_view", view)
        prog.mat4("u_proj", proj)
        prog.vec3("u_eye", eye)
        prog.int("u_grid", 1 if show_grid else 0)
        size = min(far * 2.0, 400.0)
        prog.float("u_fade", size * 0.5)
        quad = self.meshes.get("quad")
        for g in grounds:
            # Keep a finite quad below the eye. The plane then looks infinite.
            local_eye = invert_transform(g.model) @ np.append(eye, 1.0)
            offset = np.eye(4)
            offset[0, 3] = np.floor(local_eye[0])
            offset[1, 3] = np.floor(local_eye[1])
            prog.mat4("u_model", g.model @ offset @ scale_matrix((size, size, 1.0)))
            prog.vec4("u_color", g.color)
            quad.draw()

    @staticmethod
    def fallback_grid_lines(extent: int = 10) -> list[tuple]:
        lines = []
        for i in range(-extent, extent + 1):
            a = 0.35 if i else 0.0
            if i == 0:
                continue
            col = (0.55, 0.55, 0.58, a)
            lines.append((np.array([i, -extent, 0.0]), np.array([i, extent, 0.0]), col))
            lines.append((np.array([-extent, i, 0.0]), np.array([extent, i, 0.0]), col))
        lines.append((np.array([-extent, 0, 0.0]), np.array([extent, 0, 0.0]), (0.85, 0.25, 0.25, 0.9)))
        lines.append((np.array([0, -extent, 0.0]), np.array([0, extent, 0.0]), (0.25, 0.8, 0.3, 0.9)))
        return lines


