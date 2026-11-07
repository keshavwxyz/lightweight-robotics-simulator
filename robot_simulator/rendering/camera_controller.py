"""Orbit camera for the viewport (the world Z axis points up).

  * orbit : left mouse drag
  * pan   : middle mouse drag, or right mouse drag
  * zoom  : mouse wheel
  * focus : `focus(point, radius)` (F key on the selected object)
"""
from __future__ import annotations

import math

import numpy as np

from ..utils.transforms import look_at, perspective


class OrbitCamera:
    def __init__(self):
        self.target = np.array([0.0, 0.0, 0.3])
        self.distance = 4.0
        self.yaw = math.radians(145.0)   # view from behind and to the left of +X
        self.pitch = math.radians(28.0)
        self.fovy = math.radians(50.0)
        self.near = 0.01
        self.far = 500.0

    @property
    def eye(self) -> np.ndarray:
        cp = math.cos(self.pitch)
        return self.target + self.distance * np.array([cp * math.cos(self.yaw), cp * math.sin(self.yaw),
                                                       math.sin(self.pitch)])

    def view_matrix(self) -> np.ndarray:
        return look_at(self.eye, self.target, (0, 0, 1))

    def projection_matrix(self, aspect: float) -> np.ndarray:
        return perspective(self.fovy, max(aspect, 1e-3), self.near, self.far)

    def orbit(self, dx: float, dy: float) -> None:
        self.yaw -= dx * 0.006
        self.pitch = min(max(self.pitch + dy * 0.006, math.radians(-89)), math.radians(89))

    def pan(self, dx: float, dy: float, viewport_height: float) -> None:
        V = self.view_matrix()
        right, up = V[0, :3], V[1, :3]
        scale = 2.0 * self.distance * math.tan(self.fovy / 2) / max(viewport_height, 1.0)
        self.target = self.target - right * dx * scale + up * dy * scale

    def zoom(self, wheel: float) -> None:
        self.distance = min(max(self.distance * (0.88 ** wheel), 0.05), 200.0)

    def focus(self, point, radius: float = 0.5) -> None:
        self.target = np.asarray(point, dtype=float).copy()
        self.distance = max(radius * 3.0, 0.3)

    def ray(self, u: float, v: float, aspect: float) -> tuple[np.ndarray, np.ndarray]:
        """Return the world ray through the viewport point (u, v). u and v are in [-1, 1]; v points up."""
        inv = np.linalg.inv(self.projection_matrix(aspect) @ self.view_matrix())
        p0 = inv @ np.array([u, v, -1.0, 1.0])
        p1 = inv @ np.array([u, v, 1.0, 1.0])
        p0, p1 = p0[:3] / p0[3], p1[:3] / p1[3]
        d = p1 - p0
        return p0, d / np.linalg.norm(d)
