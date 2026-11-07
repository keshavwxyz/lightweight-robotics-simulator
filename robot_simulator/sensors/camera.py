"""Ideal pinhole RGB camera. The custom OpenGL renderer makes the images.

The camera body frame obeys REP-103 (x forward, y left, z up). The images use
the related optical frame `<name>_optical_frame` (z forward, x right, y down).
There is no noise, blur, distortion or exposure model.

The sensor does not use OpenGL. It calculates the view and projection
matrices. An image backend from the rendering subsystem then makes the image.
Without a backend, the camera makes no images and logs this one time.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

from ..utils.transforms import BODY_FROM_GL, invert_transform, perspective
from .sensor_base import Sensor, SensorConfig

log = logging.getLogger(__name__)

# backend(view 4x4, projection 4x4, width, height) -> (height, width, 3) uint8, row 0 = top
ImageBackend = Callable[[np.ndarray, np.ndarray, int, int], Optional[np.ndarray]]


@dataclass
class CameraConfig(SensorConfig):
    type: str = "camera"
    rate_hz: float = 15.0
    width: int = 640                   # px
    height: int = 480                  # px
    horizontal_fov: float = math.radians(80.0)  # rad
    near: float = 0.05                 # m
    far: float = 50.0                  # m


@dataclass
class CameraImage:
    stamp: float
    frame_id: str          # optical frame
    width: int
    height: int
    data: np.ndarray       # (H, W, 3) uint8 rgb8
    K: tuple               # 3x3 row-major intrinsics
    P: tuple               # 3x4 row-major projection


def intrinsics(width: int, height: int, hfov: float) -> tuple[float, float, float, float]:
    fx = (width / 2.0) / math.tan(hfov / 2.0)
    fy = fx  # square pixels
    cx = (width - 1) / 2.0
    cy = (height - 1) / 2.0
    return fx, fy, cx, cy


class Camera(Sensor):
    TYPE = "camera"

    def __init__(self, config: CameraConfig, robot):
        super().__init__(config, robot)
        self._warned = False

    @property
    def optical_frame_id(self) -> str:
        if self.config.frame_id:          # use this existing optical frame (REP-103) without change
            return self.config.frame_id
        return f"{self.robot.tf_prefix}{self.config.name}_optical_frame"

    def matrices(self) -> tuple[np.ndarray, np.ndarray]:
        c: CameraConfig = self.config  # type: ignore[assignment]
        world_from_gl = self.world_transform() @ BODY_FROM_GL
        view = invert_transform(world_from_gl)
        fx, fy, _, _ = intrinsics(c.width, c.height, c.horizontal_fov)
        vfov = 2.0 * math.atan((c.height / 2.0) / fy)
        proj = perspective(vfov, c.width / c.height, c.near, c.far)
        return view, proj

    def update(self, sim_time: float) -> Optional[CameraImage]:
        backend: Optional[ImageBackend] = getattr(self.robot, "image_backend", None)
        c: CameraConfig = self.config  # type: ignore[assignment]
        if backend is None:
            if not self._warned:
                log.warning("Camera '%s': no OpenGL image backend available; no images are produced", c.name)
                self._warned = True
            return None
        view, proj = self.matrices()
        img = backend(view, proj, int(c.width), int(c.height))
        if img is None:
            return None
        fx, fy, cx, cy = intrinsics(c.width, c.height, c.horizontal_fov)
        K = (fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0)
        P = (fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0)
        return CameraImage(stamp=sim_time, frame_id=self.optical_frame_id, width=int(c.width),
                           height=int(c.height), data=img, K=K, P=P)
