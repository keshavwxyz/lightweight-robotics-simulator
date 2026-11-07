"""Planar LiDAR with batched PyBullet ray casts.

The ranges obey REP-117. A beam without a return in [range_min, range_max]
gives +inf. The angle of beam i is angle_min + i * angle_increment. The angle
is counter-clockwise from the +X axis of the sensor, in its XY plane.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .sensor_base import Sensor, SensorConfig


@dataclass
class Lidar2DConfig(SensorConfig):
    type: str = "lidar_2d"
    rate_hz: float = 10.0
    angular_resolution: float = math.radians(1.0)  # rad between beams
    range_min: float = 0.12                        # m
    range_max: float = 12.0                        # m
    horizontal_fov: float = 2 * math.pi            # rad; 2*pi = full 360 deg scan, a smaller value is centered on +X


@dataclass
class LaserScanData:
    stamp: float
    frame_id: str
    angle_min: float
    angle_max: float
    angle_increment: float
    scan_time: float
    time_increment: float
    range_min: float
    range_max: float
    ranges: np.ndarray            # float32, inf = no return
    world_origin: np.ndarray      # for the visualization
    world_points: np.ndarray      # (K,3) hit points for the visualization


class Lidar2D(Sensor):
    TYPE = "lidar_2d"

    def __init__(self, config: Lidar2DConfig, robot):
        super().__init__(config, robot)
        self._dirs_local: Optional[np.ndarray] = None
        self._angles: Optional[np.ndarray] = None
        self._build_beams()

    def _build_beams(self) -> None:
        c: Lidar2DConfig = self.config  # type: ignore[assignment]
        res = max(c.angular_resolution, 1e-4)
        fov = min(max(c.horizontal_fov, res), 2 * math.pi)
        if fov >= 2 * math.pi - 1e-9:
            n = max(3, int(round(2 * math.pi / res)))
            inc = 2 * math.pi / n
            self._angles = -math.pi + inc * np.arange(n)
        else:
            # Partial scan: beams from -fov/2 to +fov/2, with the two limits
            # (the same as min_angle and max_angle of a Gazebo gpu_lidar).
            n = max(2, int(round(fov / res)) + 1)
            inc = fov / (n - 1)
            self._angles = -fov / 2 + inc * np.arange(n)
        self._dirs_local = np.stack([np.cos(self._angles), np.sin(self._angles), np.zeros(n)], axis=1)
        self._inc = inc
        self._cfg_key = (c.angular_resolution, c.horizontal_fov)

    @property
    def num_beams(self) -> int:
        return len(self._angles)  # type: ignore[arg-type]

    def update(self, sim_time: float) -> LaserScanData:
        c: Lidar2DConfig = self.config  # type: ignore[assignment]
        if self._cfg_key != (c.angular_resolution, c.horizontal_fov):
            self._build_beams()
        T = self.world_transform()
        origin = T[:3, 3]
        dirs = self._dirs_local @ T[:3, :3].T            # (N,3) world directions
        starts = origin + dirs * c.range_min
        ends = origin + dirs * c.range_max
        frac, _ = self.robot.engine.ray_test_batch(starts, ends)
        hit = frac < 1.0
        ranges = np.full(len(frac), np.inf, dtype=np.float32)
        ranges[hit] = (c.range_min + frac[hit] * (c.range_max - c.range_min)).astype(np.float32)
        pts = starts[hit] + (ends[hit] - starts[hit]) * frac[hit, None]
        angles = self._angles
        return LaserScanData(stamp=sim_time, frame_id=self.frame_id, angle_min=float(angles[0]),
                             angle_max=float(angles[-1]), angle_increment=self._inc,
                             scan_time=self.period if math.isfinite(self.period) else 0.0,
                             time_increment=0.0, range_min=c.range_min, range_max=c.range_max,
                             ranges=ranges, world_origin=origin.copy(), world_points=pts)
