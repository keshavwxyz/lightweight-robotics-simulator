"""Perfect (noise-free) IMU.

  * orientation       : rotation of the IMU frame in the world frame. It comes
                        from the parent link transform and the mount.
  * angular_velocity  : angular velocity of the parent link, in the IMU
                        frame [rad/s].
  * linear_acceleration: specific force f = a - g in the IMU frame [m/s^2].
                        a is the acceleration of the IMU point. It is the
                        difference of the world velocity between two IMU
                        updates, divided by the time. A level IMU at rest
                        gives (0, 0, +9.81).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..utils.transforms import matrix_to_quat
from .sensor_base import Sensor, SensorConfig


@dataclass
class ImuConfig(SensorConfig):
    type: str = "imu"
    rate_hz: float = 100.0


@dataclass
class ImuReading:
    stamp: float
    frame_id: str
    orientation: tuple[float, float, float, float]   # x, y, z, w
    angular_velocity: np.ndarray                     # rad/s (IMU frame)
    linear_acceleration: np.ndarray                  # m/s^2 (IMU frame, specific force)


class Imu(Sensor):
    TYPE = "imu"

    def __init__(self, config: ImuConfig, robot):
        super().__init__(config, robot)
        self._prev_v: Optional[np.ndarray] = None
        self._prev_t = 0.0

    def update(self, sim_time: float) -> ImuReading:
        phys = self.robot.physics
        T = self.world_transform()
        R = T[:3, :3]
        p_imu = T[:3, 3]
        com, v_com, w = phys.link_velocity(self.config.link)
        v_imu = v_com + np.cross(w, p_imu - com)      # rigid-body velocity of the IMU point

        if self._prev_v is None or sim_time <= self._prev_t:
            a = np.zeros(3)
        else:
            a = (v_imu - self._prev_v) / (sim_time - self._prev_t)
        self._prev_v, self._prev_t = v_imu, sim_time

        g = np.asarray(self.robot.gravity, dtype=float)
        specific_force = R.T @ (a - g)
        return ImuReading(stamp=sim_time, frame_id=self.frame_id, orientation=matrix_to_quat(R),
                          angular_velocity=R.T @ w, linear_acceleration=specific_force)

    def reset(self) -> None:
        super().reset()
        self._prev_v = None
        self._prev_t = 0.0
