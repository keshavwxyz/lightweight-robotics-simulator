"""Base class and configuration of all sensors.

A sensor is attached to a robot link with a fixed offset (xyz [m], rpy [rad]).
The sensor scheduler updates each sensor at its own rate, not more frequently.
The sensors do not use the rendering or ROS 2. They make data objects that
other subsystems use.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

import numpy as np

from ..utils.transforms import make_transform


@dataclass
class SensorConfig:
    name: str = "sensor"
    type: str = ""
    link: str = ""                                   # parent link
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rate_hz: float = 30.0
    enabled: bool = True
    topic: str = ""                                  # "" = default topic
    # "" = the simulator makes the frame "<name>_link" (camera: "<name>_optical_frame") and
    # publishes its static TF. Another value is an existing frame, for example a URDF link
    # from robot_state_publisher. The messages then use it, and the simulator sends no TF.
    frame_id: str = ""


class Sensor:
    TYPE = ""

    def __init__(self, config: SensorConfig, robot):  # robot: RobotInstance (no import, to prevent an import cycle)
        self.config = config
        self.robot = robot
        self.next_update_time = 0.0
        self.latest: Optional[Any] = None
        self.update_count = 0
        self._mount = make_transform(config.xyz, config.rpy)

    @property
    def name(self) -> str:
        return self.config.name

    @property
    def frame_override(self) -> bool:
        return bool(self.config.frame_id)

    @property
    def frame_id(self) -> str:
        if self.config.frame_id:
            return self.config.frame_id
        return f"{self.robot.tf_prefix}{self.config.name}_link"

    def refresh_mount(self) -> None:
        self._mount = make_transform(self.config.xyz, self.config.rpy)

    @property
    def mount_transform(self) -> np.ndarray:
        return self._mount

    def world_transform(self) -> np.ndarray:
        return self.robot.physics.link_transform(self.config.link) @ self._mount

    @property
    def period(self) -> float:
        return 1.0 / self.config.rate_hz if self.config.rate_hz > 0 else float("inf")

    def due(self, sim_time: float) -> bool:
        return self.config.enabled and sim_time + 1e-9 >= self.next_update_time

    def tick(self, sim_time: float, dt_physics: float) -> Optional[Any]:
        """Update the sensor. The scheduler calls this when the sensor is due.

        The next update time is on a fixed grid. Thus, the rate does not drift.
        """
        out = self.update(sim_time)
        self.update_count += 1
        self.next_update_time += self.period
        if self.next_update_time <= sim_time:  # for example, after a rate change
            self.next_update_time = sim_time + self.period
        self.latest = out
        return out

    def update(self, sim_time: float) -> Optional[Any]:  # pragma: no cover - abstract
        raise NotImplementedError

    def reset(self) -> None:
        self.next_update_time = 0.0
        self.latest = None
