"""Sensor subsystem. There are three sensor types: camera, perfect IMU and 2D LiDAR."""
from __future__ import annotations

from ..core.configuration import from_dict
from .camera import Camera, CameraConfig
from .imu import Imu, ImuConfig
from .lidar_2d import Lidar2D, Lidar2DConfig
from .sensor_base import Sensor, SensorConfig

SENSOR_TYPES = {
    "camera": (Camera, CameraConfig),
    "imu": (Imu, ImuConfig),
    "lidar_2d": (Lidar2D, Lidar2DConfig),
}


def sensor_config_from_dict(data: dict) -> SensorConfig:
    t = data.get("type")
    if t not in SENSOR_TYPES:
        raise ValueError(f"Unknown sensor type '{t}' (supported: {', '.join(SENSOR_TYPES)})")
    return from_dict(SENSOR_TYPES[t][1], data)


def create_sensor(config: SensorConfig, robot) -> Sensor:
    if config.type not in SENSOR_TYPES:
        raise ValueError(f"Unknown sensor type '{config.type}'")
    return SENSOR_TYPES[config.type][0](config, robot)


__all__ = ["Sensor", "SensorConfig", "Camera", "CameraConfig", "Imu", "ImuConfig", "Lidar2D",
           "Lidar2DConfig", "SENSOR_TYPES", "create_sensor", "sensor_config_from_dict"]
