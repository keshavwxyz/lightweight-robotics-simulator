"""Configuration data classes.

Each configuration object is a dataclass. It converts to and from a JSON
dictionary. Project files and `config.json` use these dictionaries. All units
are SI.
"""
from __future__ import annotations

import dataclasses
import json
import logging
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any, Optional, Type, TypeVar

log = logging.getLogger(__name__)
T = TypeVar("T")


def to_dict(obj: Any) -> Any:
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        return {f.name: to_dict(getattr(obj, f.name)) for f in fields(obj)}
    if isinstance(obj, (list, tuple)):
        return [to_dict(v) for v in obj]
    if isinstance(obj, dict):
        return {k: to_dict(v) for k, v in obj.items()}
    return obj


def from_dict(cls: Type[T], data: Optional[dict]) -> T:
    """Make dataclass `cls` from `data`.

    The function ignores unknown keys and logs a warning. Missing keys keep
    their default values. The `nested` metadata entry gives nested dataclasses.
    """
    if data is None:
        return cls()
    kwargs = {}
    names = {f.name: f for f in fields(cls)}  # type: ignore[arg-type]
    for k, v in data.items():
        if k not in names:
            log.warning("Unknown configuration key '%s' for %s ignored", k, cls.__name__)
            continue
        f = names[k]
        nested = f.metadata.get("nested")
        if nested is not None:
            if f.metadata.get("list"):
                v = [from_dict(nested, item) for item in (v or [])]
            else:
                v = from_dict(nested, v)
        elif isinstance(v, list) and isinstance(f.default, tuple):
            v = tuple(v)
        kwargs[k] = v
    return cls(**kwargs)


@dataclass
class PhysicsConfig:
    frequency_hz: float = 240.0          # physics frequency; timestep = 1 / frequency
    gravity: tuple[float, float, float] = (0.0, 0.0, -9.81)   # m/s^2
    solver_iterations: int = 50
    collision_detection: bool = True     # collision detection on or off for all bodies
    default_friction: float = 0.8        # lateral friction coefficient of environment objects
    default_restitution: float = 0.0
    contact_erp: float = 0.2
    max_steps_per_frame: int = 64        # maximum catch-up steps for each frame

    @property
    def timestep(self) -> float:
        return 1.0 / self.frequency_hz


@dataclass
class Ros2Config:
    enabled: bool = True
    node_name: str = "lightweight_sim"
    state_publish_rate_hz: float = 50.0  # joint_states, tf, odom
    clock_publish_rate_hz: float = 100.0 # /clock (simulation time)
    publish_clock: bool = True
    cmd_vel_timeout_s: float = 0.5
    cmd_vel_stamped: bool = False        # use geometry_msgs/TwistStamped, not Twist
    domain_id: Optional[int] = None      # None = use ROS_DOMAIN_ID of the environment


@dataclass
class RenderConfig:
    window_width: int = 1600
    window_height: int = 900
    vsync: bool = True
    show_grid: bool = True
    show_frames: bool = False
    show_collision: bool = False
    show_sensors: bool = True
    show_joint_axes: bool = False
    background: tuple[float, float, float] = (0.16, 0.17, 0.20)


@dataclass
class AppConfig:
    physics: PhysicsConfig = field(default_factory=PhysicsConfig, metadata={"nested": PhysicsConfig})
    ros2: Ros2Config = field(default_factory=Ros2Config, metadata={"nested": Ros2Config})
    render: RenderConfig = field(default_factory=RenderConfig, metadata={"nested": RenderConfig})
    package_paths: list[str] = field(default_factory=list)

    @classmethod
    def load(cls, path: str | Path) -> "AppConfig":
        p = Path(path)
        if not p.is_file():
            return cls()
        try:
            return from_dict(cls, json.loads(p.read_text()))
        except Exception as exc:
            log.error("Could not read configuration %s: %s; using defaults", p, exc)
            return cls()
