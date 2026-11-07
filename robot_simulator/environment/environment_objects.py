"""Simple environment objects. Only primitive shapes are available (no CAD)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

SHAPES = ("box", "cylinder", "sphere", "plane")


@dataclass
class EnvObjectConfig:
    name: str = "object"
    shape: str = "box"                                   # box | cylinder | sphere | plane
    size: tuple[float, float, float] = (1.0, 1.0, 1.0)   # box full extents [m]
    radius: float = 0.5                                  # cylinder / sphere [m]
    length: float = 1.0                                  # cylinder height [m]
    position: tuple[float, float, float] = (0.0, 0.0, 0.5)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    mass: float = 0.0                                    # kg; 0 = static
    color: tuple[float, float, float, float] = (0.6, 0.6, 0.65, 1.0)
    collision_enabled: bool = True
    visual_enabled: bool = True
    friction: Optional[float] = None                     # None = physics default
    restitution: Optional[float] = None

    @property
    def is_static(self) -> bool:
        return self.mass <= 0.0

    def dims(self) -> dict:
        return {"size": self.size, "radius": self.radius, "length": self.length}

    def geometry_key(self) -> tuple:
        return (self.shape, tuple(self.size), self.radius, self.length)


class EnvironmentObject:
    """Object in the simulation: configuration and PyBullet body id."""

    def __init__(self, config: EnvObjectConfig, body: int):
        self.config = config
        self.body = body
        self._built_geometry = config.geometry_key()
        self._built_mass = config.mass

    def needs_rebuild(self) -> bool:
        return (self.config.geometry_key() != self._built_geometry) or (self.config.mass != self._built_mass)


def preset(kind: str, name: str) -> EnvObjectConfig:
    if kind == "ground_plane":
        return EnvObjectConfig(name=name, shape="plane", position=(0, 0, 0), color=(0.35, 0.37, 0.40, 1.0))
    if kind == "box":
        return EnvObjectConfig(name=name, shape="box", size=(0.5, 0.5, 0.5), position=(1.5, 0, 0.25),
                               color=(0.80, 0.55, 0.25, 1.0))
    if kind == "cylinder":
        return EnvObjectConfig(name=name, shape="cylinder", radius=0.25, length=0.8, position=(1.5, 1.0, 0.4),
                               color=(0.30, 0.60, 0.80, 1.0))
    if kind == "sphere":
        return EnvObjectConfig(name=name, shape="sphere", radius=0.25, position=(1.5, -1.0, 0.25),
                               color=(0.70, 0.30, 0.35, 1.0))
    if kind == "wall":
        return EnvObjectConfig(name=name, shape="box", size=(4.0, 0.1, 1.0), position=(0, 3.0, 0.5),
                               color=(0.75, 0.75, 0.72, 1.0))
    if kind == "dynamic_box":
        return EnvObjectConfig(name=name, shape="box", size=(0.3, 0.3, 0.3), position=(1.0, 0, 1.0), mass=1.0,
                               color=(0.35, 0.70, 0.40, 1.0))
    raise ValueError(f"Unknown preset '{kind}'")


PRESETS = ("ground_plane", "box", "cylinder", "sphere", "wall", "dynamic_box")
