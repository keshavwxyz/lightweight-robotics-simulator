"""URDF link description data structures."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ..utils.transforms import make_transform

GEOMETRY_TYPES = ("box", "cylinder", "sphere", "mesh")


@dataclass
class Origin:
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def matrix(self) -> np.ndarray:
        return make_transform(self.xyz, self.rpy)


@dataclass
class Geometry:
    """One URDF geometry primitive. Sizes are in metres."""
    type: str                                    # box | cylinder | sphere | mesh
    size: tuple[float, float, float] = (0.0, 0.0, 0.0)   # box full extents
    radius: float = 0.0                          # cylinder / sphere
    length: float = 0.0                          # cylinder (along local Z)
    filename: str = ""                           # mesh: as written in the URDF
    resolved_path: Optional[str] = None          # mesh: absolute path on disk (None = unresolved)
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)  # mesh scale

    def scale_vector(self) -> tuple[float, float, float]:
        """Return the scale that the renderer applies to the unit primitive shape."""
        if self.type == "box":
            return self.size
        if self.type == "cylinder":
            return (self.radius, self.radius, self.length)
        if self.type == "sphere":
            return (self.radius, self.radius, self.radius)
        return self.scale


@dataclass
class Material:
    name: str = ""
    rgba: Optional[tuple[float, float, float, float]] = None
    texture: Optional[str] = None


@dataclass
class Visual:
    geometry: Geometry
    origin: Origin = field(default_factory=Origin)
    material: Optional[Material] = None
    name: str = ""


@dataclass
class Collision:
    geometry: Geometry
    origin: Origin = field(default_factory=Origin)
    name: str = ""


@dataclass
class Inertial:
    mass: float = 0.0
    origin: Origin = field(default_factory=Origin)
    # ixx, ixy, ixz, iyy, iyz, izz  (kg m^2)
    inertia: tuple[float, float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def inertia_matrix(self) -> np.ndarray:
        ixx, ixy, ixz, iyy, iyz, izz = self.inertia
        return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])


@dataclass
class Link:
    name: str
    inertial: Optional[Inertial] = None
    visuals: list[Visual] = field(default_factory=list)
    collisions: list[Collision] = field(default_factory=list)
