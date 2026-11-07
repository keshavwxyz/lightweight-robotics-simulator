"""URDF joint description data structures."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from .link import Origin

SUPPORTED_JOINT_TYPES = ("fixed", "revolute", "continuous", "prismatic")
ROTATIONAL_JOINT_TYPES = ("revolute", "continuous")
LINEAR_JOINT_TYPES = ("prismatic",)


@dataclass
class JointLimit:
    lower: float = 0.0       # rad or m
    upper: float = 0.0       # rad or m
    effort: float = 0.0      # N m or N
    velocity: float = 0.0    # rad/s or m/s


@dataclass
class JointDynamics:
    damping: float = 0.0
    friction: float = 0.0


@dataclass
class Mimic:
    joint: str
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: Origin = field(default_factory=Origin)
    axis: tuple[float, float, float] = (1.0, 0.0, 0.0)
    limit: Optional[JointLimit] = None
    dynamics: JointDynamics = field(default_factory=JointDynamics)
    mimic: Optional[Mimic] = None
    # The type in the URDF. It is different from `type` if the parser changed an
    # unsupported type ("floating", "planar") to "fixed".
    original_type: str = ""

    @property
    def is_movable(self) -> bool:
        return self.type != "fixed"

    @property
    def is_rotational(self) -> bool:
        return self.type in ROTATIONAL_JOINT_TYPES

    @property
    def is_linear(self) -> bool:
        return self.type in LINEAR_JOINT_TYPES

    @property
    def has_position_limits(self) -> bool:
        return (self.type in ("revolute", "prismatic") and self.limit is not None
                and self.limit.upper > self.limit.lower)
