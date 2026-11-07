"""Ground plane: an infinite static collision plane at z = 0."""
from __future__ import annotations

from .environment_objects import EnvObjectConfig, preset


def default_ground(name: str = "ground_plane") -> EnvObjectConfig:
    return preset("ground_plane", name)
