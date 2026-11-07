"""Keeps all environment objects and their PyBullet bodies."""
from __future__ import annotations

import logging
from typing import Optional

from ..physics.collision_manager import CollisionManager
from ..physics.physics_engine import PhysicsEngine
from ..utils.transforms import rpy_to_quat
from .environment_objects import EnvironmentObject, EnvObjectConfig

log = logging.getLogger(__name__)


class WorldManager:
    def __init__(self, engine: PhysicsEngine, collisions: CollisionManager):
        self.engine = engine
        self.collisions = collisions
        self.objects: dict[str, EnvironmentObject] = {}

    def unique_name(self, base: str) -> str:
        if base not in self.objects:
            return base
        i = 1
        while f"{base}_{i}" in self.objects:
            i += 1
        return f"{base}_{i}"

    def add(self, config: EnvObjectConfig) -> EnvironmentObject:
        config.name = self.unique_name(config.name)
        obj = EnvironmentObject(config, self._create_body(config))
        self.objects[config.name] = obj
        return obj

    def _create_body(self, c: EnvObjectConfig) -> int:
        body = self.engine.create_primitive(c.shape, c.dims(), c.position, rpy_to_quat(c.rpy), c.mass)
        pc = self.engine.config
        self.engine.set_body_dynamics(
            body, -1,
            lateralFriction=float(c.friction if c.friction is not None else pc.default_friction),
            restitution=float(c.restitution if c.restitution is not None else pc.default_restitution))
        self.collisions.register(body, c.collision_enabled)
        return body

    def remove(self, name: str) -> None:
        obj = self.objects.pop(name, None)
        if obj is not None:
            self.collisions.unregister(obj.body)
            self.engine.remove_body(obj.body)

    def rename(self, old: str, new: str) -> str:
        if old not in self.objects or not new or new == old:
            return old
        new = self.unique_name(new)
        obj = self.objects.pop(old)
        obj.config.name = new
        self.objects[new] = obj
        return new

    def apply_config(self, name: str) -> None:
        """Send the changed configuration (pose, size, flags) to the physics engine."""
        obj = self.objects[name]
        c = obj.config
        if obj.needs_rebuild():
            self.collisions.unregister(obj.body)
            self.engine.remove_body(obj.body)
            self.objects[name] = EnvironmentObject(c, self._create_body(c))
            return
        self.engine.reset_body_pose(obj.body, c.position, rpy_to_quat(c.rpy))
        self.collisions.set_body_enabled(obj.body, c.collision_enabled)
        pc = self.engine.config
        self.engine.set_body_dynamics(
            obj.body, -1,
            lateralFriction=float(c.friction if c.friction is not None else pc.default_friction),
            restitution=float(c.restitution if c.restitution is not None else pc.default_restitution))

    def reset(self) -> None:
        """Move the dynamic objects back to their initial poses."""
        for obj in self.objects.values():
            if not obj.config.is_static:
                self.engine.reset_body_pose(obj.body, obj.config.position, rpy_to_quat(obj.config.rpy))

    def body_names(self) -> dict[int, str]:
        return {o.body: n for n, o in self.objects.items()}

    def find_by_body(self, body: int) -> Optional[str]:
        for n, o in self.objects.items():
            if o.body == body:
                return n
        return None

    def clear(self) -> None:
        for n in list(self.objects):
            self.remove(n)
