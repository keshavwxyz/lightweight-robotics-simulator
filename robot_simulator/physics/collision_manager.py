"""Collision filtering and contact queries."""
from __future__ import annotations

import pybullet as p

# Collision groups (bit masks). By default, all bodies collide with all bodies.
GROUP_DEFAULT = 0x1
MASK_ALL = 0xFFFF


class CollisionManager:
    def __init__(self, client: int):
        self.client = client
        self._enabled_global = True
        self._body_enabled: dict[int, bool] = {}

    def register(self, body: int, enabled: bool = True) -> None:
        self._body_enabled[body] = enabled
        self._apply(body)

    def unregister(self, body: int) -> None:
        self._body_enabled.pop(body, None)

    def set_body_enabled(self, body: int, enabled: bool) -> None:
        self._body_enabled[body] = enabled
        self._apply(body)

    def set_global_enabled(self, enabled: bool) -> None:
        if enabled == self._enabled_global:
            return
        self._enabled_global = enabled
        for body in self._body_enabled:
            self._apply(body)

    def _apply(self, body: int) -> None:
        on = self._enabled_global and self._body_enabled.get(body, True)
        group, mask = (GROUP_DEFAULT, MASK_ALL) if on else (0, 0)
        for link in range(-1, p.getNumJoints(body, physicsClientId=self.client)):
            p.setCollisionFilterGroupMask(body, link, group, mask, physicsClientId=self.client)

    def contact_count(self, body_a: int, body_b: int) -> int:
        return len(p.getContactPoints(bodyA=body_a, bodyB=body_b, physicsClientId=self.client))
