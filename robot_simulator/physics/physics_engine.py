"""PyBullet physics backend (DIRECT mode only).

Only this module connects to PyBullet. It does not use the rendering, ROS 2
or the user interface.
"""
from __future__ import annotations

import logging
from typing import Optional, Sequence

import numpy as np
import pybullet as p

from ..core.configuration import PhysicsConfig

log = logging.getLogger(__name__)


class PhysicsEngine:
    def __init__(self, config: Optional[PhysicsConfig] = None):
        self.config = config or PhysicsConfig()
        # DIRECT mode: PyBullet has no GUI and no OpenGL context.
        self.client = p.connect(p.DIRECT)
        if self.client < 0:
            raise RuntimeError("Could not connect to PyBullet in DIRECT mode")
        self.sim_time = 0.0
        self.step_count = 0
        self.apply_config()

    def apply_config(self) -> None:
        c = self.config
        p.setGravity(*c.gravity, physicsClientId=self.client)
        p.setPhysicsEngineParameter(
            fixedTimeStep=c.timestep,
            numSolverIterations=int(c.solver_iterations),
            numSubSteps=0,
            erp=c.contact_erp,
            contactERP=c.contact_erp,
            enableConeFriction=1,
            deterministicOverlappingPairs=1,
            physicsClientId=self.client,
        )

    @property
    def timestep(self) -> float:
        return self.config.timestep

    def step(self) -> None:
        p.stepSimulation(physicsClientId=self.client)
        self.step_count += 1
        self.sim_time = self.step_count * self.config.timestep

    def reset_time(self) -> None:
        self.step_count = 0
        self.sim_time = 0.0

    def load_urdf(self, path: str, position: Sequence[float], orientation: Sequence[float],
                  fixed_base: bool, self_collision: bool) -> int:
        flags = p.URDF_USE_INERTIA_FROM_FILE | p.URDF_MAINTAIN_LINK_ORDER
        if self_collision:
            flags |= p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
        body = p.loadURDF(path, basePosition=list(position), baseOrientation=list(orientation),
                          useFixedBase=bool(fixed_base), flags=flags, physicsClientId=self.client)
        if body < 0:
            raise RuntimeError(f"PyBullet failed to load URDF {path}")
        return body

    def create_primitive(self, shape: str, dims: dict, position: Sequence[float],
                         orientation: Sequence[float], mass: float) -> int:
        c = self.client
        if shape == "box":
            half = [d / 2.0 for d in dims["size"]]
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=c)
        elif shape == "cylinder":
            col = p.createCollisionShape(p.GEOM_CYLINDER, radius=dims["radius"], height=dims["length"],
                                         physicsClientId=c)
        elif shape == "sphere":
            col = p.createCollisionShape(p.GEOM_SPHERE, radius=dims["radius"], physicsClientId=c)
        elif shape == "plane":
            col = p.createCollisionShape(p.GEOM_PLANE, planeNormal=[0, 0, 1], physicsClientId=c)
            mass = 0.0
        else:
            raise ValueError(f"Unsupported primitive '{shape}'")
        return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=col, baseVisualShapeIndex=-1,
                                 basePosition=list(position), baseOrientation=list(orientation),
                                 physicsClientId=c)

    def remove_body(self, body: int) -> None:
        p.removeBody(body, physicsClientId=self.client)

    def set_body_dynamics(self, body: int, link: int = -1, **kwargs) -> None:
        p.changeDynamics(body, link, physicsClientId=self.client, **kwargs)

    def reset_body_pose(self, body: int, position: Sequence[float], orientation: Sequence[float]) -> None:
        p.resetBasePositionAndOrientation(body, list(position), list(orientation), physicsClientId=self.client)
        p.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0], physicsClientId=self.client)

    def body_pose(self, body: int):
        return p.getBasePositionAndOrientation(body, physicsClientId=self.client)

    def ray_test_batch(self, origins: np.ndarray, ends: np.ndarray) -> np.ndarray:
        """Cast rays.

        Return an (N,) array of hit fractions (1.0 = no hit) and an (N,) array
        of hit body ids (-1 = no body).
        """
        n = len(origins)
        fractions = np.ones(n)
        bodies = np.full(n, -1, dtype=np.int64)
        step = p.MAX_RAY_INTERSECTION_BATCH_SIZE - 1
        for s in range(0, n, step):
            res = p.rayTestBatch(origins[s:s + step].tolist(), ends[s:s + step].tolist(),
                                 physicsClientId=self.client)
            for i, r in enumerate(res):
                if r[0] >= 0:
                    bodies[s + i] = r[0]
                    fractions[s + i] = r[2]
        return fractions, bodies  # type: ignore[return-value]

    def ray_test(self, start: Sequence[float], end: Sequence[float]):
        r = p.rayTest(list(start), list(end), physicsClientId=self.client)[0]
        return r[0], r[1], r[2], r[3]  # body, link, fraction, position

    def contacts(self, body_a: int, body_b: Optional[int] = None):
        if body_b is None:
            return p.getContactPoints(bodyA=body_a, physicsClientId=self.client)
        return p.getContactPoints(bodyA=body_a, bodyB=body_b, physicsClientId=self.client)

    def disconnect(self) -> None:
        if self.client >= 0:
            try:
                p.disconnect(physicsClientId=self.client)
            except p.error:
                pass
            self.client = -1

    def __del__(self):  # pragma: no cover
        try:
            self.disconnect()
        except Exception:
            pass
