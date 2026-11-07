"""Connection between a `RobotModel` and its PyBullet multibody.

This module reads joint states and link transforms from PyBullet. It keeps
the result for one physics step. Thus, all users in the same step share one
query. It writes the world transforms of the links into the RobotModel.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np
import pybullet as p

from ..robot.robot_model import RobotModel
from ..utils.transforms import invert_transform, matrix_to_pose, pose_to_matrix
from .physics_engine import PhysicsEngine

log = logging.getLogger(__name__)


@dataclass
class JointStateArrays:
    names: list[str]
    position: np.ndarray
    velocity: np.ndarray
    effort: np.ndarray


class RobotPhysics:
    def __init__(self, engine: PhysicsEngine, model: RobotModel, urdf_path: str,
                 position: Sequence[float], orientation: Sequence[float],
                 fixed_base: bool, self_collision: bool, lateral_friction: Optional[float] = None):
        self.engine = engine
        self.client = engine.client
        self.model = model
        self.body = engine.load_urdf(urdf_path, position, orientation, fixed_base, self_collision)

        # The PyBullet joint index is the index of its child link. The base link is -1.
        self.link_index: dict[str, int] = {model.root_link: -1}
        self.joint_index: dict[str, int] = {}
        n = p.getNumJoints(self.body, physicsClientId=self.client)
        for i in range(n):
            info = p.getJointInfo(self.body, i, physicsClientId=self.client)
            jname = info[1].decode()
            lname = info[12].decode()
            self.joint_index[jname] = i
            self.link_index[lname] = i
        missing = [j for j in model.joints if j not in self.joint_index]
        if missing:
            raise RuntimeError(f"PyBullet did not create joints {missing}; URDF and physics model disagree")
        self.index_to_link = {v: k for k, v in self.link_index.items()}
        self.child_link_indices = [i for i in range(n)]
        self.movable_joints = [j.name for j in model.movable_joints]
        self.movable_indices = [self.joint_index[j] for j in self.movable_joints]
        self.index_of_movable = {name: k for k, name in enumerate(self.movable_joints)}

        dyn = p.getDynamicsInfo(self.body, -1, physicsClientId=self.client)
        self._T_com_link = invert_transform(pose_to_matrix(dyn[3], dyn[4]))  # base COM -> base link frame
        self._T_link_com = pose_to_matrix(dyn[3], dyn[4])

        self._configure_passive_joints()
        if lateral_friction is not None:
            self.set_friction(lateral_friction)
        self._joint_cache_step = -1
        self._link_cache_step = -1
        self._joint_cache: Optional[JointStateArrays] = None

    def _configure_passive_joints(self) -> None:
        """Stop the default velocity motors of PyBullet.

        A joint without a motor is passive. A velocity motor with target 0 gives
        the URDF <dynamics friction>. The friction value limits this motor.
        """
        for name in self.movable_joints:
            joint = self.model.joints[name]
            p.setJointMotorControl2(self.body, self.joint_index[name], p.VELOCITY_CONTROL,
                                    targetVelocity=0.0, force=float(joint.dynamics.friction),
                                    physicsClientId=self.client)
            p.changeDynamics(self.body, self.joint_index[name], jointDamping=float(joint.dynamics.damping),
                             physicsClientId=self.client)

    def release_joint(self, name: str) -> None:
        """Make a joint passive again. Use this after you remove its motor."""
        self._configure_passive_joint(name)

    def _configure_passive_joint(self, name: str) -> None:
        joint = self.model.joints[name]
        p.setJointMotorControl2(self.body, self.joint_index[name], p.VELOCITY_CONTROL,
                                targetVelocity=0.0, force=float(joint.dynamics.friction),
                                physicsClientId=self.client)

    def set_friction(self, mu: float) -> None:
        for idx in [-1] + self.child_link_indices:
            p.changeDynamics(self.body, idx, lateralFriction=float(mu), physicsClientId=self.client)

    def set_link_friction(self, link: str, mu: float) -> None:
        p.changeDynamics(self.body, self.link_index[link], lateralFriction=float(mu), physicsClientId=self.client)

    def joint_states(self) -> JointStateArrays:
        step = self.engine.step_count
        if self._joint_cache is not None and self._joint_cache_step == step:
            return self._joint_cache
        if self.movable_indices:
            states = p.getJointStates(self.body, self.movable_indices, physicsClientId=self.client)
            pos = np.fromiter((s[0] for s in states), float, len(states))
            vel = np.fromiter((s[1] for s in states), float, len(states))
            eff = np.fromiter((s[3] for s in states), float, len(states))
        else:
            pos = vel = eff = np.zeros(0)
        self._joint_cache = JointStateArrays(self.movable_joints, pos, vel, eff)
        self._joint_cache_step = step
        return self._joint_cache

    def update_link_transforms(self, force: bool = False) -> dict[str, np.ndarray]:
        """Update `model.world_transforms` (URDF link frames, not center-of-mass frames)."""
        step = self.engine.step_count
        if not force and self._link_cache_step == step:
            return self.model.world_transforms
        wt = self.model.world_transforms
        pos, orn = p.getBasePositionAndOrientation(self.body, physicsClientId=self.client)
        wt[self.model.root_link] = pose_to_matrix(pos, orn) @ self._T_com_link
        if self.child_link_indices:
            states = p.getLinkStates(self.body, self.child_link_indices, computeForwardKinematics=1,
                                     physicsClientId=self.client)
            for idx, s in zip(self.child_link_indices, states):
                wt[self.index_to_link[idx]] = pose_to_matrix(s[4], s[5])
        self._link_cache_step = step
        return wt

    def invalidate_cache(self) -> None:
        self._joint_cache_step = -1
        self._link_cache_step = -1

    def link_transform(self, link: str) -> np.ndarray:
        return self.update_link_transforms()[link]

    def link_velocity(self, link: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return the center-of-mass position, its linear velocity and the angular velocity, in the world frame."""
        idx = self.link_index[link]
        if idx == -1:
            pos, _ = p.getBasePositionAndOrientation(self.body, physicsClientId=self.client)
            v, w = p.getBaseVelocity(self.body, physicsClientId=self.client)
            return np.asarray(pos), np.asarray(v), np.asarray(w)
        s = p.getLinkState(self.body, idx, computeLinkVelocity=1, computeForwardKinematics=1,
                           physicsClientId=self.client)
        return np.asarray(s[0]), np.asarray(s[6]), np.asarray(s[7])

    def base_pose(self) -> tuple[tuple, tuple]:
        return matrix_to_pose(self.link_transform(self.model.root_link))

    def reset_state(self, base_position: Sequence[float], base_orientation: Sequence[float],
                    joint_positions: dict[str, float]) -> None:
        T_link = pose_to_matrix(base_position, base_orientation)
        com_pos, com_orn = matrix_to_pose(T_link @ self._T_link_com)
        p.resetBasePositionAndOrientation(self.body, com_pos, com_orn, physicsClientId=self.client)
        p.resetBaseVelocity(self.body, [0, 0, 0], [0, 0, 0], physicsClientId=self.client)
        for name, idx in zip(self.movable_joints, self.movable_indices):
            p.resetJointState(self.body, idx, float(joint_positions.get(name, 0.0)), 0.0,
                              physicsClientId=self.client)
        self.invalidate_cache()

    def remove(self) -> None:
        self.engine.remove_body(self.body)
