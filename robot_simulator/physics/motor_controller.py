"""Applies the motor commands of one robot to PyBullet at each physics step.

This module also does URDF <mimic> joints. A mimic joint follows
`multiplier * q_leader + offset`. A stiff position servo moves it. The effort
limit of the mimic joint limits this servo. Mimic joints are constraints. The
user cannot assign motors to them.
"""
from __future__ import annotations

import logging
from typing import Optional

import pybullet as p

from ..motors.linear_motor import LinearMotor
from ..motors.motor_base import Motor, MotorConfig
from ..motors.rotational_motor import RotationalMotor
from .robot_physics import RobotPhysics

log = logging.getLogger(__name__)

MOTOR_CLASSES = {"rotational": RotationalMotor, "linear": LinearMotor}


def motor_type_for_joint(joint_type: str) -> Optional[str]:
    if joint_type in RotationalMotor.APPLICABLE_JOINT_TYPES:
        return "rotational"
    if joint_type in LinearMotor.APPLICABLE_JOINT_TYPES:
        return "linear"
    return None


class MotorError(ValueError):
    pass


class MotorController:
    def __init__(self, robot: RobotPhysics):
        self.robot = robot
        self.motors: dict[str, Motor] = {}
        self.mimics = [j for j in robot.model.joints.values() if j.mimic is not None and j.is_movable]

    def assign(self, config: MotorConfig) -> Motor:
        joint = self.robot.model.joints.get(config.joint)
        if joint is None:
            raise MotorError(f"Joint '{config.joint}' does not exist")
        if joint.type == "fixed":
            raise MotorError(f"Joint '{joint.name}' is fixed; fixed joints never receive motors")
        if joint.mimic is not None:
            raise MotorError(f"Joint '{joint.name}' mimics '{joint.mimic.joint}' and is driven by it")
        expected = motor_type_for_joint(joint.type)
        if config.type != expected:
            raise MotorError(f"A {config.type} motor cannot drive {joint.type} joint '{joint.name}' "
                             f"(use a {expected} motor)")
        lim = joint.limit
        motor = MOTOR_CLASSES[config.type](config, self.robot.joint_index[joint.name],
                                           lower=lim.lower if lim else 0.0, upper=lim.upper if lim else 0.0,
                                           has_limits=joint.has_position_limits)
        self.motors[joint.name] = motor
        return motor

    def remove(self, joint_name: str) -> None:
        if self.motors.pop(joint_name, None) is not None:
            self.robot.release_joint(joint_name)

    def get(self, joint_name: str) -> Optional[Motor]:
        return self.motors.get(joint_name)

    def apply(self) -> None:
        if not self.motors and not self.mimics:
            return
        js = self.robot.joint_states()
        idx = self.robot.index_of_movable
        client, body = self.robot.client, self.robot.body
        for name, motor in self.motors.items():
            motor.apply(client, body, float(js.velocity[idx[name]]))
        for j in self.mimics:
            leader = j.mimic.joint  # type: ignore[union-attr]
            if leader not in idx:
                continue
            target = j.mimic.multiplier * float(js.position[idx[leader]]) + j.mimic.offset  # type: ignore[union-attr]
            force = j.limit.effort if j.limit and j.limit.effort > 0 else 100.0
            p.setJointMotorControl2(body, self.robot.joint_index[j.name], p.POSITION_CONTROL,
                                    targetPosition=target, positionGain=1.0, velocityGain=1.0, force=force,
                                    physicsClientId=client)

    def reset(self, initial_positions: dict[str, float]) -> None:
        for name, m in self.motors.items():
            m.reset(initial_positions.get(name, 0.0))

    def configs(self) -> list[MotorConfig]:
        return [m.config for m in self.motors.values()]

    def effort_of(self, joint_name: str, measured: float) -> float:
        m = self.motors.get(joint_name)
        if m is not None and m.mode.value == "effort":
            return m.applied_effort
        return measured
