"""Motor logic for the two motor types.

There are only two motor types: rotational_motor.py and linear_motor.py.
They are ideal actuators. They have no electrical model. Each motor is a
position, velocity or effort controller with limits. It operates directly on
a PyBullet joint.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import pybullet as p


class ControlMode(str, Enum):
    POSITION = "position"
    VELOCITY = "velocity"
    EFFORT = "effort"


@dataclass
class MotorConfig:
    joint: str
    type: str                      # "rotational" | "linear"
    mode: str = ControlMode.POSITION.value
    max_effort: float = 10.0       # N m (rotational) or N (linear)
    max_velocity: float = 1.0      # rad/s or m/s
    position_gain: float = 0.3     # PyBullet normalised position gain (Kp)
    velocity_gain: float = 1.0     # PyBullet normalised velocity gain (Kd)
    target_position: float = 0.0   # rad or m
    target_velocity: float = 0.0   # rad/s or m/s
    target_effort: float = 0.0     # N m or N


class Motor:
    """Base class. Each subclass sets the unit names and the applicable joint types."""

    TYPE = ""
    APPLICABLE_JOINT_TYPES: tuple[str, ...] = ()
    EFFORT_NAME = "effort"

    def __init__(self, config: MotorConfig, joint_index: int,
                 lower: float = 0.0, upper: float = 0.0, has_limits: bool = False):
        self.config = config
        self.joint_index = joint_index
        self.lower, self.upper, self.has_limits = lower, upper, has_limits
        self._dirty = True        # the command must go to PyBullet again
        self.applied_effort = 0.0

    @property
    def mode(self) -> ControlMode:
        return ControlMode(self.config.mode)

    def set_mode(self, mode: ControlMode | str) -> None:
        mode = ControlMode(mode)
        if mode.value != self.config.mode:
            self.config.mode = mode.value
            self._dirty = True

    def set_position(self, value: float) -> None:
        if self.has_limits:
            value = min(max(value, self.lower), self.upper)
        self.config.target_position = float(value)
        self.set_mode(ControlMode.POSITION)
        self._dirty = True

    def set_velocity(self, value: float) -> None:
        self.config.target_velocity = float(value)
        self.set_mode(ControlMode.VELOCITY)
        self._dirty = True

    def set_effort(self, value: float) -> None:
        self.config.target_effort = float(value)
        self.set_mode(ControlMode.EFFORT)
        self._dirty = True

    def mark_dirty(self) -> None:
        self._dirty = True

    def apply(self, client: int, body: int, current_velocity: float) -> None:
        """Send the command to PyBullet.

        PyBullet keeps position and velocity commands. Thus, the motor sends them
        again only after a change. The motor must send effort commands at each
        physics step.
        """
        c = self.config
        mode = self.mode
        j = self.joint_index
        max_eff = abs(c.max_effort)
        max_vel = abs(c.max_velocity)
        if mode is ControlMode.EFFORT:
            if self._dirty:
                # Stop the built-in velocity motor of PyBullet. Torque control then has full effect.
                p.setJointMotorControl2(body, j, p.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0,
                                        physicsClientId=client)
                self._dirty = False
            eff = min(max(c.target_effort, -max_eff), max_eff)
            # Ideal velocity limit: at the velocity limit, apply no more effort in that direction.
            if max_vel > 0 and abs(current_velocity) >= max_vel and math.copysign(1, eff) == math.copysign(1, current_velocity):
                eff = 0.0
            self.applied_effort = eff
            p.setJointMotorControl2(body, j, p.TORQUE_CONTROL, force=eff, physicsClientId=client)
            return
        if not self._dirty:
            return
        if mode is ControlMode.POSITION:
            target = c.target_position
            if self.has_limits:
                target = min(max(target, self.lower), self.upper)
            kwargs = dict(targetPosition=target, targetVelocity=0.0, positionGain=c.position_gain,
                          velocityGain=c.velocity_gain, force=max_eff)
            if max_vel > 0:
                kwargs["maxVelocity"] = max_vel
            p.setJointMotorControl2(body, j, p.POSITION_CONTROL, physicsClientId=client, **kwargs)
        else:  # VELOCITY
            v = c.target_velocity
            if max_vel > 0:
                v = min(max(v, -max_vel), max_vel)
            p.setJointMotorControl2(body, j, p.VELOCITY_CONTROL, targetVelocity=v,
                                    velocityGain=c.velocity_gain, force=max_eff, physicsClientId=client)
        self._dirty = False

    def reset(self, initial_position: float = 0.0) -> None:
        self.config.target_position = initial_position
        self.config.target_velocity = 0.0
        self.config.target_effort = 0.0
        self._dirty = True
