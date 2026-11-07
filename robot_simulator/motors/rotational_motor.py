"""Rotational motor for revolute and continuous joints.

Units: position [rad], velocity [rad/s], effort [N m].
"""
from __future__ import annotations

from .motor_base import Motor


class RotationalMotor(Motor):
    TYPE = "rotational"
    APPLICABLE_JOINT_TYPES = ("revolute", "continuous")
    EFFORT_NAME = "torque"
