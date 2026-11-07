"""Linear motor for prismatic joints.

Units: position [m], velocity [m/s], effort [N].
"""
from __future__ import annotations

from .motor_base import Motor


class LinearMotor(Motor):
    TYPE = "linear"
    APPLICABLE_JOINT_TYPES = ("prismatic",)
    EFFORT_NAME = "force"
