"""Differential-drive base controller.

This controller changes a body velocity (v [m/s], w [rad/s]) into wheel
angular velocities. It sends these velocities to the rotational motors of the
wheel joints and sets these motors to velocity mode. It is not a motor type.

    w_left  = (v - w * L / 2) / r
    w_right = (v + w * L / 2) / r

r is the wheel radius. L is the wheel separation. A positive joint velocity
must move the robot forward. If the URDF axis has the opposite direction, set
`invert_left` or `invert_right`.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from ..physics.motor_controller import MotorController


@dataclass
class DiffDriveConfig:
    enabled: bool = False
    left_joints: list[str] = field(default_factory=list)
    right_joints: list[str] = field(default_factory=list)
    wheel_radius: float = 0.05        # m
    wheel_separation: float = 0.3     # m
    invert_left: bool = False
    invert_right: bool = False
    max_linear_velocity: float = 1.0  # m/s
    max_angular_velocity: float = 3.0 # rad/s


class DiffDriveController:
    def __init__(self, config: DiffDriveConfig, motors: MotorController):
        self.config = config
        self.motors = motors
        self.last_command_time = -1.0
        self.v = 0.0
        self.w = 0.0

    def validate(self) -> list[str]:
        errors = []
        c = self.config
        if c.wheel_radius <= 0:
            errors.append("wheel radius must be > 0")
        if c.wheel_separation <= 0:
            errors.append("wheel separation must be > 0")
        if not c.left_joints or not c.right_joints:
            errors.append("at least one left and one right wheel joint is required")
        for j in c.left_joints + c.right_joints:
            m = self.motors.get(j)
            if m is None:
                errors.append(f"wheel joint '{j}' has no rotational motor assigned")
            elif m.TYPE != "rotational":
                errors.append(f"wheel joint '{j}' must use a rotational motor")
        return errors

    def command(self, v: float, w: float, sim_time: float) -> None:
        c = self.config
        self.v = max(-c.max_linear_velocity, min(c.max_linear_velocity, v))
        self.w = max(-c.max_angular_velocity, min(c.max_angular_velocity, w))
        self.last_command_time = sim_time
        self._send()

    def stop(self) -> None:
        self.v = self.w = 0.0
        self._send()

    def check_timeout(self, sim_time: float, timeout: float) -> None:
        if self.last_command_time >= 0 and timeout > 0 and sim_time - self.last_command_time > timeout:
            self.last_command_time = -1.0
            self.stop()

    def _send(self) -> None:
        c = self.config
        half = c.wheel_separation / 2.0
        wl = (self.v - self.w * half) / c.wheel_radius
        wr = (self.v + self.w * half) / c.wheel_radius
        if c.invert_left:
            wl = -wl
        if c.invert_right:
            wr = -wr
        for j in c.left_joints:
            m = self.motors.get(j)
            if m is not None:
                m.set_velocity(wl)
        for j in c.right_joints:
            m = self.motors.get(j)
            if m is not None:
                m.set_velocity(wr)
