"""Models of ros2_control controllers, without ROS.

The simulator has no ros2_control. It publishes joint states and accepts joint
velocity commands. These classes give the behavior of two standard controllers.
They read the parameters from the controllers.yaml file of your project.

  * diff_drive_controller: velocity and acceleration limits, command time-out,
    wheel velocities and odometry from the wheel encoders.
  * joint_trajectory_controller (velocity interface): trajectory sample,
    velocity feed-forward with P feedback, hold of the last point.

This module does not import ROS. Thus, you can test it without a ROS graph.
"""

from dataclasses import dataclass, field
import math


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


@dataclass
class AxisLimits:
    """Limits of one axis (linear.x or angular.z), as in diff_drive_controller."""

    min_velocity: float = -math.inf
    max_velocity: float = math.inf
    max_acceleration: float = math.inf     # increase of |v|
    max_deceleration: float = math.inf     # decrease of |v| (positive value)

    @classmethod
    def from_params(cls, params, prefix):
        def get(key, default):
            return float(params.get(f'{prefix}.{key}', default))
        dec = get('max_deceleration', -math.inf)
        return cls(min_velocity=get('min_velocity', -math.inf),
                   max_velocity=get('max_velocity', math.inf),
                   max_acceleration=abs(get('max_acceleration', math.inf)),
                   max_deceleration=abs(dec) if math.isfinite(dec) else
                   abs(get('max_acceleration', math.inf)))

    def limit(self, target, previous, dt):
        v = _clamp(target, self.min_velocity, self.max_velocity)
        if dt <= 0:
            return previous
        speeding_up = abs(v) > abs(previous) and v * previous >= 0
        a = self.max_acceleration if speeding_up else self.max_deceleration
        if math.isfinite(a):
            v = _clamp(v, previous - a * dt, previous + a * dt)
        return v


@dataclass
class DiffDriveModel:
    """Command part and odometry part of diff_drive_controller."""

    wheel_separation: float
    wheel_radius: float
    linear: AxisLimits = field(default_factory=AxisLimits)
    angular: AxisLimits = field(default_factory=AxisLimits)
    cmd_timeout: float = 0.5
    v: float = 0.0
    w: float = 0.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    wz: float = 0.0
    _left: float | None = None
    _right: float | None = None

    @classmethod
    def from_params(cls, params):
        """Make the model from the flat parameters of a diff_drive_controller."""
        return cls(wheel_separation=float(params['wheel_separation']),
                   wheel_radius=float(params['wheel_radius']),
                   linear=AxisLimits.from_params(params, 'linear.x'),
                   angular=AxisLimits.from_params(params, 'angular.z'),
                   cmd_timeout=float(params.get('cmd_vel_timeout', 0.5)))

    def update_command(self, cmd_v, cmd_w, cmd_age, dt):
        """Return the (left, right) wheel velocities [rad/s] for this control period."""
        if cmd_age is None or cmd_age > self.cmd_timeout:
            cmd_v, cmd_w = 0.0, 0.0
        self.v = self.linear.limit(cmd_v, self.v, dt)
        self.w = self.angular.limit(cmd_w, self.w, dt)
        half = self.wheel_separation / 2.0
        return ((self.v - self.w * half) / self.wheel_radius,
                (self.v + self.w * half) / self.wheel_radius)

    def update_odometry(self, left_pos, right_pos, dt):
        """Add the motion of the wheel encoder positions [rad] (second-order Runge-Kutta)."""
        if self._left is None:
            self._left, self._right = left_pos, right_pos
            return
        dl = (left_pos - self._left) * self.wheel_radius
        dr = (right_pos - self._right) * self.wheel_radius
        self._left, self._right = left_pos, right_pos
        ds = (dl + dr) / 2.0
        dyaw = (dr - dl) / self.wheel_separation
        mid = self.yaw + dyaw / 2.0
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))
        if dt > 0:
            self.vx = ds / dt
            self.wz = dyaw / dt


@dataclass
class TrajectoryFollower:
    """joint_trajectory_controller for one joint on the velocity command interface."""

    p_gain: float = 1.0
    ff_velocity_scale: float = 1.0
    max_velocity: float = math.inf
    points: list = field(default_factory=list)   # [(time from start, position, velocity or None)]
    start_time: float | None = None
    hold_position: float | None = None

    def set_trajectory(self, points, start_time, current_pos):
        """Start a new trajectory. The current state is the first point (t = 0)."""
        if not points:
            self.points, self.start_time, self.hold_position = [], None, current_pos
            return
        self.points = [(0.0, current_pos, 0.0)] + [p for p in points if p[0] > 0.0]
        self.start_time = start_time

    def sample(self, now):
        """Return (position, velocity) of the reference, or None if there is no reference."""
        if not self.points:
            if self.hold_position is None:
                return None
            return self.hold_position, 0.0
        t = now - self.start_time
        if t <= 0.0:
            return self.points[0][1], 0.0
        for (t0, p0, v0), (t1, p1, v1) in zip(self.points, self.points[1:]):
            if t <= t1:
                return _hermite(t - t0, t1 - t0, p0, v0, p1, v1)
        last = self.points[-1]
        self.hold_position = last[1]
        return last[1], 0.0

    def command(self, now, position):
        """Return the velocity command for the measured `position`."""
        if self.hold_position is None and not self.points:
            # An active joint_trajectory_controller holds the current position.
            self.hold_position = position
        ref = self.sample(now)
        if ref is None:
            return 0.0
        pos_d, vel_d = ref
        u = self.ff_velocity_scale * vel_d + self.p_gain * (pos_d - position)
        return _clamp(u, -self.max_velocity, self.max_velocity)


def _hermite(t, T, p0, v0, p1, v1):
    """Cubic Hermite interpolation. If a velocity is missing, use linear interpolation."""
    if T <= 0:
        return p1, (v1 or 0.0)
    if v0 is None or v1 is None:
        s = t / T
        return p0 + (p1 - p0) * s, (p1 - p0) / T
    s = t / T
    h00, h10 = 2 * s**3 - 3 * s**2 + 1, s**3 - 2 * s**2 + s
    h01, h11 = -2 * s**3 + 3 * s**2, s**3 - s**2
    pos = h00 * p0 + h10 * T * v0 + h01 * p1 + h11 * T * v1
    d00, d10 = (6 * s**2 - 6 * s) / T, 3 * s**2 - 4 * s + 1
    d01, d11 = (-6 * s**2 + 6 * s) / T, 3 * s**2 - 2 * s
    vel = d00 * p0 + d10 * v0 + d01 * p1 + d11 * v1
    return pos, vel


def controller_sections(data):
    """
    Return {controller name: ros__parameters} from a parsed controllers.yaml.

    Some launch files put the file in a namespace ({ns: {...}}). This function
    accepts the two layouts.
    """
    if 'controller_manager' not in data and len(data) == 1:
        data = next(iter(data.values()))
    return {name: (value or {}).get('ros__parameters', {})
            for name, value in data.items() if isinstance(value, dict)}


def flatten(params, prefix=''):
    """Change nested parameter dictionaries into ROS keys with dots (linear.x.max_velocity)."""
    out = {}
    for k, v in params.items():
        key = f'{prefix}{k}'
        if isinstance(v, dict):
            out.update(flatten(v, key + '.'))
        else:
            out[key] = v
    return out
