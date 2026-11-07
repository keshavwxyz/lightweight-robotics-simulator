"""Rigid-body math functions. They use only numpy.

Conventions of the simulator:
  * Quaternions are (x, y, z, w). PyBullet and ROS messages use the same order.
  * Euler angles are URDF "rpy" angles about fixed axes X (roll), Y (pitch) and
    Z (yaw). Thus, R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
  * Homogeneous transforms are 4x4 numpy float64 arrays. T_a_b changes points
    from frame b into frame a.
  * All units are SI (metres, radians, seconds).
"""
from __future__ import annotations

import math
from typing import Sequence

import numpy as np

IDENTITY_QUAT = (0.0, 0.0, 0.0, 1.0)


def rpy_to_matrix(rpy: Sequence[float]) -> np.ndarray:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def matrix_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    sp = -R[2, 0]
    if abs(sp) >= 1.0 - 1e-12:  # gimbal lock
        pitch = math.copysign(math.pi / 2, sp)
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    else:
        pitch = math.asin(sp)
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def quat_to_matrix(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def matrix_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    if q[3] < 0:  # canonical form
        q = -q
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])


def rpy_to_quat(rpy: Sequence[float]) -> tuple[float, float, float, float]:
    return matrix_to_quat(rpy_to_matrix(rpy))


def quat_to_rpy(q: Sequence[float]) -> tuple[float, float, float]:
    return matrix_to_rpy(quat_to_matrix(q))


def make_transform(xyz: Sequence[float] = (0, 0, 0), rpy: Sequence[float] = (0, 0, 0)) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return T


def pose_to_matrix(pos: Sequence[float], quat: Sequence[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(quat)
    T[:3, 3] = pos
    return T


def matrix_to_pose(T: np.ndarray) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    return (float(T[0, 3]), float(T[1, 3]), float(T[2, 3])), matrix_to_quat(T[:3, :3])


def invert_transform(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ T[:3, 3]
    return out


def scale_matrix(s: Sequence[float]) -> np.ndarray:
    S = np.eye(4)
    S[0, 0], S[1, 1], S[2, 2] = s
    return S



def perspective(fovy: float, aspect: float, near: float, far: float) -> np.ndarray:
    f = 1.0 / math.tan(fovy / 2.0)
    M = np.zeros((4, 4))
    M[0, 0] = f / aspect
    M[1, 1] = f
    M[2, 2] = (far + near) / (near - far)
    M[2, 3] = 2.0 * far * near / (near - far)
    M[3, 2] = -1.0
    return M


def look_at(eye: Sequence[float], target: Sequence[float], up: Sequence[float] = (0, 0, 1)) -> np.ndarray:
    e = np.asarray(eye, dtype=float)
    f = np.asarray(target, dtype=float) - e
    f /= np.linalg.norm(f)
    s = np.cross(f, up)
    if np.linalg.norm(s) < 1e-9:
        s = np.cross(f, (0, 1, 0))
    s /= np.linalg.norm(s)
    u = np.cross(s, f)
    M = np.eye(4)
    M[0, :3], M[1, :3], M[2, :3] = s, u, -f
    M[:3, 3] = -M[:3, :3] @ e
    return M


# Rotation from the OpenGL camera axes (x right, y up, view along -z) to the
# ROS body axes of camera links (x forward, y left, z up).
BODY_FROM_GL = np.array([
    [0.0, 0.0, -1.0, 0.0],
    [-1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
])

# Rotation from a ROS body frame (x forward, y left, z up) to the REP-103
# optical frame (z forward, x right, y down). This is rpy = (-pi/2, 0, -pi/2).
BODY_FROM_OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)
