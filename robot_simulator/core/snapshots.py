"""Immutable data that the simulation loop sends to other threads (ROS 2).

Snapshots are copies. The receiver can keep a snapshot for an unlimited time.
The simulation continues to run.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class JointDescriptor:
    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray                 # 4x4 parent -> joint frame (URDF origin)


@dataclass(frozen=True)
class SensorDescriptor:
    name: str
    type: str
    link: str
    frame_id: str
    mount: np.ndarray                  # 4x4 link -> sensor frame
    optical_frame_id: Optional[str] = None
    topic: str = ""
    frame_override: bool = False       # another node publishes this frame; the simulator sends no TF


@dataclass(frozen=True)
class RobotDescriptor:
    """Structure of a robot, as the ROS 2 bridge uses it."""
    name: str
    version: int
    tf_prefix: str
    root_link: str
    links: tuple[str, ...]
    joints: tuple[JointDescriptor, ...]
    movable_joints: tuple[str, ...]
    sensors: tuple[SensorDescriptor, ...]
    fixed_base: bool
    odom_enabled: bool
    odom_frame: str
    base_frame: str
    odom_publish_world_tf: bool
    odom_origin: np.ndarray            # world -> odom
    has_drive: bool
    ros_enabled: bool = True
    namespace: str = ""                # resolved: "" (root) or "/name"
    publish_tf: bool = True
    joint_states_topic: str = "joint_states"
    cmd_vel_topic: str = "cmd_vel"
    joint_commands_topic: str = "joint_commands"
    odom_topic: str = "odom"
    odom_child_frame: str = ""


@dataclass
class RobotStateSnapshot:
    robot: str
    stamp: float
    joint_names: list[str]
    position: np.ndarray
    velocity: np.ndarray
    effort: np.ndarray
    link_transforms: dict[str, np.ndarray]           # world transforms (copies)
    base_linear_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))   # world frame
    base_angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # world frame
