"""Robot hierarchy from a parsed URDF.

`RobotModel` is the only source of the robot structure: links, joints and
parent/child relations. The physics layer writes the world transform of each
link into `world_transforms`. The renderer, the sensors and the ROS 2 bridge
read these transforms.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterator, Optional

import numpy as np

from .joint import Joint
from .link import Link
from .urdf_parser import URDFDescription


def _axis_angle_matrix(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


@dataclass
class TreeNode:
    link: str
    parent_joint: Optional[str]
    children: list["TreeNode"] = field(default_factory=list)


class RobotModel:
    def __init__(self, desc: URDFDescription):
        self.name: str = desc.name
        self.description = desc
        self.links: dict[str, Link] = desc.links
        self.joints: dict[str, Joint] = desc.joints
        self.source_path = desc.source_path
        self.warnings = list(desc.warnings)

        self.parent_joint_of: dict[str, Joint] = {j.child: j for j in self.joints.values()}
        self.child_joints_of: dict[str, list[Joint]] = {n: [] for n in self.links}
        for j in self.joints.values():
            self.child_joints_of[j.parent].append(j)
        roots = [n for n in self.links if n not in self.parent_joint_of]
        self.root_link: str = roots[0]
        self.world_transforms: dict[str, np.ndarray] = {n: np.eye(4) for n in self.links}

    @property
    def movable_joints(self) -> list[Joint]:
        return [j for j in self.iter_joints_depth_first() if j.is_movable]

    def iter_joints_depth_first(self) -> Iterator[Joint]:
        stack = list(reversed(self.child_joints_of[self.root_link]))
        while stack:
            j = stack.pop()
            yield j
            stack.extend(reversed(self.child_joints_of[j.child]))

    def tree(self) -> TreeNode:
        def build(link: str, pj: Optional[str]) -> TreeNode:
            node = TreeNode(link, pj)
            for j in self.child_joints_of[link]:
                node.children.append(build(j.child, j.name))
            return node
        return build(self.root_link, None)

    def chain_to_root(self, link: str) -> list[Joint]:
        chain = []
        while link in self.parent_joint_of:
            j = self.parent_joint_of[link]
            chain.append(j)
            link = j.parent
        return chain

    def format_tree(self) -> str:
        lines = [f"Robot '{self.name}'"]

        def rec(node: TreeNode, prefix: str):
            kids = node.children
            for i, k in enumerate(kids):
                last = i == len(kids) - 1
                j = self.joints[k.parent_joint] if k.parent_joint else None
                jt = f"  [{j.name}: {j.type}]" if j else ""
                lines.append(f"{prefix}{'└── ' if last else '├── '}{k.link}{jt}")
                rec(k, prefix + ("    " if last else "│   "))
        lines.append(f"└── {self.root_link} (root)")
        rec(self.tree(), "    ")
        return "\n".join(lines)

    @staticmethod
    def joint_motion_transform(joint: Joint, q: float) -> np.ndarray:
        T = np.eye(4)
        if joint.is_rotational:
            T[:3, :3] = _axis_angle_matrix(joint.axis, q)
        elif joint.is_linear:
            T[:3, 3] = np.asarray(joint.axis) * q
        return T

    def forward_kinematics(self, joint_positions: dict[str, float],
                           root_transform: Optional[np.ndarray] = None) -> dict[str, np.ndarray]:
        """Calculate the URDF forward kinematics without the physics engine."""
        out = {self.root_link: np.eye(4) if root_transform is None else root_transform}
        for j in self.iter_joints_depth_first():
            out[j.child] = out[j.parent] @ j.origin.matrix() @ self.joint_motion_transform(
                j, joint_positions.get(j.name, 0.0))
        return out

    def relative_transform(self, parent: str, child: str) -> np.ndarray:
        """Return the transform of `child` in the frame of `parent` (from the world transforms)."""
        Tp = self.world_transforms[parent]
        Tc = self.world_transforms[child]
        R = Tp[:3, :3]
        inv = np.eye(4)
        inv[:3, :3] = R.T
        inv[:3, 3] = -R.T @ Tp[:3, 3]
        return inv @ Tc

    def total_mass(self) -> float:
        return sum(l.inertial.mass for l in self.links.values() if l.inertial)
