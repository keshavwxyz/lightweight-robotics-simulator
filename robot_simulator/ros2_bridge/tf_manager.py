"""TF tree (REP-105). Each frame has only one parent.

For each robot (the frame names start with the `tf_prefix` of the robot):

  odometry enabled:
      [world -> odom]  (/tf_static, only if `publish_world_tf`)
      odom  -> <root link>                      (/tf)
  odometry disabled:
      world -> <root link]                      (/tf_static if fixed base, else /tf)
  every URDF joint parent -> child:
      fixed joints                              (/tf_static, URDF origin)
      revolute / continuous / prismatic         (/tf, from simulated link poses)
  sensors:
      <link> -> <sensor>_link                   (/tf_static, mount offset)
      <camera>_link -> <camera>_optical_frame   (/tf_static, REP-103 optical rotation)

Each frame has exactly one parent. Thus, the tree is valid.
"""
from __future__ import annotations

import numpy as np

from ..core.snapshots import RobotDescriptor, RobotStateSnapshot
from ..utils.transforms import BODY_FROM_OPTICAL_RPY, invert_transform, make_transform, matrix_to_pose

WORLD_FRAME = "world"
_OPTICAL = make_transform((0, 0, 0), BODY_FROM_OPTICAL_RPY)


class TFManager:
    def __init__(self, world_frame: str = WORLD_FRAME):
        self.world_frame = world_frame
        self.static: dict[str, dict[str, tuple[str, np.ndarray]]] = {}  # robot -> child -> (parent, T)
        self.descriptors: dict[str, RobotDescriptor] = {}

    def set_robot(self, d: RobotDescriptor) -> None:
        self.descriptors[d.name] = d
        p = d.tf_prefix
        st: dict[str, tuple[str, np.ndarray]] = {}
        if not d.publish_tf:                 # other nodes (robot_state_publisher, a localizer) publish the TF
            self.static[d.name] = st
            return
        for j in d.joints:
            if j.type == "fixed":
                st[p + j.child] = (p + j.parent, j.origin)
        for s in d.sensors:
            if s.frame_override:             # existing frame (URDF link): another node publishes it
                continue
            st[s.frame_id] = (p + s.link, s.mount)
            if s.optical_frame_id:
                st[s.optical_frame_id] = (s.frame_id, _OPTICAL)
        if d.odom_enabled:
            if d.odom_publish_world_tf:
                st[p + d.odom_frame] = (self.world_frame, d.odom_origin)
        elif d.fixed_base:
            pass  # the first snapshot adds world -> root (the pose is then known)
        self.static[d.name] = st

    def remove_robot(self, name: str) -> None:
        self.static.pop(name, None)
        self.descriptors.pop(name, None)

    def fixed_base_root(self, snap: RobotStateSnapshot) -> bool:
        """Add the static transform world -> root for a robot with a fixed base.

        Return True if the static transforms changed.
        """
        d = self.descriptors.get(snap.robot)
        if d is None or d.odom_enabled or not d.fixed_base or not d.publish_tf:
            return False
        st = self.static[d.name]
        key = d.tf_prefix + d.root_link
        T = snap.link_transforms[d.root_link]
        prev = st.get(key)
        if prev is not None and np.allclose(prev[1], T, atol=1e-6):
            return False
        st[key] = (self.world_frame, T)
        return True

    def all_static(self) -> list[tuple[str, str, np.ndarray]]:
        out = []
        for st in self.static.values():
            for child, (parent, T) in st.items():
                out.append((parent, child, T))
        return out

    def dynamic(self, snap: RobotStateSnapshot) -> list[tuple[str, str, np.ndarray]]:
        d = self.descriptors.get(snap.robot)
        if d is None or not d.publish_tf:
            return []
        p = d.tf_prefix
        wt = snap.link_transforms
        out = []
        if d.odom_enabled:
            out.append((p + d.odom_frame, p + d.root_link, invert_transform(d.odom_origin) @ wt[d.root_link]))
        elif not d.fixed_base:
            out.append((self.world_frame, p + d.root_link, wt[d.root_link]))
        for j in d.joints:
            if j.type != "fixed":
                out.append((p + j.parent, p + j.child, invert_transform(wt[j.parent]) @ wt[j.child]))
        return out


def to_transform_stamped(parent: str, child: str, T: np.ndarray, stamp):
    from geometry_msgs.msg import TransformStamped
    pos, q = matrix_to_pose(T)
    m = TransformStamped()
    m.header.stamp = stamp
    m.header.frame_id = parent
    m.child_frame_id = child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = pos
    m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
    return m
