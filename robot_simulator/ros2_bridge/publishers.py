"""Change simulator data into standard ROS 2 messages.

Only the thread of the ROS 2 bridge uses this module.
"""
from __future__ import annotations

import array
import math

import numpy as np

from ..core.snapshots import RobotDescriptor, RobotStateSnapshot
from ..utils.transforms import invert_transform, matrix_to_pose
from .ros_node import time_msg

DEFAULT_TOPICS = {"imu": "imu/data", "lidar_2d": "scan", "camera": "camera/image_raw"}


def resolve_topic(desc: RobotDescriptor, name: str) -> str:
    """Return the full topic name.

    An absolute name ("/x") does not change. A relative name goes into the
    robot namespace ("" = root namespace, default "/<robot name>").
    """
    if name.startswith("/"):
        return name
    return f"{desc.namespace}/{name}"


def sensor_topic(desc: RobotDescriptor, sensor_name: str) -> str:
    """Return the topic of a sensor. Relative names are in the robot namespace."""
    s = next(s for s in desc.sensors if s.name == sensor_name)
    if s.topic:
        return resolve_topic(desc, s.topic)
    same_type = [o for o in desc.sensors if o.type == s.type]
    if len(same_type) == 1 or same_type[0].name == s.name:
        return resolve_topic(desc, DEFAULT_TOPICS[s.type])
    suffix = {"imu": "data", "lidar_2d": "scan", "camera": "image_raw"}[s.type]
    return resolve_topic(desc, f"{s.name}/{suffix}")


def camera_info_topic(image_topic: str) -> str:
    base = image_topic.rsplit("/", 1)[0]
    return f"{base}/camera_info"


def joint_state_msg(snap: RobotStateSnapshot):
    from sensor_msgs.msg import JointState
    m = JointState()
    m.header.stamp = time_msg(snap.stamp)
    m.name = list(snap.joint_names)
    m.position = snap.position.tolist()
    m.velocity = snap.velocity.tolist()
    m.effort = snap.effort.tolist()
    return m


def odometry_msg(desc: RobotDescriptor, snap: RobotStateSnapshot):
    """Return the ground-truth odometry of `base_frame` in the odom frame.

    The twist is in the child (base) frame, as nav_msgs/Odometry specifies.
    """
    from nav_msgs.msg import Odometry
    p = desc.tf_prefix
    T_wb = snap.link_transforms[desc.base_frame]
    T_ob = invert_transform(desc.odom_origin) @ T_wb
    pos, q = matrix_to_pose(T_ob)
    R_wb = T_wb[:3, :3]
    v = R_wb.T @ snap.base_linear_velocity
    w = R_wb.T @ snap.base_angular_velocity
    m = Odometry()
    m.header.stamp = time_msg(snap.stamp)
    m.header.frame_id = p + desc.odom_frame
    m.child_frame_id = desc.odom_child_frame or (p + desc.base_frame)
    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = pos
    o = m.pose.pose.orientation
    o.x, o.y, o.z, o.w = q
    m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = (float(x) for x in v)
    m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z = (float(x) for x in w)
    # Perfect odometry: a small diagonal covariance. Subscribers then do not read it as unknown.
    cov = [0.0] * 36
    for i in range(6):
        cov[i * 7] = 1e-6
    m.pose.covariance = cov
    m.twist.covariance = cov
    return m


def imu_msg(data):
    from sensor_msgs.msg import Imu
    m = Imu()
    m.header.stamp = time_msg(data.stamp)
    m.header.frame_id = data.frame_id
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = data.orientation
    av, la = data.angular_velocity, data.linear_acceleration
    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = float(av[0]), float(av[1]), float(av[2])
    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = float(la[0]), float(la[1]), float(la[2])
    # Ideal sensor: covariance 0 means "known exactly" (REP-145). -1 would mean "not available".
    return m


def laser_scan_msg(data):
    from sensor_msgs.msg import LaserScan
    m = LaserScan()
    m.header.stamp = time_msg(data.stamp)
    m.header.frame_id = data.frame_id
    m.angle_min = float(data.angle_min)
    m.angle_max = float(data.angle_max)
    m.angle_increment = float(data.angle_increment)
    m.time_increment = float(data.time_increment)
    m.scan_time = float(data.scan_time)
    m.range_min = float(data.range_min)
    m.range_max = float(data.range_max)
    m.ranges = array.array("f", np.asarray(data.ranges, dtype=np.float32).tobytes())
    return m


def image_msg(img):
    from sensor_msgs.msg import Image
    m = Image()
    m.header.stamp = time_msg(img.stamp)
    m.header.frame_id = img.frame_id
    m.height, m.width = img.height, img.width
    m.encoding = "rgb8"
    m.is_bigendian = 0
    m.step = img.width * 3
    m.data = array.array("B", img.data.tobytes())
    return m


def camera_info_msg(img):
    from sensor_msgs.msg import CameraInfo
    m = CameraInfo()
    m.header.stamp = time_msg(img.stamp)
    m.header.frame_id = img.frame_id
    m.height, m.width = img.height, img.width
    m.distortion_model = "plumb_bob"
    m.d = [0.0] * 5
    m.k = list(img.K)
    m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    m.p = list(img.P)
    return m


def clock_msg(t: float):
    from rosgraph_msgs.msg import Clock
    return Clock(clock=time_msg(t))


def finite_or_inf(v: float) -> float:
    return v if math.isfinite(v) else float("inf")
