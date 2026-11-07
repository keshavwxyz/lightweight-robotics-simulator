"""Optional rclpy import.

The simulator runs without ROS 2. This module tells if the bridge can start.
"""
from __future__ import annotations

import logging

log = logging.getLogger(__name__)

import importlib

try:
    importlib.import_module("rclpy.node")
    ROS_AVAILABLE = True
    ROS_IMPORT_ERROR = ""
except Exception as exc:  # ImportError, or missing shared libraries if ROS is not sourced
    ROS_AVAILABLE = False
    ROS_IMPORT_ERROR = str(exc)

try:
    importlib.import_module("simulation_interfaces.srv")
    SIM_INTERFACES_AVAILABLE = ROS_AVAILABLE
except Exception:
    SIM_INTERFACES_AVAILABLE = False


def ros_distro() -> str:
    import os
    return os.environ.get("ROS_DISTRO", "unknown")


def time_msg(t: float):
    from builtin_interfaces.msg import Time
    sec = int(t)
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)


def create_node(name: str, domain_id=None):
    """Make an rclpy context and a node that use no global state."""
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node

    ctx = Context()
    rclpy.init(context=ctx, domain_id=int(domain_id) if domain_id is not None else None)
    node = Node(name, context=ctx)
    return ctx, node
