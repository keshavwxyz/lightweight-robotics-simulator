"""Command subscribers.

The callbacks only add commands to the queue. They do not use the physics
engine.

/<robot>/cmd_vel         geometry_msgs/msg/Twist (or TwistStamped, see Ros2Config.cmd_vel_stamped)
                         linear.x [m/s] and angular.z [rad/s] drive the differential-drive controller.
/<robot>/joint_commands  sensor_msgs/msg/JointState
                         For each name[i], the first field that is not empty and not
                         NaN is used, in this sequence: position[i], velocity[i],
                         effort[i]. It sets the control mode of the motor of that joint.
                         Arrays can be empty.
"""
from __future__ import annotations

import logging
import math

from ..core.commands import CmdVelCommand, JointTargetsCommand

log = logging.getLogger(__name__)


def create_robot_subscribers(node, robot: str, commands, stamped_cmd_vel: bool,
                             cmd_vel_topic: str, joint_commands_topic: str) -> list:
    from sensor_msgs.msg import JointState

    subs = []
    if stamped_cmd_vel:
        from geometry_msgs.msg import TwistStamped

        def on_twist_stamped(msg: TwistStamped):
            commands.put(CmdVelCommand(robot, float(msg.twist.linear.x), float(msg.twist.angular.z)))
        subs.append(node.create_subscription(TwistStamped, cmd_vel_topic, on_twist_stamped, 10))
    else:
        from geometry_msgs.msg import Twist

        def on_twist(msg: Twist):
            commands.put(CmdVelCommand(robot, float(msg.linear.x), float(msg.angular.z)))
        subs.append(node.create_subscription(Twist, cmd_vel_topic, on_twist, 10))

    def on_joint_commands(msg: JointState):
        n = len(msg.name)
        nan = [math.nan] * n
        commands.put(JointTargetsCommand(
            robot, list(msg.name),
            list(msg.position) if len(msg.position) else nan,
            list(msg.velocity) if len(msg.velocity) else nan,
            list(msg.effort) if len(msg.effort) else nan))
    subs.append(node.create_subscription(JointState, joint_commands_topic, on_joint_commands, 10))
    return subs
