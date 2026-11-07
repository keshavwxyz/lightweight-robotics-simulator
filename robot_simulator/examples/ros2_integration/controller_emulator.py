"""
ros2_control controller emulator for the Lightweight Robotics Simulator.

Use this node if your project uses ros2_control controllers in Gazebo. The node
gives the same ROS interfaces as the controllers. It reads the same
controllers.yaml file.

  joint_state_broadcaster      not necessary: the simulator publishes `joint_states`
  diff_drive_controller        subscribes `<name>/cmd_vel` (TwistStamped or Twist),
                               publishes `<name>/odom` (odometry from the wheel encoders)
  joint_trajectory_controller  subscribes `<name>/joint_trajectory`

The node sends the joint velocities to the simulator on `joint_commands_topic`
(sensor_msgs/JointState, velocity field), at the controller_manager update rate.

Parameters:
  controllers_file       path of controllers.yaml (required)
  base_controller        name of the diff_drive_controller ('' = none)
  trajectory_controllers names of joint_trajectory_controllers (velocity interface)
  joint_commands_topic   command topic of the simulator (default 'sim/joint_commands')
  stamped_cmd_vel        true = TwistStamped (default), false = Twist

Example:
  python3 controller_emulator.py --ros-args -p use_sim_time:=true \\
      -p controllers_file:=/path/controllers.yaml -p base_controller:=diff_drive_base_controller
"""

import math
import os
import sys

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from controller_logic import (controller_sections, DiffDriveModel, flatten,  # noqa: E402
                              TrajectoryFollower)


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ControllerEmulator(Node):

    def __init__(self):
        super().__init__('controller_emulator')
        path = self.declare_parameter('controllers_file', '').value
        base_name = self.declare_parameter('base_controller', '').value
        traj_names = [n for n in self.declare_parameter('trajectory_controllers', ['']).value if n]
        cmd_topic = self.declare_parameter('joint_commands_topic', 'sim/joint_commands').value
        stamped = self.declare_parameter('stamped_cmd_vel', True).value
        if not path:
            raise RuntimeError('parameter controllers_file is necessary')
        with open(path, encoding='utf-8') as f:
            sections = controller_sections(yaml.safe_load(f))
        self.rate = float(sections.get('controller_manager', {}).get('update_rate', 50))
        self.positions = {}
        self.state_time = None
        self.cmd_pub = self.create_publisher(JointState, cmd_topic, 10)
        self.create_subscription(JointState, 'joint_states', self._on_joint_states, 20)

        self.drive = None
        if base_name:
            p = flatten(sections[base_name])
            self.drive = DiffDriveModel.from_params(p)
            self.left, self.right = p['left_wheel_names'][0], p['right_wheel_names'][0]
            self.odom_frame = p.get('odom_frame_id', 'odom')
            self.base_frame = p.get('base_frame_id', 'base_link')
            self.cmd = (0.0, 0.0)
            self.cmd_time = None
            self.last_odom_time = None
            msg_type = TwistStamped if stamped else Twist
            self.create_subscription(msg_type, f'{base_name}/cmd_vel', self._on_cmd, 10)
            self.odom_pub = self.create_publisher(Odometry, f'{base_name}/odom', 10)

        self.followers = {}
        for name in traj_names:
            p = sections[name]
            gains = p.get('gains') or {}
            for joint in p['joints']:
                g = gains.get(joint, {})
                self.followers[joint] = TrajectoryFollower(
                    p_gain=float(g.get('p', 1.0)),
                    ff_velocity_scale=float(g.get('ff_velocity_scale', 1.0)))
            self.create_subscription(JointTrajectory, f'{name}/joint_trajectory',
                                     self._on_trajectory, 10)
        self.create_timer(1.0 / self.rate, self._update)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg):
        twist = msg.twist if isinstance(msg, TwistStamped) else msg
        self.cmd = (twist.linear.x, twist.angular.z)
        stamp = stamp_to_sec(msg.header.stamp) if isinstance(msg, TwistStamped) else 0.0
        self.cmd_time = stamp if stamp > 0 else self.now()

    def _on_trajectory(self, msg):
        start = stamp_to_sec(msg.header.stamp)
        start = start if start > 0 else self.now()
        for k, joint in enumerate(msg.joint_names):
            follower = self.followers.get(joint)
            if follower is None or joint not in self.positions:
                continue
            points = []
            for pt in msg.points:
                t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                vel = pt.velocities[k] if len(pt.velocities) > k else None
                points.append((t, pt.positions[k], vel))
            follower.set_trajectory(points, start, self.positions[joint])

    def _on_joint_states(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.positions[name] = pos
        self.state_time = stamp_to_sec(msg.header.stamp)
        if self.drive is None or self.left not in self.positions:
            return
        dt = 0.0 if self.last_odom_time is None else self.state_time - self.last_odom_time
        if self.last_odom_time is None or dt > 0:
            self.drive.update_odometry(self.positions[self.left], self.positions[self.right], dt)
            self.last_odom_time = self.state_time
            self._publish_odom(msg.header.stamp)

    def _update(self):
        if self.state_time is None:
            return                       # the simulator does not run yet
        now = self.now()
        names, velocities = [], []
        if self.drive is not None:
            age = None if self.cmd_time is None else now - self.cmd_time
            wl, wr = self.drive.update_command(self.cmd[0], self.cmd[1], age, 1.0 / self.rate)
            names += [self.left, self.right]
            velocities += [wl, wr]
        for joint, follower in self.followers.items():
            if joint in self.positions:
                names.append(joint)
                velocities.append(follower.command(now, self.positions[joint]))
        if names:
            cmd = JointState(name=names, velocity=velocities)
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(cmd)

    def _publish_odom(self, stamp):
        d = self.drive
        m = Odometry()
        m.header.stamp = stamp
        m.header.frame_id = self.odom_frame
        m.child_frame_id = self.base_frame
        m.pose.pose.position.x = d.x
        m.pose.pose.position.y = d.y
        m.pose.pose.orientation.z = math.sin(d.yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(d.yaw / 2.0)
        m.twist.twist.linear.x = d.vx
        m.twist.twist.angular.z = d.wz
        self.odom_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerEmulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
