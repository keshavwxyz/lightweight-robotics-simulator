"""ROS 2 bridge.

Threads:
  * The simulation loop (main thread) calls the `SimulationListener` methods.
    These methods only add an item to a deque with a size limit. They then
    trigger a guard condition. They do not block and do not wait for ROS.
  * One bridge thread runs an rclpy executor. The guard-condition callback
    removes the items from the deque, makes the messages and publishes them.
    The subscriber and service callbacks run on the same thread. They only
    add commands to the thread-safe `CommandQueue` of the simulation.

If the bridge is too slow, it discards the oldest items. All sensor data has
time stamps. Thus, the data stays consistent for the subscribers.
"""
from __future__ import annotations

import logging
import threading
from collections import deque
from typing import Optional

from ..core.configuration import Ros2Config
from ..core.snapshots import RobotDescriptor, RobotStateSnapshot
from . import publishers as pubs
from .ros_node import ROS_AVAILABLE, ROS_IMPORT_ERROR, create_node, ros_distro, time_msg
from .tf_manager import TFManager, to_transform_stamped

log = logging.getLogger(__name__)


class _RobotPorts:
    def __init__(self):
        self.joint_states = None
        self.odom = None
        self.sensors: dict[str, tuple] = {}     # sensor name -> (publisher, extra publisher)
        self.subscriptions: list = []


class Ros2Bridge:
    def __init__(self, sim, config: Optional[Ros2Config] = None):
        self.sim = sim
        self._config_override = config
        self.status = "disabled"
        self.error = ""
        self._queue: deque = deque(maxlen=2000)
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._node = None
        self._ctx = None
        self._executor = None
        self._guard = None
        self._ports: dict[str, _RobotPorts] = {}
        self._descs: dict[str, RobotDescriptor] = {}
        self.tf = TFManager()
        self.published = 0
        self.dropped = 0

    @property
    def config(self) -> Ros2Config:
        """Return the ROS 2 settings.

        The bridge reads them from the simulation at start-up. A project that
        loads after the bridge was made replaces sim.config.ros2.
        """
        return self._config_override or self.sim.config.ros2

    @staticmethod
    def available() -> bool:
        return ROS_AVAILABLE

    def start(self) -> bool:
        if self._running:
            return True
        if not ROS_AVAILABLE:
            self.status = "unavailable"
            self.error = f"rclpy not importable ({ROS_IMPORT_ERROR}); source /opt/ros/<distro>/setup.bash"
            log.warning("ROS 2 bridge unavailable: %s", self.error)
            return False
        ready = threading.Event()
        self._running = True
        self._thread = threading.Thread(target=self._thread_main, args=(ready,), name="ros2_bridge", daemon=True)
        self._thread.start()
        ready.wait(10.0)
        if self.status != "connected":
            self._running = False
            return False
        self.sim.state_rate_hz = self.config.state_publish_rate_hz
        self.sim.clock_rate_hz = self.config.clock_publish_rate_hz if self.config.publish_clock else 0.0
        self.sim.add_listener(self)
        self.sim.publish_state_now()
        return True

    def stop(self) -> None:
        if not self._running:
            return
        self.sim.remove_listener(self)
        self.sim.state_rate_hz = 0.0
        self.sim.clock_rate_hz = 0.0
        self._running = False
        if self._guard is not None:
            self._guard.trigger()
        if self._thread:
            self._thread.join(timeout=5.0)
        self.status = "disabled"
        self._ports.clear()
        self._descs.clear()
        self.tf = TFManager()

    def _thread_main(self, ready: threading.Event) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rcl_interfaces.msg import SetParametersResult
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage

        from ..core.commands import SimControlCommand
        from .services import create_services

        try:
            self._ctx, node = create_node(self.config.node_name, self.config.domain_id)
        except Exception as exc:
            self.status, self.error = "error", str(exc)
            log.error("Could not start ROS 2 node: %s", exc)
            ready.set()
            return
        self._node = node
        try:
            static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=ReliabilityPolicy.RELIABLE)
            self._tf_pub = node.create_publisher(TFMessage, "/tf", 100)
            self._tf_static_pub = node.create_publisher(TFMessage, "/tf_static", static_qos)
            self._clock_pub = node.create_publisher(Clock, "/clock", 10)
            self._services = create_services(node, self.sim.commands, lambda: self.sim.state.value)

            node.declare_parameter("sim_speed", float(self.sim.speed))

            def on_params(params):
                for prm in params:
                    if prm.name == "sim_speed":
                        try:
                            v = float(prm.value)
                        except (TypeError, ValueError):
                            return SetParametersResult(successful=False, reason="sim_speed must be a number")
                        if v <= 0:
                            return SetParametersResult(successful=False, reason="sim_speed must be > 0")
                        self.sim.commands.put(SimControlCommand("speed", v))
                return SetParametersResult(successful=True)
            node.add_on_set_parameters_callback(on_params)

            self._executor = SingleThreadedExecutor(context=self._ctx)
            self._executor.add_node(node)
            self._guard = node.create_guard_condition(self._drain)
            self.status = "connected"
            log.info("ROS 2 bridge started (node '/%s', distro %s, domain %s)", self.config.node_name, ros_distro(),
                     self.config.domain_id if self.config.domain_id is not None else "env")
        except Exception as exc:
            self.status, self.error = "error", str(exc)
            log.exception("ROS 2 bridge set-up failed")
            ready.set()
            return
        ready.set()
        try:
            while self._running and rclpy.ok(context=self._ctx):
                self._executor.spin_once(timeout_sec=0.5)
        except Exception:
            log.exception("ROS 2 executor stopped with an error")
            self.status = "error"
        finally:
            try:
                self._executor.shutdown()
                node.destroy_node()
                rclpy.shutdown(context=self._ctx)
            except Exception:
                pass

    def _push(self, item) -> None:
        if len(self._queue) == self._queue.maxlen:
            self.dropped += 1
        self._queue.append(item)
        g = self._guard
        if g is not None:
            g.trigger()

    def on_robot_added(self, desc: RobotDescriptor) -> None:
        self._push(("add", desc))

    def on_robot_changed(self, desc: RobotDescriptor) -> None:
        self._push(("change", desc))

    def on_robot_removed(self, name: str) -> None:
        self._push(("remove", name))

    def on_robot_state(self, snap: RobotStateSnapshot) -> None:
        self._push(("state", snap))

    def on_sensor_data(self, robot: str, sensor: str, sensor_type: str, data) -> None:
        self._push(("sensor", (robot, sensor, sensor_type, data)))

    def on_clock(self, t: float) -> None:
        self._push(("clock", t))

    def on_reset(self) -> None:
        self._push(("reset", None))

    def _drain(self) -> None:
        while self._queue:
            try:
                kind, payload = self._queue.popleft()
            except IndexError:
                break
            try:
                getattr(self, f"_handle_{kind}")(payload)
            except Exception:
                log.exception("ROS 2 bridge failed to handle '%s'", kind)

    def _handle_add(self, desc: RobotDescriptor) -> None:
        if not desc.ros_enabled:
            return
        ports = _RobotPorts()
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState
        from .subscribers import create_robot_subscribers
        n = self._node
        ports.joint_states = n.create_publisher(JointState, pubs.resolve_topic(desc, desc.joint_states_topic), 10)
        if desc.odom_enabled:
            ports.odom = n.create_publisher(Odometry, pubs.resolve_topic(desc, desc.odom_topic), 10)
        ports.subscriptions = create_robot_subscribers(
            n, desc.name, self.sim.commands, self.config.cmd_vel_stamped,
            pubs.resolve_topic(desc, desc.cmd_vel_topic), pubs.resolve_topic(desc, desc.joint_commands_topic))
        self._ports[desc.name] = ports
        self._descs[desc.name] = desc
        self._create_sensor_publishers(desc, ports)
        self.tf.set_robot(desc)
        self._publish_static()
        log.info("ROS 2: robot '%s' registered (topics under %s/)", desc.name, desc.namespace or "(root)")

    def _create_sensor_publishers(self, desc: RobotDescriptor, ports: _RobotPorts) -> None:
        from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
        n = self._node
        for s in desc.sensors:
            topic = pubs.sensor_topic(desc, s.name)
            if s.type == "imu":
                ports.sensors[s.name] = (n.create_publisher(Imu, topic, 10), None)
            elif s.type == "lidar_2d":
                ports.sensors[s.name] = (n.create_publisher(LaserScan, topic, 10), None)
            elif s.type == "camera":
                ports.sensors[s.name] = (n.create_publisher(Image, topic, 5),
                                         n.create_publisher(CameraInfo, pubs.camera_info_topic(topic), 5))

    def _destroy_ports(self, ports: _RobotPorts, keep_subscriptions: bool = False) -> None:
        n = self._node
        for pub, extra in ports.sensors.values():
            n.destroy_publisher(pub)
            if extra is not None:
                n.destroy_publisher(extra)
        ports.sensors.clear()
        if not keep_subscriptions:
            for p in (ports.joint_states, ports.odom):
                if p is not None:
                    n.destroy_publisher(p)
            for s in ports.subscriptions:
                n.destroy_subscription(s)

    def _handle_change(self, desc: RobotDescriptor) -> None:
        ports = self._ports.get(desc.name)
        old = self._descs.get(desc.name)
        ros_changed = old is not None and (
            (old.ros_enabled, old.namespace, old.joint_states_topic, old.cmd_vel_topic,
             old.joint_commands_topic, old.odom_topic) !=
            (desc.ros_enabled, desc.namespace, desc.joint_states_topic, desc.cmd_vel_topic,
             desc.joint_commands_topic, desc.odom_topic))
        if ports is not None and (ros_changed or not desc.ros_enabled):
            self._handle_remove(desc.name)        # make all publishers and subscribers again with the new names
            ports = None
        if ports is None:
            return self._handle_add(desc)
        self._destroy_ports(ports, keep_subscriptions=True)
        if old is not None and old.odom_enabled != desc.odom_enabled:
            if ports.odom is not None:
                self._node.destroy_publisher(ports.odom)
                ports.odom = None
            if desc.odom_enabled:
                from nav_msgs.msg import Odometry
                ports.odom = self._node.create_publisher(Odometry, pubs.resolve_topic(desc, desc.odom_topic), 10)
        self._descs[desc.name] = desc
        self._create_sensor_publishers(desc, ports)
        self.tf.set_robot(desc)
        self._publish_static()

    def _handle_remove(self, name: str) -> None:
        ports = self._ports.pop(name, None)
        if ports is not None:
            self._destroy_ports(ports)
        self._descs.pop(name, None)
        self.tf.remove_robot(name)
        self._publish_static()

    def _handle_state(self, snap: RobotStateSnapshot) -> None:
        ports = self._ports.get(snap.robot)
        desc = self._descs.get(snap.robot)
        if ports is None or desc is None:
            return
        stamp = time_msg(snap.stamp)
        if snap.joint_names:
            ports.joint_states.publish(pubs.joint_state_msg(snap))
        if ports.odom is not None and desc.odom_enabled:
            ports.odom.publish(pubs.odometry_msg(desc, snap))
        from tf2_msgs.msg import TFMessage
        tfs = [to_transform_stamped(p, c, T, stamp) for p, c, T in self.tf.dynamic(snap)]
        if tfs:
            self._tf_pub.publish(TFMessage(transforms=tfs))
        if self.tf.fixed_base_root(snap):
            self._publish_static()
        self.published += 1

    def _handle_sensor(self, payload) -> None:
        robot, sensor, stype, data = payload
        ports = self._ports.get(robot)
        if ports is None or sensor not in ports.sensors:
            return
        pub, extra = ports.sensors[sensor]
        if stype == "imu":
            pub.publish(pubs.imu_msg(data))
        elif stype == "lidar_2d":
            pub.publish(pubs.laser_scan_msg(data))
        elif stype == "camera":
            pub.publish(pubs.image_msg(data))
            extra.publish(pubs.camera_info_msg(data))
        self.published += 1

    def _handle_clock(self, t: float) -> None:
        from rosgraph_msgs.msg import Clock
        self._clock_pub.publish(Clock(clock=time_msg(t)))

    def _handle_reset(self, _payload) -> None:
        pass  # The time goes back. Nodes with use_sim_time find this themselves.

    def _publish_static(self) -> None:
        from tf2_msgs.msg import TFMessage
        stamp = time_msg(self.sim.sim_time)
        tfs = [to_transform_stamped(p, c, T, stamp) for p, c, T in self.tf.all_static()]
        # Transient local with depth 1: the latched message must contain all static transforms.
        self._tf_static_pub.publish(TFMessage(transforms=tfs))

    def topics(self) -> list[tuple[str, str]]:
        out = [("/clock", "rosgraph_msgs/msg/Clock"), ("/tf", "tf2_msgs/msg/TFMessage"),
               ("/tf_static", "tf2_msgs/msg/TFMessage")]
        for name, desc in self._descs.items():
            r = lambda t: pubs.resolve_topic(desc, t)  # noqa: E731
            out.append((r(desc.joint_states_topic), "sensor_msgs/msg/JointState"))
            if desc.odom_enabled:
                out.append((r(desc.odom_topic), "nav_msgs/msg/Odometry"))
            out.append((r(desc.cmd_vel_topic),
                        "geometry_msgs/msg/Twist" + ("Stamped" if self.config.cmd_vel_stamped else "")))
            out.append((r(desc.joint_commands_topic), "sensor_msgs/msg/JointState"))
            for s in desc.sensors:
                t = pubs.sensor_topic(desc, s.name)
                typ = {"imu": "sensor_msgs/msg/Imu", "lidar_2d": "sensor_msgs/msg/LaserScan",
                       "camera": "sensor_msgs/msg/Image"}[s.type]
                out.append((t, typ))
                if s.type == "camera":
                    out.append((pubs.camera_info_topic(t), "sensor_msgs/msg/CameraInfo"))
        return out
