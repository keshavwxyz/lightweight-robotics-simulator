"""ROS 2 bridge: publications, TF tree, cmd_vel and simulation services.

The simulation loop runs on the test thread, as in the application. A separate
rclpy node is the external ROS 2 application.
"""
import threading
import time

import pytest

from .conftest import diff_drive_config, requires_ros

pytestmark = requires_ros


class ExternalApp:
    def __init__(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from sensor_msgs.msg import Imu, JointState, LaserScan
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage

        self.ctx = Context()
        rclpy.init(context=self.ctx)
        self.node = Node("external_test_app", context=self.ctx)
        self.msgs = {k: [] for k in ("joint_states", "tf", "tf_static", "odom", "imu", "scan")}
        n = self.node
        n.create_subscription(JointState, "/diff_bot/joint_states", lambda m: self.msgs["joint_states"].append(m), 10)
        n.create_subscription(Odometry, "/diff_bot/odom", lambda m: self.msgs["odom"].append(m), 10)
        n.create_subscription(Imu, "/diff_bot/imu/data", lambda m: self.msgs["imu"].append(m), 10)
        n.create_subscription(LaserScan, "/diff_bot/scan", lambda m: self.msgs["scan"].append(m), 10)
        n.create_subscription(TFMessage, "/tf", lambda m: self.msgs["tf"].append(m), 100)
        n.create_subscription(TFMessage, "/tf_static", lambda m: self.msgs["tf_static"].append(m),
                              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.executor = SingleThreadedExecutor(context=self.ctx)
        self.executor.add_node(n)
        self._run = True
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        while self._run:
            self.executor.spin_once(timeout_sec=0.05)

    def call(self, srv_type, name, request, timeout=10.0):
        client = self.node.create_client(srv_type, name)
        assert client.wait_for_service(timeout_sec=timeout), f"service {name} not available"
        fut = client.call_async(request)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.01)
        assert fut.done(), f"service {name} timed out"
        return fut.result()

    def close(self):
        import rclpy
        self._run = False
        self.thread.join(2)
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.ctx)


@pytest.fixture
def ros_setup(sim):
    from robot_simulator.ros2_bridge.bridge_manager import Ros2Bridge
    from robot_simulator.sensors import ImuConfig, Lidar2DConfig
    cfg = diff_drive_config()
    cfg.sensors = [ImuConfig(name="imu", link="base_link", rate_hz=50),
                   Lidar2DConfig(name="lidar", link="lidar_mount", xyz=(0, 0, 0.03), rate_hz=10)]
    robot = sim.spawn_robot(cfg)
    bridge = Ros2Bridge(sim)
    assert bridge.start(), bridge.error
    app = ExternalApp()
    sim.play()
    yield sim, robot, bridge, app
    app.close()
    bridge.stop()


def run_until(sim, cond, timeout=10.0):
    """Run the simulation loop at wall-clock speed until cond() is true."""
    t0 = last = time.perf_counter()
    while time.perf_counter() - t0 < timeout:
        now = time.perf_counter()
        sim.update(now - last)
        last = now
        if cond():
            return True
        time.sleep(0.005)
    return False


def test_joint_state_and_sensor_publication(ros_setup):
    sim, robot, bridge, app = ros_setup
    assert run_until(sim, lambda: all(app.msgs[k] for k in ("joint_states", "odom", "imu", "scan")))
    js = app.msgs["joint_states"][-1]
    assert list(js.name) == ["left_wheel_joint", "right_wheel_joint"]
    assert len(js.position) == len(js.velocity) == len(js.effort) == 2
    imu = app.msgs["imu"][-1]
    assert imu.header.frame_id == "imu_link"
    assert imu.linear_acceleration.z == pytest.approx(9.81, abs=0.2)
    scan = app.msgs["scan"][-1]
    assert scan.header.frame_id == "lidar_link" and len(scan.ranges) == 360
    odom = app.msgs["odom"][-1]
    assert odom.header.frame_id == "odom" and odom.child_frame_id == "base_footprint"


def test_tf_hierarchy_follows_urdf(ros_setup):
    sim, robot, bridge, app = ros_setup
    assert run_until(sim, lambda: app.msgs["tf"] and app.msgs["tf_static"])
    parent_of = {}
    for msg in [app.msgs["tf_static"][-1]] + app.msgs["tf"][-5:]:
        for t in msg.transforms:
            prev = parent_of.setdefault(t.child_frame_id, t.header.frame_id)
            assert prev == t.header.frame_id, f"conflicting parents for {t.child_frame_id}"
    # each URDF joint has the parent/child relation of the URDF
    for j in robot.model.joints.values():
        assert parent_of[j.child] == j.parent
    assert parent_of["base_footprint"] == "odom"
    assert parent_of["imu_link"] == "base_link"
    assert parent_of["lidar_link"] == "lidar_mount"
    # dynamic wheel transforms are on /tf, fixed joints on /tf_static
    dynamic = {t.child_frame_id for m in app.msgs["tf"] for t in m.transforms}
    assert {"left_wheel", "right_wheel", "base_footprint"} <= dynamic
    assert "base_link" not in dynamic
    # tree is connected: every frame reaches 'odom'
    for frame in parent_of:
        f, hops = frame, 0
        while f in parent_of and hops < 50:
            f, hops = parent_of[f], hops + 1
        assert f == "odom"


def test_cmd_vel_drives_robot(ros_setup):
    from geometry_msgs.msg import Twist
    sim, robot, bridge, app = ros_setup
    pub = app.node.create_publisher(Twist, "/diff_bot/cmd_vel", 10)
    msg = Twist()
    msg.linear.x = 0.4
    t_end = time.time() + 3.0
    x0 = robot.physics.link_transform("base_link")[0, 3]

    def publishing():
        if time.time() < t_end:
            pub.publish(msg)
            return False
        return True
    run_until(sim, publishing, timeout=5.0)
    x1 = robot.physics.link_transform("base_link")[0, 3]
    assert x1 - x0 > 0.5                                       # ~3 s at 0.4 m/s minus acceleration
    assert app.msgs["odom"][-1].twist.twist.linear.x == pytest.approx(0.4, abs=0.05)


def test_joint_commands_topic(ros_setup):
    from sensor_msgs.msg import JointState
    sim, robot, bridge, app = ros_setup
    pub = app.node.create_publisher(JointState, "/diff_bot/joint_commands", 10)
    cmd = JointState(name=["left_wheel_joint"], velocity=[3.0])

    def sent():
        pub.publish(cmd)
        return robot.motors.get("left_wheel_joint").config.target_velocity == 3.0
    assert run_until(sim, sent, timeout=5.0)


def test_pause_resume_reset_services(ros_setup):
    from std_srvs.srv import Trigger
    sim, robot, bridge, app = ros_setup
    run_until(sim, lambda: sim.sim_time > 0.5)
    result = {}

    def call(name):
        result[name] = app.call(Trigger, f"/sim/{name}", Trigger.Request())
    for name, expected in (("pause", "paused"), ("resume", "playing"), ("reset", None)):
        t = threading.Thread(target=call, args=(name,))
        t.start()
        assert run_until(sim, lambda: name in result, timeout=10.0)
        t.join()
        assert result[name].success
        if expected:
            assert sim.state.value == expected
    assert sim.sim_time < 0.2


@pytest.mark.skipif(not __import__("robot_simulator.ros2_bridge.ros_node", fromlist=["x"]).SIM_INTERFACES_AVAILABLE,
                    reason="simulation_interfaces not installed")
def test_spawn_and_delete_services(ros_setup):
    from simulation_interfaces.msg import Result
    from simulation_interfaces.srv import DeleteEntity, SpawnEntity
    from .conftest import ARM_URDF
    sim, robot, bridge, app = ros_setup
    result = {}

    def spawn():
        req = SpawnEntity.Request(name="arm", uri=ARM_URDF)
        req.initial_pose.pose.position.x = 2.0
        result["spawn"] = app.call(SpawnEntity, "/sim/spawn_entity", req)
    t = threading.Thread(target=spawn)
    t.start()
    assert run_until(sim, lambda: "spawn" in result)
    t.join()
    assert result["spawn"].result.result == Result.RESULT_OK
    assert result["spawn"].entity_name == "arm" and "arm" in sim.robots

    def delete():
        result["delete"] = app.call(DeleteEntity, "/sim/delete_entity", DeleteEntity.Request(entity="arm"))
    t = threading.Thread(target=delete)
    t.start()
    assert run_until(sim, lambda: "delete" in result)
    t.join()
    assert result["delete"].result.result == Result.RESULT_OK and "arm" not in sim.robots
