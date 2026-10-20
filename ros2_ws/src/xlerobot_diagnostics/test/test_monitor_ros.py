"""Exercise DDS QoS and wall timers with simulated time enabled; no GPU needed."""

import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from xlerobot_diagnostics.monitor import BridgeMonitor


def test_diagnostics_work_before_clock_and_after_clock_stalls():
    context = Context()
    rclpy.init(context=context, domain_id=92)
    monitor = BridgeMonitor(context=context, namespace="test_robot", parameter_overrides=[
        Parameter("use_sim_time", value=True),
        Parameter("stale_after", value=0.5),
    ])
    peer = Node("test_peer", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(monitor)
    executor.add_node(peer)
    statuses = []
    peer.create_subscription(DiagnosticArray, "/diagnostics", statuses.append, 10)
    clock_pub = peer.create_publisher(Clock, "/clock", qos_profile_sensor_data)
    joint_pub = peer.create_publisher(JointState, "/test_robot/joint_states",
                                      qos_profile_sensor_data)
    tf_pub = peer.create_publisher(TFMessage, "/tf", qos_profile_sensor_data)

    def spin_until(predicate, timeout=6.0, publish=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publish:
                publish()
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError("DDS condition did not become true before timeout")

    try:
        # A ROS-time timer would never fire here because /clock has not started.
        spin_until(lambda: bool(statuses))
        assert statuses[-1].status[0].level == DiagnosticStatus.WARN
        assert monitor.get_clock().now().nanoseconds == 0
        clock = Clock()
        transform = TransformStamped(child_frame_id="head_tilt_link")
        transform.header.frame_id = "world"
        transform.transform.rotation.w = 1.0
        tf = TFMessage(transforms=[transform])
        joints = JointState(name=["head_pan_joint"], position=[0.0])

        def publish(advance=True):
            if advance:
                clock.clock.sec += 1
            clock_pub.publish(clock)
            joint_pub.publish(joints)
            tf_pub.publish(tf)

        spin_until(lambda: statuses[-1].status[0].level == DiagnosticStatus.OK,
                   publish=publish)
        spin_until(lambda: "not advancing" in statuses[-1].status[0].message,
                   publish=lambda: publish(advance=False))
        assert statuses[-1].status[0].level == DiagnosticStatus.WARN
    finally:
        executor.shutdown()
        peer.destroy_node()
        monitor.destroy_node()
        context.shutdown()
