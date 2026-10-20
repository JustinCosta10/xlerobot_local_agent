"""Backend-aware diagnostics and a bounded passive interface check."""

import argparse
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.clock import Clock as RclClock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from xlerobot_diagnostics.health import BridgeHealth


class BridgeMonitor(Node):
    def __init__(self, **kwargs):
        super().__init__("bridge_monitor", **kwargs)
        self.declare_parameter("stale_after", 3.0)
        self.declare_parameter("base_frame", "head_tilt_link")
        self.declare_parameter("require_clock", True)
        self.declare_parameter("require_backend_feedback", False)
        self.declare_parameter("expected_joints", ["head_pan_joint"])
        self.health = BridgeHealth(
            self.get_parameter("stale_after").value,
            self.get_parameter("base_frame").value,
            self.get_parameter("require_clock").value,
            self.get_parameter("expected_joints").value,
            self.get_parameter("require_backend_feedback").value,
        )
        self.create_subscription(Clock, "/clock", self.on_clock,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(JointState, "joint_states", self.on_joints,
                                 qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self.on_tf, qos_profile_sensor_data)
        self.create_subscription(JointState, "backend/joint_states", self.on_backend,
                                 qos_profile_sensor_data)
        self.diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.wall_clock = RclClock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0, self.publish_health, clock=self.wall_clock)
        self.previous_status = None

    def on_clock(self, msg):
        self.health.clock(msg.clock.sec * 1_000_000_000 + msg.clock.nanosec, time.monotonic())

    def on_joints(self, msg):
        self.health.joints(msg.name, msg.position, time.monotonic())

    def on_backend(self, msg):
        self.health.backend_feedback(msg.name, msg.position, msg.velocity, time.monotonic())

    def on_tf(self, msg):
        self.health.transforms(
            [(tf.header.frame_id, tf.child_frame_id) for tf in msg.transforms],
            time.monotonic(),
        )

    def publish_health(self):
        problems = self.health.problems(time.monotonic())
        status = DiagnosticStatus(
            level=DiagnosticStatus.WARN if problems else DiagnosticStatus.OK,
            name=f"{self.get_namespace().rstrip('/')}/robot_interface",
            hardware_id="xlerobot",
            message="; ".join(problems) if problems else "Robot interface healthy",
            values=[KeyValue(key="joint_count", value=str(len(self.health.joint_names))),
                    KeyValue(key="base_frame", value=self.health.base_frame)],
        )
        message = DiagnosticArray(status=[status])
        message.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics.publish(message)
        if status.message != self.previous_status:
            # rclpy caches severity per call site: keep these on separate lines.
            if problems:
                self.get_logger().warning(status.message)
            else:
                self.get_logger().info(status.message)
            self.previous_status = status.message


def main(args=None):
    rclpy.init(args=args)
    node = BridgeMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def check_main(args=None):
    parser = argparse.ArgumentParser(description="Wait for clock, valid joints, and base TF.")
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="wall seconds; simulator startup may need 120 or more")
    parsed, ros_args = parser.parse_known_args(args)
    if not math.isfinite(parsed.timeout) or parsed.timeout <= 0:
        parser.error("--timeout must be positive and finite")
    rclpy.init(args=ros_args)
    node = BridgeMonitor()
    deadline = time.monotonic() + parsed.timeout
    result = 1
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
            if not node.health.problems(time.monotonic()):
                clock_text = "advancing /clock" if node.health.require_clock else "hardware time"
                print(f"PASS: {clock_text}, {len(node.health.joint_names)} joints, "
                      f"and TF for {node.health.base_frame}")
                print("Joints: " + ", ".join(node.health.joint_names))
                result = 0
                break
        if result:
            print("FAIL: " + "; ".join(node.health.problems(time.monotonic())))
    except (KeyboardInterrupt, ExternalShutdownException):
        result = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return result
