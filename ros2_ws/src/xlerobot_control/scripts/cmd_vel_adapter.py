#!/usr/bin/env python3
"""Stamp public Twist commands for the shared differential-drive controller."""

import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class CmdVelAdapter(Node):
    def __init__(self):
        super().__init__("cmd_vel_adapter")
        self.commands = self.create_publisher(TwistStamped, "cmd_vel", 1)
        self.create_subscription(
            Twist, "/cmd_vel", self.on_command,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
    def on_command(self, twist):
        if not all(math.isfinite(value) for value in (twist.linear.x, twist.angular.z)):
            self.get_logger().warning("Ignoring non-finite cmd_vel")
            return
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return  # Wait for the first simulation clock before accepting commands.
        message = TwistStamped()
        message.header.stamp = now.to_msg()
        message.header.frame_id = "base_link"
        message.twist = twist
        # Forward each received command once: do not refresh an expired command.
        self.commands.publish(message)


def main():
    rclpy.init()
    node = CmdVelAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
