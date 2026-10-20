"""Gate autonomous motion on fresh camera, mapping and localization data."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nvblox_msgs.msg import DistanceMapSlice
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def fresh(sample, wall_now, sim_now, timeout):
    return (sample is not None and 0 <= wall_now - sample[0] <= timeout
            and -0.1 <= sim_now - sample[1] <= timeout)


class MappingStatus(Node):
    def __init__(self, name):
        super().__init__(name)
        self.samples = {}
        self.create_subscription(Image, "/camera/depth/image_rect_raw",
                                 lambda m: self.record("depth", m), qos_profile_sensor_data)
        self.create_subscription(DistanceMapSlice, "nvblox_node/static_map_slice",
                                 self.on_map, 1)
        self.create_subscription(PoseStamped, "sim_ground_truth",
                                 lambda m: self.record("pose", m), 10)

    def record(self, name, message):
        self.samples[name] = (time.monotonic(), stamp_seconds(message.header.stamp))

    def on_map(self, message):
        if message.width and message.height and message.data:
            self.record("map", message)

    def ready(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        return now > 0 and all(fresh(self.samples.get(key), time.monotonic(), now, 2.0)
                               for key in ("depth", "map", "pose"))


class NavigationGuard(MappingStatus):
    def __init__(self):
        super().__init__("navigation_guard")
        self.output = self.create_publisher(TwistStamped, "cmd_vel", 1)
        self.command = None
        self.received_at = None
        self.was_blocked = False
        self.create_subscription(TwistStamped, "cmd_vel_navigation", self.on_command, 1)
        # A wall-clock watchdog also expires if Isaac pauses or its clock stops.
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_command(self, message):
        self.command = message
        self.received_at = (time.monotonic(), stamp_seconds(message.header.stamp))

    def tick(self):
        if self.command is None:
            return
        now = self.get_clock().now()
        valid = fresh(self.received_at, time.monotonic(), now.nanoseconds * 1e-9, 0.5)
        ready = self.ready()
        output = TwistStamped()
        output.header.stamp = now.to_msg()
        output.header.frame_id = "base_link"
        v, w = self.command.twist.linear.x, self.command.twist.angular.z
        if valid and ready and math.isfinite(v) and math.isfinite(w):
            output.twist.linear.x = max(-0.1, min(0.1, v))
            output.twist.angular.z = max(-0.2, min(0.2, w))
        elif valid and not ready and not self.was_blocked:
            self.get_logger().warning("Stopping navigation: depth, map or localization is stale")
        self.was_blocked = not ready
        self.output.publish(output)
        if not valid:
            # Stop once, then allow manual driving while navigation is idle.
            self.command = None


def main():
    rclpy.init()
    node = NavigationGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def wait_for_mapping():
    rclpy.init()
    node = MappingStatus("wait_for_mapping")
    deadline = time.monotonic() + 240
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.ready():
                node.get_logger().info("Live depth map ready; activating Nav2")
                return
        raise RuntimeError("Depth mapping did not become ready within 240 seconds")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
