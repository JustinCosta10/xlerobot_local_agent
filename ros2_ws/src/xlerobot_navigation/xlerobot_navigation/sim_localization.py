"""Localize the flat-world simulator while preserving wheel-odometry TF ownership."""

from collections import deque
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def correction(map_pose, odom_transform):
    """Compute map->odom from two same-time base poses, in the horizontal plane."""
    angle = yaw(map_pose.orientation) - yaw(odom_transform.rotation)
    c, s = math.cos(angle), math.sin(angle)
    p = odom_transform.translation
    return (map_pose.position.x - c * p.x + s * p.y,
            map_pose.position.y - s * p.x - c * p.y,
            map_pose.position.z - p.z, angle)


class SimLocalization(Node):
    def __init__(self):
        super().__init__("sim_localization")
        self.buffer = Buffer(cache_time=Duration(seconds=10), node=self)
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.pending = deque(maxlen=100)
        self.create_subscription(PoseStamped, "sim_ground_truth", lambda msg: self.pending.append(msg), 10)
        self.create_timer(1.0 / 30.0, self.update)

    def update(self):
        # Waiting for same-time odometry avoids mixing a current camera pose with
        # an older wheel pose. nvblox also queries the resulting TF at image time.
        while self.pending:
            message = self.pending[0]
            stamp = Time.from_msg(message.header.stamp)
            if message.header.frame_id != "map":
                self.pending.popleft()
                continue
            try:
                odom = self.buffer.lookup_transform("odom", "base_link", stamp)
            except TransformException:
                if (self.get_clock().now() - stamp).nanoseconds > 2_000_000_000:
                    self.pending.popleft()
                    continue
                return
            self.pending.popleft()
            x, y, z, angle = correction(message.pose, odom.transform)
            tf = TransformStamped()
            tf.header.frame_id = "map"
            # Match localization conventions: allow consumers to query slightly
            # ahead of the last sample. Corrections vary slowly in this flat scene.
            tf.header.stamp = (stamp + Duration(seconds=0.1)).to_msg()
            tf.child_frame_id = "odom"
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = x, y, z
            tf.transform.rotation.z = math.sin(angle / 2)
            tf.transform.rotation.w = math.cos(angle / 2)
            self.broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = SimLocalization()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
