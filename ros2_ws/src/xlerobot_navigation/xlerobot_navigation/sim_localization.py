"""Localize the simulator in 3D while preserving wheel-odometry TF ownership."""

from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from geometry_msgs.msg import PoseStamped, Transform, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def rotation(q):
    return Rotation.from_quat([q.x, q.y, q.z, q.w])


def correction(map_pose, odom_transform):
    """Compute T_map_odom = T_map_base * inverse(T_odom_base) at one time."""
    orientation = rotation(map_pose.orientation) * rotation(odom_transform.rotation).inv()
    p = odom_transform.translation
    position = np.array([map_pose.position.x, map_pose.position.y, map_pose.position.z])
    position -= orientation.apply([p.x, p.y, p.z])
    result = Transform()
    result.translation.x, result.translation.y, result.translation.z = map(float, position)
    (result.rotation.x, result.rotation.y,
     result.rotation.z, result.rotation.w) = map(float, orientation.as_quat())
    return result


class SimLocalization(Node):
    def __init__(self):
        super().__init__("sim_localization")
        self.buffer = Buffer(cache_time=Duration(seconds=10), node=self)
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.pose_publisher = self.create_publisher(PoseStamped, "localization/pose", 10)
        self.pending = deque(maxlen=100)
        self.create_subscription(PoseStamped, "sim_ground_truth", lambda msg: self.pending.append(msg), 10)
        self.create_timer(1.0 / 30.0, self.update)

    def update(self):
        # Waiting for same-time odometry avoids mixing a current camera pose with
        # an older wheel pose. nvblox also queries the resulting TF at image time.
        while self.pending:
            message = self.pending[0]
            stamp = Time.from_msg(message.header.stamp)
            if message.header.frame_id != "sim_world":
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
            tf = TransformStamped()
            tf.header.frame_id = "map"
            # Depth integration queries TF at image acquisition time. Future-dating
            # a changing correction misregisters observations during wheel slip/turns.
            tf.header.stamp = message.header.stamp
            tf.child_frame_id = "odom"
            # Preserve roll/pitch as well as yaw so the reconstructed camera pose
            # follows chassis tilt. Nav2 costmaps stay in the horizontal map frame.
            tf.transform = correction(message.pose, odom.transform)
            self.broadcaster.sendTransform(tf)
            pose = PoseStamped()
            pose.header.frame_id, pose.header.stamp = "map", message.header.stamp
            pose.pose = message.pose
            self.pose_publisher.publish(pose)


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
