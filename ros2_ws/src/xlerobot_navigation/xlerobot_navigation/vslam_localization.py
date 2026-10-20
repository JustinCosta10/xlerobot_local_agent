"""Adapt cuVSLAM's camera-rig pose to the robot's wheel-odometry TF tree.

This node does not estimate motion or consume simulator ground truth. cuVSLAM
estimates head_camera_link; timestamped joint TF removes articulated head motion.
The first valid base pose defines map X/Y/yaw; nominal axle height defines Z.
"""

from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster
from tf2_ros import TransformException, TransformListener


def matrix(position, orientation):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([
        orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()
    result[:3, 3] = [position.x, position.y, position.z]
    return result


def set_pose(position, orientation, value):
    position.x, position.y, position.z = map(float, value[:3, 3])
    orientation.x, orientation.y, orientation.z, orientation.w = map(
        float, Rotation.from_matrix(value[:3, :3]).as_quat())


def initial_alignment(vslam_sensor, base_sensor, base_height):
    initial_base = np.eye(4)
    initial_base[2, 3] = base_height
    return initial_base @ base_sensor @ np.linalg.inv(vslam_sensor)


class VslamLocalization(Node):
    def __init__(self):
        super().__init__("vslam_localization")
        self.height = self.declare_parameter("initial_base_height", 0.0635).value
        self.buffer = Buffer(cache_time=Duration(seconds=10), node=self)
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.output = self.create_publisher(PoseStamped, "localization/pose", 10)
        self.pending = deque(maxlen=30)
        self.alignment = None
        self.create_subscription(Odometry, "visual_slam/vis/slam_odometry", self.on_pose, 10)
        self.create_timer(1 / 60, self.update)

    def on_pose(self, message):
        if message.header.frame_id == "vslam_map" and message.child_frame_id == "head_camera_link":
            self.pending.append(message)

    def transform(self, value, parent, child, stamp):
        result = TransformStamped()
        result.header.frame_id, result.child_frame_id = parent, child
        result.header.stamp = stamp
        set_pose(result.transform.translation, result.transform.rotation, value)
        return result

    def update(self):
        while self.pending:
            message = self.pending[0]
            stamp = Time.from_msg(message.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            if age > 2 or age < -0.1:
                self.pending.popleft()
                continue
            try:
                mount = self.buffer.lookup_transform("base_link", message.child_frame_id, stamp).transform
                wheel = self.buffer.lookup_transform("odom", "base_link", stamp).transform
            except TransformException:
                return
            self.pending.popleft()
            sensor = matrix(message.pose.pose.position, message.pose.pose.orientation)
            base_sensor = matrix(mount.translation, mount.rotation)
            if self.alignment is None:
                self.alignment = initial_alignment(sensor, base_sensor, self.height)
                self.static_broadcaster.sendTransform(self.transform(
                    self.alignment, "map", "vslam_map", message.header.stamp))
                self.get_logger().info("cuVSLAM initialized: map origin is the initial base position")
            map_base = self.alignment @ sensor @ np.linalg.inv(base_sensor)
            map_odom = map_base @ np.linalg.inv(matrix(wheel.translation, wheel.rotation))
            self.broadcaster.sendTransform(self.transform(
                map_odom, "map", "odom", message.header.stamp))
            pose = PoseStamped()
            pose.header.frame_id, pose.header.stamp = "map", message.header.stamp
            set_pose(pose.pose.position, pose.pose.orientation, map_base)
            self.output.publish(pose)


def main():
    rclpy.init()
    node = VslamLocalization()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
