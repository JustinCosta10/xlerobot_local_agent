"""Render only matching processed frames, outside the GPU inference process."""

import json

from cv_bridge import CvBridge
import message_filters
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from .processing import render_segmentation


class YoloeVisualizer(Node):
    def __init__(self, **kwargs):
        super().__init__("yoloe_visualizer", **kwargs)
        self.alpha = self.declare_parameter(
            "mask_alpha", 0.45, ParameterDescriptor(read_only=True)).value
        if not 0 <= self.alpha <= 1:
            raise ValueError("mask_alpha must be in [0, 1]")
        self.bridge = CvBridge()
        self.names = {}
        self.raw = self.create_publisher(Image, "yoloe/segmentation/image_raw", 1)
        self.compressed = self.create_publisher(
            CompressedImage, "yoloe/segmentation/image_raw/compressed", 1)
        self.metadata = self.create_subscription(
            String, "yoloe/classes", self.set_classes,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.inputs = [message_filters.Subscriber(self, msg_type, topic, qos_profile=1)
                       for msg_type, topic in (
                           (Image, "yoloe/source_image"), (Image, "yoloe/semantic_mask"),
                           (Detection2DArray, "yoloe/detections"))]
        self.sync = message_filters.TimeSynchronizer(self.inputs, queue_size=3)
        self.sync.registerCallback(self.render)

    def set_classes(self, message):
        self.names = json.loads(message.data)["labels"]

    def render(self, source, semantic, detections):
        want_raw = self.raw.get_subscription_count() > 0
        want_compressed = self.compressed.get_subscription_count() > 0
        if not (want_raw or want_compressed):
            return
        image = self.bridge.imgmsg_to_cv2(source, desired_encoding="bgr8")
        labels = self.bridge.imgmsg_to_cv2(semantic, desired_encoding="passthrough")
        output = render_segmentation(image, labels, detections, self.names, self.alpha)
        if want_raw:
            message = self.bridge.cv2_to_imgmsg(output, encoding="bgr8")
            message.header = source.header
            self.raw.publish(message)
        if want_compressed:
            message = self.bridge.cv2_to_compressed_imgmsg(output, dst_format="jpg")
            message.header = source.header
            message.format = "bgr8; jpeg compressed bgr8"
            self.compressed.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloeVisualizer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
