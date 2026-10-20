"""Convert Isaac's aligned depth in meters to cuVSLAM's uint16 millimeters."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image


def millimeter_depth(message):
    if message.encoding != "32FC1" or message.step < message.width * 4:
        raise ValueError("Expected a 32FC1 depth image in meters")
    depth = np.ndarray((message.height, message.width),
                       dtype=">f4" if message.is_bigendian else "<f4",
                       buffer=message.data, strides=(message.step, 4))
    valid = np.isfinite(depth) & (depth > 0) & (depth < 65.535)
    result = np.zeros(depth.shape, dtype="<u2")
    result[valid] = np.maximum(1, np.rint(depth[valid] * 1000)).astype("<u2")
    output = Image()
    output.header = message.header
    output.height, output.width = message.height, message.width
    output.encoding, output.is_bigendian = "mono16", 0
    output.step = message.width * 2
    output.data = result.tobytes()
    return output


class DepthConversion(Node):
    def __init__(self):
        super().__init__("cuvslam_depth_conversion")
        qos = QoSProfile(depth=10)  # Reliable delivery preserves RGB/depth pairs.
        self.output = self.create_publisher(Image, "cuvslam/aligned_depth", qos)
        self.create_subscription(Image, "/camera/aligned_depth_to_color/image_raw_float",
                                 self.convert, qos)

    def convert(self, message):
        try:
            self.output.publish(millimeter_depth(message))
        except (ValueError, TypeError) as error:
            self.get_logger().error(str(error), throttle_duration_sec=5)


def main():
    rclpy.init()
    node = DepthConversion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

