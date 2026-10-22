#!/usr/bin/env python3
"""GPU acceptance: recorded RGB at 30 Hz, or the camera in full navigation bringup.

Run after sourcing env.sh. Uses an isolated ROS domain and stops its own launch.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--navigation", action="store_true", help="also launch Isaac/cuVSLAM/nvblox/Nav2")
    parser.add_argument("--domain-id", type=int, default=84)
    parser.add_argument("--seconds", type=float, default=8)
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 101 or args.seconds < 3:
        parser.error("domain-id must be in 0..101 and seconds must be at least 3")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    workspace = Path(__file__).resolve().parents[1]
    log_path = workspace / "log" / ("yoloe-navigation-test.log" if args.navigation else "yoloe-test.log")
    log_path.parent.mkdir(exist_ok=True)
    command = (["ros2", "launch", "xlerobot_navigation", "sim_navigation.launch.py",
                "headless:=true", "rviz:=false", "foxglove:=false"] if args.navigation else
               ["ros2", "launch", "xlerobot_perception", "yoloe.launch.py",
                "image_topic:=/yoloe_test/camera"])
    command.append("namespace:=yoloe_test")
    bridge = CvBridge()
    rclpy.init()
    node = Node("yoloe_acceptance")
    detections, masks, overlays, classes = [], {}, {}, {}

    def stamp(message):
        return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec

    def on_detection(message):
        detections.append((time.monotonic(), message))

    def on_mask(message):
        masks[stamp(message)] = message

    def on_overlay(message):
        overlays[stamp(message)] = message

    def on_classes(message):
        classes.update(json.loads(message.data)["labels"])

    subscriptions = [node.create_subscription(msg_type, f"/yoloe_test/yoloe/{topic}", callback, qos)
                     for msg_type, topic, callback, qos in (
                         (Detection2DArray, "detections", on_detection, 10),
                         (Image, "semantic_mask", on_mask, 10),
                         (CompressedImage, "segmentation/image_raw/compressed", on_overlay, 10),
                         (String, "classes", on_classes, QoSProfile(
                             depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))]
    timer = None
    if not args.navigation:
        assets = workspace / ".deps/yoloe/lib/python3.12/site-packages/ultralytics/assets/bus.jpg"
        image = cv2.imread(str(assets))
        if image is None:
            raise RuntimeError("Install YOLOE first: bash scripts/install_yoloe.sh")
        # A real camera-shaped non-square frame exercises unpadding/rescaling.
        image = cv2.resize(image, (480, 640))
        publisher = node.create_publisher(Image, "/yoloe_test/camera", 1)

        def publish():
            message = bridge.cv2_to_imgmsg(image, "bgr8")
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = "camera_color_optical_frame"
            publisher.publish(message)

        timer = node.create_timer(1 / 30, publish)
    with log_path.open("w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + (240 if args.navigation else 90)
            started = None
            while started is None or time.monotonic() - started < args.seconds:
                assert process.poll() is None, f"Launch exited; see {log_path}"
                assert time.monotonic() < deadline, f"No YOLOE output; see {log_path}"
                rclpy.spin_once(node, timeout_sec=.05)
                if started is None and len(overlays) >= 3 and classes:
                    started = time.monotonic()
                    detections.clear()
            assert len(detections) >= 3, "Too few detections messages"
            elapsed = detections[-1][0] - detections[0][0]
            rate = (len(detections) - 1) / elapsed
            assert 4.0 <= rate <= 5.5, f"Expected about 5 Hz, got {rate:.2f} Hz"
            aligned = [message for _, message in detections
                       if stamp(message) in masks and stamp(message) in overlays]
            assert len(aligned) >= len(detections) - 2, "Missing timestamp-aligned mask/overlay outputs"
            assert len({stamp(m) for _, m in detections}) == len(detections), "Repeated old frames"
            detected_names = set()
            ages = []
            for message in aligned:
                key = stamp(message)
                mask = masks[key]
                assert mask.header == message.header == overlays[key].header
                assert mask.encoding == "16UC1"
                labels = bridge.imgmsg_to_cv2(mask)
                assert all(str(int(label)) in classes for label in np.unique(labels))
                rendered = bridge.compressed_imgmsg_to_cv2(overlays[key])
                assert rendered.shape[:2] == labels.shape
                for detection in message.detections:
                    detected_names.add(classes[detection.results[0].hypothesis.class_id])
            if not args.navigation:
                assert detected_names and any(bridge.imgmsg_to_cv2(masks[stamp(m)]).any() for m in aligned)
                # End-to-end wall latency includes ROS delivery and CPU mask construction.
                for arrival, message in detections:
                    wall_at_arrival = time.time() - (time.monotonic() - arrival)
                    ages.append(wall_at_arrival - stamp(message) / 1e9)
                assert max(ages) < .6, f"Stale detection feed: {max(ages):.3f}s"
            if args.navigation:
                assert "Live depth map ready; activating Nav2" in log_path.read_text(), "Navigation not ready"
            output_path = log_path.with_suffix(".jpg")
            cv2.imwrite(str(output_path), bridge.compressed_imgmsg_to_cv2(overlays[stamp(aligned[-1])]))
            print(f"PASS: {rate:.2f} Hz, {len(aligned)} synchronized frames, "
                  f"{len(classes)-1} known classes; detected {sorted(detected_names)}")
            if ages:
                print(f"Detection latency: median {np.median(ages)*1000:.0f} ms, max {max(ages)*1000:.0f} ms")
            print(f"Overlay: {output_path}\nLog: {log_path}")
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=40)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=10)
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
