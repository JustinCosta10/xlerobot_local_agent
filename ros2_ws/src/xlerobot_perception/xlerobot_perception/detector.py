"""ROS image subscription with one inference worker and no frame backlog."""

import json
import math
from pathlib import Path
from threading import Event, Lock, Thread
import time

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from xlerobot_perception_interfaces.srv import SetVocabulary
from xlerobot_perception_interfaces.msg import InstanceMaskArray

from .processing import LatestFrame, instance_masks_from_arrays, messages_from_arrays
from .vocabulary import DEFAULT_CLASSES, PromptModel, Vocabulary, validate_classes


def load_model(path, encoder_path, device, image_size, names):
    model = PromptModel(path, encoder_path, device)
    model.set_classes(names)
    # Finish model setup before accepting images.
    model.predict(np.zeros((image_size, image_size, 3), dtype=np.uint8),
                  device=device, imgsz=image_size, retina_masks=True, verbose=False)
    return model


class YoloeNode(Node):
    def __init__(self, model=None, **kwargs):
        super().__init__("yoloe", **kwargs)

        def parameter(name, default):
            return self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)).value

        topic = parameter("image_topic", "/camera/color/image_raw")
        path = parameter("model", "yoloe-11s-seg.pt")
        encoder_path = parameter("text_encoder", "mobileclip_blt.ts")
        names = validate_classes(parameter("classes", DEFAULT_CLASSES))
        self.device = parameter("device", "0")
        rate = parameter("rate_hz", 5.0)
        self.image_size = parameter("image_size", 640)
        self.confidence = parameter("confidence", 0.25)
        self.iou = parameter("iou", 0.7)
        self.max_detections = parameter("max_detections", 100)
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError("rate_hz must be finite and positive")
        if self.image_size <= 0 or self.image_size % 32:
            raise ValueError("image_size must be a positive multiple of 32")
        if not (0 < self.confidence <= 1 and 0 < self.iou <= 1) or self.max_detections <= 0:
            raise ValueError("confidence/iou must be in (0, 1], max_detections must be positive")
        self.period = 1.0 / rate
        self.bridge = CvBridge()
        self.pending = LatestFrame()
        self.stop = Event()
        self.failure = None
        self.worker = None
        self.update_lock = Lock()
        self.pending_update = None
        self.model_name = Path(path).name
        self.vocabulary = Vocabulary()
        self.model = model if model is not None else load_model(
            path, encoder_path, self.device, self.image_size, names)
        if model is not None:
            self.model.set_classes(names)
        self.vocabulary.activate(names)
        self.state = self.snapshot()
        self.detections = self.create_publisher(Detection2DArray, "yoloe/detections", 1)
        self.semantic = self.create_publisher(Image, "yoloe/semantic_mask", 1)
        self.instances = self.create_publisher(InstanceMaskArray, "yoloe/instance_masks", 1)
        self.source = self.create_publisher(Image, "yoloe/source_image", 1)
        metadata_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.classes = self.create_publisher(String, "yoloe/classes", metadata_qos)
        self.publish_classes()
        # Awaiting a service response must not block the camera subscription.
        self.control_group = ReentrantCallbackGroup()
        self.control = self.create_service(
            SetVocabulary, "yoloe/set_vocabulary", self.set_vocabulary,
            callback_group=self.control_group)
        # Compatible with both reliable and best-effort camera publishers.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(Image, topic, self.pending.put, qos)
        self.worker = Thread(target=self.run, name="yoloe-inference", daemon=True)
        self.worker.start()
        self.get_logger().info(f"YOLOE ready: {topic}, up to {rate:g} Hz, device={self.device}; "
                               f"targets={names}")

    def snapshot(self):
        return (self.vocabulary.revision, tuple(self.vocabulary.active),
                tuple(self.vocabulary.class_ids))

    def publish_classes(self):
        self.classes.publish(String(data=json.dumps({
            "model": self.model_name,
            "revision": self.vocabulary.revision,
            "labels": self.vocabulary.labels,
            "active_classes": self.vocabulary.active,
            "active_class_ids": self.vocabulary.class_ids,
            "overlap_policy": "highest_detection_confidence",
        })))

    def response(self, success, message):
        revision, names, ids = self.state
        return SetVocabulary.Response(success=success, message=message, revision=revision,
                                      classes=list(names), class_ids=list(ids))

    async def set_vocabulary(self, request, _response):
        try:
            names = validate_classes(request.classes)
        except ValueError as error:
            return self.response(False, str(error))
        with self.update_lock:
            if self.stop.is_set():
                return self.response(False, "Detector is stopping")
            if self.pending_update is not None:
                return self.response(False, "A vocabulary update is already in progress; retry after its response")
            future = Future(executor=self.executor)
            self.pending_update = (names, future)
        return await future

    def apply_update(self):
        with self.update_lock:
            update = self.pending_update
        if update is None:
            return
        names, future = update
        try:
            self.vocabulary.check_capacity(names)
            self.model.set_classes(names)
            self.vocabulary.activate(names)
            self.state = self.snapshot()
            self.publish_classes()
            response = self.response(True, "Vocabulary active")
            self.get_logger().info(f"Vocabulary revision {response.revision}: {names}")
        except Exception as error:
            response = self.response(False, f"Vocabulary update failed: {error}")
            self.get_logger().error(response.message)
        with self.update_lock:
            self.pending_update = None
        future.set_result(response)

    def run(self):
        next_start = 0.0
        try:
            while not self.stop.wait(max(0.0, next_start - time.monotonic())):
                self.apply_update()
                message = self.pending.take(timeout=.05)
                if message is None:
                    continue
                next_start = time.monotonic() + self.period
                try:
                    image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                except (ValueError, RuntimeError, CvBridgeError) as error:
                    self.get_logger().error(f"Dropping invalid camera image: {error}", throttle_duration_sec=5)
                    continue
                result = self.model.predict(
                    image, device=self.device, imgsz=self.image_size,
                    conf=self.confidence, iou=self.iou, max_det=self.max_detections,
                    retina_masks=True, verbose=False)[0].cpu().numpy()
                masks = None if result.masks is None else result.masks.data
                detections, labels = messages_from_arrays(
                    message.header, image.shape[:2], result.boxes.xyxy,
                    result.boxes.conf, result.boxes.cls, masks,
                    class_ids=self.vocabulary.class_ids)
                semantic = self.bridge.cv2_to_imgmsg(labels, encoding="16UC1")
                semantic.header = message.header
                if self.stop.is_set() or not self.context.ok():
                    break
                self.detections.publish(detections)
                self.semantic.publish(semantic)
                if self.instances.get_subscription_count():
                    self.instances.publish(instance_masks_from_arrays(
                        message.header, image.shape[:2], result.boxes.conf, result.boxes.cls,
                        masks, self.vocabulary.active, self.vocabulary.revision))
                if self.source.get_subscription_count():
                    self.source.publish(message)
        except Exception as error:
            if not self.stop.is_set() and self.context.ok():
                self.failure = error
                self.get_logger().fatal(f"YOLOE inference failed: {error}")
                self.context.try_shutdown()

    def destroy_node(self):
        self.stop.set()
        self.pending.close()
        if self.worker is not None:
            self.worker.join()
        with self.update_lock:
            if self.pending_update is not None:
                self.pending_update[1].set_result(self.response(False, "Detector stopped"))
                self.pending_update = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    if node is not None and node.failure is not None:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
