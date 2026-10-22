"""Exercise real DDS, timestamp alignment, and newest-frame behavior without a GPU."""

from threading import Event, Thread
import time
from types import SimpleNamespace

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from xlerobot_perception.detector import YoloeNode
from xlerobot_perception.visualizer import YoloeVisualizer


class SlowModel:
    names = {7: "cup"}

    def __init__(self):
        self.entered = Event()
        self.release = Event()
        self.calls = []

    def predict(self, image, **_kwargs):
        self.calls.append((int(image[0, 0, 0]), time.monotonic()))
        if len(self.calls) == 1:
            self.entered.set()
            assert self.release.wait(5)
        detected = image.any()
        boxes = SimpleNamespace(
            xyxy=np.array([[0, 0, 8, 4]]) if detected else np.empty((0, 4)),
            conf=np.array([.8]) if detected else np.array([]),
            cls=np.array([7]) if detected else np.array([]))
        result = SimpleNamespace(boxes=boxes, masks=SimpleNamespace(data=np.ones((1, 4, 8))) if detected else None)
        result.cpu = lambda: result
        result.numpy = lambda: result
        return [result]


def wait_for(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, "Timed out waiting for ROS pipeline"
        time.sleep(.01)


def test_slow_inference_keeps_latest_and_publishes_aligned_outputs():
    context = Context()
    rclpy.init(context=context, domain_id=83)
    executor = SingleThreadedExecutor(context=context)
    model = SlowModel()
    detector = YoloeNode(model=model, context=context, namespace="test",
                         cli_args=["--ros-args", "-p", "image_topic:=/test/camera"])
    viewer = YoloeVisualizer(context=context, namespace="test")
    probe = Node("probe", context=context, namespace="test")
    bridge = CvBridge()
    received = {name: [] for name in ("detections", "mask", "raw", "compressed", "classes")}
    subscriptions = [probe.create_subscription(msg_type, topic, received[name].append, qos)
                     for name, msg_type, topic, qos in (
                         ("detections", Detection2DArray, "yoloe/detections", 10),
                         ("mask", Image, "yoloe/semantic_mask", 10),
                         ("raw", Image, "yoloe/segmentation/image_raw", 10),
                         ("compressed", CompressedImage, "yoloe/segmentation/image_raw/compressed", 10),
                         ("classes", String, "yoloe/classes", QoSProfile(
                             depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))]
    publisher = probe.create_publisher(Image, "camera", 1)
    for node in (detector, viewer, probe):
        executor.add_node(node)
    spin = Thread(target=executor.spin, daemon=True)
    spin.start()

    def publish(value):
        message = bridge.cv2_to_imgmsg(np.full((4, 8, 3), value, dtype=np.uint8), "rgb8")
        message.header.frame_id = "camera_optical"
        message.header.stamp.sec = value + 1
        publisher.publish(message)

    try:
        wait_for(lambda: publisher.get_subscription_count() and received["classes"])
        wait_for(lambda: viewer.raw.get_subscription_count() and detector.source.get_subscription_count())
        publish(1)
        assert model.entered.wait(3)
        for value in range(2, 21):
            publish(value)
            time.sleep(.01)
        wait_for(lambda: detector.pending.frame is not None and detector.pending.frame.header.stamp.sec == 21)
        model.release.set()
        wait_for(lambda: len(received["compressed"]) >= 2)
        assert [call[0] for call in model.calls] == [1, 20]
        for name in ("detections", "mask", "raw", "compressed"):
            assert [m.header.stamp.sec for m in received[name]] == [2, 21]
            assert all(m.header.frame_id == "camera_optical" for m in received[name])
        assert "cup" in received["classes"][0].data
        assert (bridge.imgmsg_to_cv2(received["mask"][-1]) == 8).all()
        publish(0)
        wait_for(lambda: len(received["compressed"]) >= 3)
        assert not received["detections"][-1].detections
        assert not bridge.imgmsg_to_cv2(received["mask"][-1]).any()
        time.sleep(.3)
        assert len(model.calls) == 3  # Never re-infer an old frame during a camera pause.
        assert all(b[1] - a[1] >= .19 for a, b in zip(model.calls, model.calls[1:]))
    finally:
        model.release.set()
        executor.shutdown()
        spin.join(timeout=3)
        detector.destroy_node()
        viewer.destroy_node()
        probe.destroy_node()
        context.try_shutdown()
