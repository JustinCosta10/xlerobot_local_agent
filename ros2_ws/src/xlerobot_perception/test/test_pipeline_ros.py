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
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from xlerobot_perception_interfaces.srv import SetVocabulary

from xlerobot_perception.detector import YoloeNode
from xlerobot_perception.visualizer import YoloeVisualizer


class SlowModel:
    def __init__(self):
        self.entered = Event()
        self.release = Event()
        self.calls = []
        self.predicting = False

    def set_classes(self, names):
        assert not self.predicting, "Vocabulary changed while prediction was running"
        if names == ["error"]:
            raise RuntimeError("Simulated text encoder failure")
        self.names = list(names)

    def predict(self, image, **_kwargs):
        self.predicting = True
        self.calls.append((int(image[0, 0, 0]), time.monotonic()))
        if len(self.calls) == 1:
            self.entered.set()
            assert self.release.wait(5)
        detected = image.any()
        boxes = SimpleNamespace(
            xyxy=np.array([[0, 0, 8, 4]]) if detected else np.empty((0, 4)),
            conf=np.array([.8]) if detected else np.array([]),
            cls=np.array([0]) if detected else np.array([]))
        result = SimpleNamespace(boxes=boxes, masks=SimpleNamespace(data=np.ones((1, 4, 8))) if detected else None)
        result.cpu = lambda: result
        result.numpy = lambda: result
        self.predicting = False
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
                         parameter_overrides=[Parameter("classes", value=["cup"]),
                                              Parameter("image_topic", value="/test/camera")])
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
        assert (bridge.imgmsg_to_cv2(received["mask"][-1]) == 1).all()
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


def test_service_updates_between_frames_and_preserves_ids_and_old_vocabulary_on_error():
    context = Context()
    rclpy.init(context=context, domain_id=85)
    executor = SingleThreadedExecutor(context=context)
    model = SlowModel()
    detector = YoloeNode(model=model, context=context, namespace="vocab_test")
    peer = Node("peer", context=context, namespace="vocab_test")
    client = peer.create_client(SetVocabulary, "yoloe/set_vocabulary")
    masks, metadata = [], []
    peer.create_subscription(Image, "yoloe/semantic_mask", masks.append, 10)
    peer.create_subscription(String, "yoloe/classes", metadata.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher = peer.create_publisher(Image, "/camera/color/image_raw", 1)
    for node in (detector, peer):
        executor.add_node(node)
    spin = Thread(target=executor.spin, daemon=True)
    spin.start()
    bridge = CvBridge()

    def send(value):
        message = bridge.cv2_to_imgmsg(np.full((4, 8, 3), value, dtype=np.uint8), "bgr8")
        message.header.stamp.sec = value
        publisher.publish(message)

    def call(names):
        future = client.call_async(SetVocabulary.Request(classes=names))
        wait_for(future.done)
        return future.result()

    try:
        wait_for(lambda: client.service_is_ready() and metadata and publisher.get_subscription_count())
        send(1)
        assert model.entered.wait(3)
        update = client.call_async(SetVocabulary.Request(classes=[" Mustard Bottle ", "MUG"]))
        wait_for(lambda: detector.pending_update is not None)
        assert not update.done()  # Cannot acknowledge an update until inference finishes.
        busy = call(["chair"])
        assert not busy.success and "in progress" in busy.message
        assert not call([]).success
        assert not call(["cup", " Cup "]).success
        for value in range(2, 6):
            send(value)
            time.sleep(.01)
        wait_for(lambda: detector.pending.frame is not None and detector.pending.frame.header.stamp.sec == 5)
        model.release.set()
        wait_for(update.done)
        response = update.result()
        assert response.success and response.revision == 2
        assert response.classes == ["mustard bottle", "mug"]
        assert list(response.class_ids) == [5, 1]
        wait_for(lambda: len(masks) == 2)
        assert [message.header.stamp.sec for message in masks] == [1, 5]
        assert (bridge.imgmsg_to_cv2(masks[0]) == 1).all()
        assert (bridge.imgmsg_to_cv2(masks[1]) == 5).all()
        assert [call[0] for call in model.calls] == [1, 5]
        # Services still apply when no images or /clock updates arrive.
        response = call(["mug", "mustard bottle", "book"])
        assert response.success and list(response.class_ids) == [1, 5, 8]
        assert response.revision == 3
        repeated = call(response.classes)
        assert repeated.success and repeated.revision == 3
        failed = call(["error"])
        assert not failed.success and failed.revision == 3
        assert failed.classes == response.classes == model.names
        assert list(failed.class_ids) == [1, 5, 8]
        import json
        wait_for(lambda: json.loads(metadata[-1].data)["revision"] == 3)
        labels = json.loads(metadata[-1].data)["labels"]
        assert labels["1"] == "mug" and labels["5"] == "mustard bottle" and labels["8"] == "book"
    finally:
        model.release.set()
        executor.shutdown()
        spin.join(timeout=3)
        detector.destroy_node()
        peer.destroy_node()
        context.try_shutdown()
