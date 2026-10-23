"""Real DDS/TF/SQLite integration, with controlled images instead of a detector/GPU."""

from threading import Thread
import time

from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray
from xlerobot_perception_interfaces.msg import InstanceMaskArray
from xlerobot_perception_interfaces.srv import QueryObjects

from xlerobot_perception.object_mapper import ObjectMapper, ros_stamp
from xlerobot_perception.processing import instance_masks_from_arrays


def wait_for(condition, timeout=5):
    deadline = time.monotonic() + timeout
    while not condition():
        assert time.monotonic() < deadline, "Object mapping ROS test timed out"
        time.sleep(.01)


def test_delayed_masks_use_matching_depth_and_historical_tf_then_query_saved_objects(tmp_path):
    context = Context()
    rclpy.init(context=context, domain_id=86)
    executor = SingleThreadedExecutor(context=context)
    mapper = ObjectMapper(context=context, namespace="object_test", parameter_overrides=[
        Parameter("database_path", value=str(tmp_path / "objects.sqlite3")),
        Parameter("map_frame", value="test_map"), Parameter("map_origin_frame", value=""),
        Parameter("depth_topic", value="/object_test/depth"),
        Parameter("camera_info_topic", value="/object_test/info")])
    peer = Node("probe", context=context, namespace="object_test")
    pubs = {key: peer.create_publisher(kind, name, 10) for key, kind, name in (
        ("depth", Image, "depth"), ("info", CameraInfo, "info"),
        ("source", Image, "yoloe/source_image"), ("masks", InstanceMaskArray, "yoloe/instance_masks"))}
    markers = []
    peer.create_subscription(MarkerArray, "object_map/markers", markers.append, 10)
    query = peer.create_client(QueryObjects, "object_map/query")
    reset = peer.create_client(Trigger, "object_map/new_session")
    bridge = CvBridge()
    base = peer.get_clock().now().nanoseconds - 1_000_000_000
    stamps = [base + index * 200_000_000 for index in range(4)]
    for stamp, x in [(stamp, 1.) for stamp in stamps] + [(base + 1_500_000_000, 100.)]:
        transform = TransformStamped(header=Header(frame_id="test_map", stamp=ros_stamp(stamp)),
                                     child_frame_id="optical")
        transform.transform.translation.x, transform.transform.translation.z = x, .1
        transform.transform.rotation.w = 1.
        mapper.buffer.set_transform(transform, "test")
    for node in (mapper, peer):
        executor.add_node(node)
    spin = Thread(target=executor.spin, daemon=True)
    spin.start()

    def call(client, request):
        future = client.call_async(request)
        wait_for(future.done)
        return future.result()

    try:
        wait_for(lambda: all(pub.get_subscription_count() for pub in pubs.values()) and query.service_is_ready())
        mask_array = np.zeros((2, 40, 80), dtype=np.float32)
        mask_array[0, 10:20, 10:18] = 1
        mask_array[1, 10:20, 50:58] = 1
        for index, stamp in enumerate(stamps):
            header = Header(frame_id="optical", stamp=ros_stamp(stamp))
            info = CameraInfo(header=header, height=40, width=80,
                              k=[200., 0., 40., 0., 200., 20., 0., 0., 1.])
            pubs["info"].publish(info)
            source = bridge.cv2_to_imgmsg(np.full((40, 80, 3), (200, 80, 30), np.uint8), "bgr8")
            source.header = header
            pubs["source"].publish(source)
            instance_masks = mask_array[::-1] if index == 3 else mask_array
            pubs["masks"].publish(instance_masks_from_arrays(
                header, (40, 80), [.8, .9], [0, 0], instance_masks, ["cube"], 1))
            if index == 0:
                wrong = bridge.cv2_to_imgmsg(np.full((40, 80), 4., np.float32), "32FC1")
                wrong.header = Header(frame_id="optical", stamp=ros_stamp(stamp + 100_000_000))
                pubs["depth"].publish(wrong)
                time.sleep(.15)
                assert mapper.memory.last_stamp_ns == -1  # No approximate/latest-depth shortcut.
            depth = bridge.cv2_to_imgmsg(np.ones((40, 80), np.float32), "32FC1")
            depth.header = header
            pubs["depth"].publish(depth)
            wait_for(lambda: mapper.memory.last_stamp_ns == stamp)
            result = call(query, QueryObjects.Request(label=" CUBE ", include_images=True))
            assert result.success
            if index < 2:
                assert not result.objects
            else:
                assert len(result.objects) == 2
                objects = sorted(result.objects, key=lambda item: item.position.x)
                np.testing.assert_allclose([item.position.x for item in objects], [.8675, 1.0675], atol=1e-6)
                np.testing.assert_allclose([item.position.z for item in objects], [1.1, 1.1], atol=1e-6)
                assert all(item.observation_count == index + 1 and item.crop.data for item in objects)
                assert all(item.header.stamp == header.stamp for item in objects)
                if index == 2:
                    ids = [item.object_id for item in objects]
                else:
                    assert [item.object_id for item in objects] == ids
        wait_for(lambda: any(len(message.markers) == 5 for message in markers))
        previous = result.current_session_id
        new_session = call(reset, Trigger.Request())
        assert new_session.success and new_session.message != previous
        assert not call(query, QueryObjects.Request(include_stale=True)).objects
        historical = call(query, QueryObjects.Request(session_id=previous, include_stale=True, include_images=True))
        assert len(historical.objects) == 2 and all(item.stale for item in historical.objects)
        assert all(item.crop.data for item in historical.objects)
        assert mapper.failure is None
    finally:
        executor.shutdown()
        spin.join(timeout=3)
        mapper.destroy_node()
        peer.destroy_node()
        context.try_shutdown()


def test_clock_and_navigation_origin_resets_start_new_database_sessions(tmp_path):
    context = Context()
    rclpy.init(context=context, domain_id=87)
    executor = SingleThreadedExecutor(context=context)
    mapper = ObjectMapper(context=context, namespace="clock_test", parameter_overrides=[
        Parameter("database_path", value=str(tmp_path / "objects.sqlite3")),
        Parameter("use_sim_time", value=True)])
    peer = Node("clock_source", context=context)
    clock = peer.create_publisher(Clock, "/clock", 10)
    origins = peer.create_publisher(TFMessage, "/tf_static", QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for node in (mapper, peer):
        executor.add_node(node)
    spin = Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        wait_for(clock.get_subscription_count)
        clock.publish(Clock(clock=ros_stamp(20_000_000_000)))
        wait_for(lambda: mapper.get_clock().now().nanoseconds == 20_000_000_000)
        time.sleep(.1)
        previous = mapper.memory.session_id
        clock.publish(Clock(clock=ros_stamp(5_000_000_000)))
        wait_for(lambda: mapper.memory.session_id != previous)
        assert mapper.memory.last_stamp_ns == -1
        previous = mapper.memory.session_id
        origin = TransformStamped(header=Header(frame_id="map", stamp=ros_stamp(2_000_000_000)),
                                  child_frame_id="vslam_map")
        origin.transform.rotation.w = 1.
        origins.publish(TFMessage(transforms=[origin]))
        wait_for(lambda: mapper.origin_token is not None)
        assert mapper.memory.session_id == previous  # First origin establishes the session reference.
        origin.header.stamp = ros_stamp(3_000_000_000)
        origins.publish(TFMessage(transforms=[origin]))
        wait_for(lambda: mapper.memory.session_id != previous)
        assert mapper.failure is None
    finally:
        executor.shutdown()
        spin.join(timeout=3)
        mapper.destroy_node()
        peer.destroy_node()
        context.try_shutdown()
