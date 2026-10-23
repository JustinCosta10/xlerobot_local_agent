"""Time-aligned instance masks and depth to a persistent, session-scoped object map."""

import math
import os
from pathlib import Path
import time

from builtin_interfaces.msg import Time as TimeMessage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from xlerobot_perception_interfaces.msg import InstanceMaskArray, TrackedObject, TrackedObjectArray
from xlerobot_perception_interfaces.srv import QueryObjects

from .object_geometry import Observation, camera_parameters, depth_in_meters, encode_crop, project_mask
from .object_memory import ObjectMemory
from .object_sync import StampCache


def ros_stamp(nanoseconds):
    sec, nanosec = divmod(int(nanoseconds), 10**9)
    return TimeMessage(sec=sec, nanosec=nanosec)


def object_message(record):
    result = TrackedObject(object_id=record["object_id"], session_id=record["session_id"],
                           label=record["label"], confidence=float(record["confidence"]),
                           observation_count=record["observation_count"], stale=record["stale"])
    result.header.frame_id = record["frame_id"]
    result.header.stamp = ros_stamp(record["last_seen_ns"])
    result.first_seen = ros_stamp(record["first_seen_ns"])
    result.position.x, result.position.y, result.position.z = record["x"], record["y"], record["z"]
    meta = record["metadata"]
    result.extent.x, result.extent.y, result.extent.z = meta["extent"]
    result.surface_spread.x, result.surface_spread.y, result.surface_spread.z = meta["spread"]
    result.valid_depth_fraction, result.point_count = float(meta["valid_fraction"]), meta["point_count"]
    if record.get("crop_jpeg"):
        result.crop.header.frame_id = record["crop_frame"]
        result.crop.header.stamp = ros_stamp(record["crop_stamp_ns"])
        result.crop.format = "jpeg"
        result.crop.data = record["crop_jpeg"]
    return result


class ObjectMapper(Node):
    def __init__(self, **kwargs):
        super().__init__("object_mapper", **kwargs)

        def parameter(name, default):
            return self.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value

        self.frame = parameter("map_frame", "map")
        self.origin_frame = parameter("map_origin_frame", "vslam_map")
        self.wait = float(parameter("max_wait_s", 2.0))
        tolerance = float(parameter("sync_tolerance_s", .005))
        capacity = int(parameter("depth_cache_frames", 90))
        self.min_confidence = float(parameter("min_confidence", .35))
        self.projection = dict(
            erosion=int(parameter("mask_erosion_pixels", 1)),
            min_pixels=int(parameter("min_depth_pixels", 20)),
            min_fraction=float(parameter("min_valid_depth_fraction", .5)),
            min_depth=float(parameter("min_depth_m", .1)), max_depth=float(parameter("max_depth_m", 8.0)))
        minimum = int(parameter("confirmation_observations", 3))
        association = float(parameter("association_distance_m", .08))
        window = float(parameter("confirmation_window_s", 2.0))
        stale = float(parameter("stale_after_s", 10.0))
        history = float(parameter("history_period_s", 1.0))
        numbers = [self.wait, tolerance, self.min_confidence, association, window, stale, history,
                   self.projection["min_fraction"], self.projection["min_depth"], self.projection["max_depth"]]
        if (not all(math.isfinite(value) for value in numbers) or self.wait <= 0 or tolerance < 0
                or capacity < 1 or minimum < 1 or min(association, window, stale) <= 0 or history < 0
                or not 0 <= self.min_confidence <= 1 or not 0 < self.projection["min_fraction"] <= 1
                or self.projection["erosion"] < 0 or self.projection["min_pixels"] < 1
                or not 0 < self.projection["min_depth"] < self.projection["max_depth"] or not self.frame):
            raise ValueError("Invalid object mapping thresholds or cache configuration")
        self.tolerance = round(tolerance * 1e9)
        default_path = os.environ.get("XLEROBOT_OBJECT_DATABASE", str(Path.home() / ".ros/xlerobot/objects.sqlite3"))
        database = parameter("database_path", default_path)
        self.memory = ObjectMemory(database, self.frame, minimum, association, window, stale, history)
        self.memory.new_session(self.get_clock().now().nanoseconds, "mapper startup")
        self.bridge = CvBridge()
        self.instances = StampCache(15, self.wait)
        self.sources = StampCache(15, self.wait)
        self.depths = StampCache(capacity, self.wait)
        self.infos = StampCache(capacity, self.wait)
        self.caches = (self.instances, self.sources, self.depths, self.infos)
        self.buffer = Buffer(cache_time=Duration(seconds=max(10.0, self.wait + 1)), node=self)
        self.listener = TransformListener(self.buffer, self)
        self.reset_requested = None
        self.origin_token = None
        self.last_publish = 0.0
        self.failure = None
        self.jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True),
            post_callback=lambda _: self.request_reset("ROS clock reset"))
        camera_qos = QoSProfile(depth=capacity, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw_float"),
                                 self.depths.put, camera_qos)
        self.create_subscription(CameraInfo, parameter("camera_info_topic", "/camera/color/camera_info"),
                                 self.infos.put, camera_qos)
        self.create_subscription(InstanceMaskArray, "yoloe/instance_masks", self.instances.put, 5)
        self.create_subscription(Image, "yoloe/source_image", self.sources.put, 5)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.objects = self.create_publisher(TrackedObjectArray, "object_map/objects", latched)
        self.markers = self.create_publisher(MarkerArray, "object_map/markers", latched)
        self.create_subscription(TFMessage, "/tf_static", self.on_static_tf,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(QueryObjects, "object_map/query", self.query)
        self.create_service(Trigger, "object_map/new_session", self.new_session)
        # A steady timer allows pending work to expire while simulation is paused.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05, self.update, clock=self.steady_clock)
        self.get_logger().info(f"Object database: {self.memory.path}; session={self.memory.session_id}; "
                               f"confirm after {minimum} observations in {self.frame}")

    def request_reset(self, reason):
        self.reset_requested = reason

    def on_static_tf(self, message):
        if not self.origin_frame:
            return
        for transform in message.transforms:
            if transform.header.frame_id == self.frame and transform.child_frame_id == self.origin_frame:
                token = str(transform)
                if self.origin_token is not None and token != self.origin_token:
                    self.request_reset("navigation map origin changed")
                self.origin_token = token

    def reset_session(self, reason):
        self.memory.new_session(self.get_clock().now().nanoseconds, reason)
        for cache in self.caches:
            cache.clear()
        self.reset_requested = None
        self.last_publish = 0
        self.get_logger().info(f"New object session {self.memory.session_id}: {reason}; previous records retained")

    def new_session(self, _request, response):
        self.reset_session("manual session reset")
        self.publish_snapshot()
        response.success, response.message = True, self.memory.session_id
        return response

    def query(self, request, response):
        if self.reset_requested:
            self.reset_session(self.reset_requested)
        records = self.memory.query(
            self.get_clock().now().nanoseconds, label=request.label, object_id=request.object_id,
            session_id=request.session_id, include_stale=request.include_stale,
            include_images=request.include_images, limit=request.limit)
        response.success, response.message = True, f"{len(records)} confirmed objects"
        response.current_session_id, response.database_path = self.memory.session_id, self.memory.path
        response.objects = [object_message(record) for record in records]
        return response

    def update(self):
        try:
            if self.reset_requested:
                self.reset_session(self.reset_requested)
            for cache in (self.instances, self.sources, self.depths):
                cache.expire()
            for stamp in sorted(self.instances.frames, reverse=True):
                if stamp <= self.memory.last_stamp_ns or stamp == 0:
                    self.instances.discard_through(stamp)
                    continue
                masks = self.instances.get(stamp)
                source = self.sources.get(stamp)
                depth = self.depths.nearest(stamp, self.tolerance)
                info = self.infos.calibration(stamp, self.tolerance)
                if source is None or depth is None or info is None:
                    self.get_logger().warning("Waiting for RGB/depth/CameraInfo matching instance capture time",
                                              throttle_duration_sec=5)
                    continue
                try:
                    transform = self.buffer.lookup_transform(
                        self.frame, masks.header.frame_id, Time.from_msg(masks.header.stamp)).transform
                except TransformException:
                    self.get_logger().warning("Waiting for camera-to-map TF at image capture time",
                                              throttle_duration_sec=5)
                    continue
                try:
                    observations = self.observations(masks, source, depth, info, transform)
                except (ValueError, TypeError, CvBridgeError) as error:
                    self.get_logger().error(f"Dropping invalid object mapping frame: {error}", throttle_duration_sec=5)
                    self.instances.discard_through(stamp)
                    continue
                self.memory.process(stamp, observations)
                self.instances.discard_through(stamp)
                self.sources.discard_through(stamp)
                self.publish_snapshot()
                break  # At most one (newest ready) frame per tick.
            if time.monotonic() - self.last_publish >= 1.0:
                self.publish_snapshot()
        except Exception as error:
            self.failure = error
            self.get_logger().fatal(f"Object mapping stopped: {error}")
            self.context.try_shutdown()

    def observations(self, masks, source, depth_message, info, transform):
        frame = masks.header.frame_id
        if not frame or any(message.header.frame_id != frame for message in (source, depth_message, info)):
            raise ValueError("RGB, aligned depth, masks, and CameraInfo must use the same optical frame")
        shape = masks.image_height, masks.image_width
        image = self.bridge.imgmsg_to_cv2(source, desired_encoding="bgr8")
        depth = depth_in_meters(self.bridge.imgmsg_to_cv2(depth_message, "passthrough"), depth_message.encoding)
        if image.shape[:2] != shape or depth.shape != shape:
            raise ValueError("RGB, registered depth, and instance masks must use the same pixel grid")
        k, d = camera_parameters(info, shape)
        q, p = transform.rotation, transform.translation
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        matrix[:3, 3] = [p.x, p.y, p.z]
        claimed = np.zeros(shape, dtype=bool)
        result = []
        for instance in sorted(masks.instances, key=lambda item: item.confidence, reverse=True):
            if not math.isfinite(instance.confidence) or not self.min_confidence <= instance.confidence <= 1:
                continue
            label = " ".join(instance.class_name.split()).casefold()
            if not label or instance.mask.header != masks.header or instance.mask.encoding != "mono8":
                raise ValueError("Invalid instance label, mask header, or mask encoding")
            mask = self.bridge.imgmsg_to_cv2(instance.mask, "mono8")
            x, y = instance.x_offset, instance.y_offset
            h, w = mask.shape
            if not h or not w or y + h > shape[0] or x + w > shape[1]:
                raise ValueError("Instance mask crop is outside the original RGB image")
            ownership = claimed[y:y + h, x:x + w]
            selected = (mask != 0) & ~ownership
            ownership |= mask != 0  # Higher confidence owns overlapping depth pixels.
            projected = project_mask(selected, (x, y), depth, k, d, matrix, **self.projection)
            if projected is None:
                continue
            position, extent, spread, fraction, count = projected
            crop = encode_crop(image, (x, y), mask.shape)
            result.append(Observation(label, position, extent, spread, float(instance.confidence),
                                      fraction, count, crop, frame))
        return result

    def publish_snapshot(self):
        now = self.get_clock().now().nanoseconds
        records = self.memory.query(now, include_stale=True, limit=None)
        message = TrackedObjectArray(session_id=self.memory.session_id)
        message.header.frame_id, message.header.stamp = self.frame, ros_stamp(now)
        message.objects = [object_message(record) for record in records]
        self.objects.publish(message)
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for index, item in enumerate(message.objects):
            point = Marker(header=message.header, ns="object_positions", id=index, type=Marker.SPHERE)
            point.pose.position, point.pose.orientation.w = item.position, 1.0
            point.scale.x = point.scale.y = point.scale.z = .035
            point.color.r, point.color.g, point.color.b, point.color.a = (
                (.5, .5, .5, .5) if item.stale else (.1, 1.0, .3, 1.0))
            point.lifetime.sec = 3
            text = Marker(header=message.header, ns="object_labels", id=index, type=Marker.TEXT_VIEW_FACING)
            text.pose.position.x, text.pose.position.y = item.position.x, item.position.y
            text.pose.position.z, text.pose.orientation.w = item.position.z + .08, 1.0
            text.scale.z, text.color, text.lifetime = .035, point.color, point.lifetime
            text.text = f"{item.label} [{item.object_id[:8]}]" + (" (stale)" if item.stale else "")
            markers.markers.extend((point, text))
        self.markers.publish(markers)
        self.last_publish = time.monotonic()

    def destroy_node(self):
        self.memory.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObjectMapper()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    if node is not None and node.failure is not None:
        raise SystemExit(1)
