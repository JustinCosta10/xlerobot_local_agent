"""Read confirmed objects and control vocabulary using the existing ROS interfaces."""

from collections import deque
import json
import math
from threading import Lock, Thread
import time

import rclpy
from rclpy.clock import JumpThreshold
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from xlerobot_perception_interfaces.msg import InstanceMaskArray, TrackedObjectArray
from xlerobot_perception_interfaces.srv import QueryObjects, SetVocabulary

from .tools import ToolError


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def object_record(item, now_ns):
    observed = stamp_ns(item.header.stamp)
    position = [item.position.x, item.position.y, item.position.z]
    if not all(math.isfinite(v) for v in position):
        raise ValueError("Invalid position")
    return dict(object_id=item.object_id, session_id=item.session_id, label=item.label,
                tracking_state="confirmed", frame_id=item.header.frame_id, position_m=position,
                observation_stamp_ns=observed, observation_age_s=(now_ns - observed) / 1e9,
                stale=item.stale, observation_count=item.observation_count,
                confidence=float(item.confidence), valid_depth_fraction=float(item.valid_depth_fraction),
                point_count=item.point_count)


class RosAdapter:
    def __init__(self, namespace="xlerobot", use_sim_time=True, service_timeout_s=5., *, context=None):
        self.owns_context = context is None
        self.context = context or Context()
        if self.owns_context:
            # CLI owns Ctrl+C. ROS callbacks continue spinning while the CLI waits on HTTP.
            rclpy.init(context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node("perception_agent", namespace=namespace, context=self.context,
                         parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)])
        self.timeout = service_timeout_s
        self.lock = Lock()
        self.metadata = None
        self.accepted_vocabulary = None
        self.session = None
        self.epoch = 0
        self.frames = deque(maxlen=128)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, "yoloe/classes", self._metadata, latched)
        self.node.create_subscription(InstanceMaskArray, "yoloe/instance_masks", self._frame, 5)
        self.node.create_subscription(TrackedObjectArray, "object_map/objects", self._snapshot, latched)
        self.vocabulary_client = self.node.create_client(SetVocabulary, "yoloe/set_vocabulary")
        self.query_client = self.node.create_client(QueryObjects, "object_map/query")
        self.jump = self.node.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True),
            post_callback=self._clock_jump)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = Thread(target=self.executor.spin, name="agent-ros", daemon=True)
        self.thread.start()

    def _clock_jump(self, _jump):
        with self.lock:
            self.epoch += 1
            self.frames.clear()

    def _metadata(self, message):
        try:
            data = json.loads(message.data)
            value = {"model": data["model"], "revision": int(data["revision"]),
                     "active_classes": list(data["active_classes"]),
                     "active_class_ids": list(data["active_class_ids"])}
        except (ValueError, KeyError, TypeError):
            return
        with self.lock:
            if value != self.metadata:
                self.epoch += 1
            self.metadata = value

    def _frame(self, message):
        # No masks, labels, per-image IDs, or tentative detections leave this callback.
        with self.lock:
            self.frames.append((time.monotonic(), stamp_ns(message.header.stamp),
                                message.vocabulary_revision))

    def _set_session(self, session):
        with self.lock:
            if self.session != session:
                self.epoch += 1
            self.session = session

    def _snapshot(self, message):
        self._set_session(message.session_id)

    def status(self):
        with self.lock:
            metadata = dict(self.metadata) if self.metadata else None
            session = self.session
        return dict(execution_mode="live_ros_perception", vocabulary=metadata, session_id=session,
                    query_available=self.query_client.service_is_ready(),
                    vocabulary_available=self.vocabulary_client.service_is_ready(),
                    objects="confirmed_only", motion_tools=False)

    def _call(self, client, request):
        deadline = time.monotonic() + self.timeout
        while not client.wait_for_service(timeout_sec=.05):
            if time.monotonic() >= deadline or not self.context.ok():
                raise ToolError("DEPENDENCY_UNAVAILABLE", "ROS service unavailable; check namespace and running perception.")
        future = client.call_async(request)
        try:
            while not future.done():
                if time.monotonic() >= deadline or not self.context.ok():
                    raise ToolError("TIMEOUT", "ROS service timed out. A sent vocabulary update may still apply.")
                time.sleep(.01)
            result = future.result()
            if result is None:
                raise ToolError("DEPENDENCY_UNAVAILABLE", "ROS service returned no result.")
            return result
        finally:
            if not future.done():
                client.remove_pending_request(future)
                future.cancel()

    def set_vocabulary(self, classes):
        response = self._call(self.vocabulary_client, SetVocabulary.Request(classes=classes))
        if response.success:
            with self.lock:
                self.accepted_vocabulary = (response.revision, list(response.classes))
        return dict(success=response.success, message=response.message,
                    mechanism="runtime_text_prompts", classes=list(response.classes),
                    class_ids=list(response.class_ids), revision=response.revision,
                    code="VOCABULARY_ACTIVE" if response.success else "VOCABULARY_REJECTED")

    def query(self, label="", object_id="", include_stale=False, limit=100):
        response = self._call(self.query_client, QueryObjects.Request(
            label=label, object_id=object_id, session_id="", include_stale=include_stale,
            include_images=False, limit=limit))
        if not response.success:
            raise ToolError("QUERY_FAILED", response.message)
        self._set_session(response.current_session_id)
        now = self.node.get_clock().now().nanoseconds
        objects, omitted = [], 0
        for item in response.objects:
            if item.session_id != response.current_session_id or (item.stale and not include_stale):
                omitted += 1
                continue
            try:
                objects.append(object_record(item, now))
            except ValueError:
                omitted += 1
        return dict(success=True, session_id=response.current_session_id, objects=objects,
                    count=len(objects), limit=limit, completeness="unknown" if
                    len(response.objects) >= limit or omitted else "complete",
                    omitted_invalid_records=omitted,
                    scope="confirmed records in current session matching this query")

    def observe(self, duration_s):
        initial = self.query(limit=1)  # Establish mapper availability and current session.
        deadline = time.monotonic() + self.timeout
        while True:
            with self.lock:
                metadata = self.metadata
                accepted = self.accepted_vocabulary
            if metadata is not None and (accepted is None or (
                    metadata["revision"] == accepted[0] and metadata["active_classes"] == accepted[1])):
                break
            if time.monotonic() >= deadline:
                raise ToolError("DEPENDENCY_UNAVAILABLE", "YOLOE metadata has not confirmed the last accepted vocabulary.")
            time.sleep(.02)
        start_ns = self.node.get_clock().now().nanoseconds
        if start_ns <= 0:
            raise ToolError("NO_FRESH_OBSERVATION", "ROS clock has not started.")
        start_wall = time.monotonic()
        with self.lock:
            epoch, metadata = self.epoch, dict(self.metadata)
        revision = metadata["revision"]
        previous_ns = start_ns
        while time.monotonic() - start_wall < duration_s:
            now = self.node.get_clock().now().nanoseconds
            with self.lock:
                changed = self.epoch != epoch
            if changed or now < previous_ns:
                raise ToolError("OBSERVATION_CHANGED", "Vocabulary, map session, or ROS time changed during observation.")
            previous_ns = now
            time.sleep(min(.02, max(0., duration_s - (time.monotonic() - start_wall))))
        end_ns = self.node.get_clock().now().nanoseconds
        result = self.query(limit=200)
        with self.lock:
            changed = self.epoch != epoch
            fresh_frames = {stamp for received, stamp, rev in self.frames
                            if received >= start_wall and start_ns < stamp <= end_ns and rev == revision}
        if changed or result["session_id"] != initial["session_id"] or end_ns < start_ns:
            raise ToolError("OBSERVATION_CHANGED", "Vocabulary, map session, or ROS time changed during observation.")
        objects = [item for item in result["objects"]
                   if item["label"] in metadata["active_classes"] and not item["stale"]
                   and start_ns < item["observation_stamp_ns"] <= end_ns] if fresh_frames else []
        return dict(success=bool(fresh_frames), code="OBSERVED" if fresh_frames else "NO_FRESH_OBSERVATION",
                    message=("Freshly observed confirmed objects." if objects else
                             "No freshly observed confirmed objects in this window." if fresh_frames else
                             "No fresh detector frames under the active vocabulary; scene state is unknown."),
                    objects=objects, count=len(objects), session_id=result["session_id"],
                    vocabulary_revision=revision, revision_scope="window metadata, not per-object provenance",
                    observation_start_ns=start_ns, observation_end_ns=end_ns,
                    perception_activity="fresh_detector_frames" if fresh_frames else "no_fresh_detector_frames",
                    completeness=result["completeness"], limit=result["limit"])

    def close(self):
        self.executor.shutdown(timeout_sec=2.)
        self.thread.join(timeout=2.)
        self.node.destroy_node()
        if self.owns_context:
            self.context.try_shutdown()
