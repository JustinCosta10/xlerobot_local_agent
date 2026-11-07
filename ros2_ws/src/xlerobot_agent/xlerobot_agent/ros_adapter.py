"""Read confirmed objects and control vocabulary using the existing ROS interfaces."""

import json
import math
from threading import Lock, Thread
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from xlerobot_perception_interfaces.msg import TrackedObjectArray
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
    def __init__(self, namespace="xlerobot", use_sim_time=True, service_timeout_s=5., *, context=None,
                 enable_navigation=False):
        if enable_navigation and not use_sim_time:
            raise ValueError("Agent navigation is currently simulation-only.")
        self.navigation_enabled = enable_navigation
        self.navigation = None
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
        self.session = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, "yoloe/classes", self._metadata, latched)
        self.node.create_subscription(TrackedObjectArray, "object_map/objects", self._snapshot, latched)
        self.vocabulary_client = self.node.create_client(SetVocabulary, "yoloe/set_vocabulary")
        self.query_client = self.node.create_client(QueryObjects, "object_map/query")
        if enable_navigation:
            from .navigation import NavigationClient
            self.navigation = NavigationClient(self.node)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = Thread(target=self.executor.spin, name="agent-ros", daemon=True)
        self.thread.start()

    def _metadata(self, message):
        try:
            data = json.loads(message.data)
            value = {"model": data["model"], "revision": int(data["revision"]),
                     "active_classes": list(data["active_classes"]),
                     "active_class_ids": list(data["active_class_ids"])}
        except (ValueError, KeyError, TypeError):
            return
        with self.lock:
            self.metadata = value

    def _set_session(self, session):
        with self.lock:
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
                    objects="confirmed_only", motion_tools=self.navigation_enabled,
                    navigation=self.navigation.status() if self.navigation else None)

    def get_navigation_state(self):
        return self.navigation.status()

    def navigate(self, **kwargs):
        return self.navigation.navigate(**kwargs)

    def cancel_navigation(self):
        return self.navigation.cancel() if self.navigation else {"success": True, "code": "DISABLED"}

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
        return dict(success=response.success, message=response.message,
                    mechanism="runtime_text_prompts", classes=list(response.classes),
                    class_ids=list(response.class_ids), revision=response.revision,
                    code="VOCABULARY_ACTIVE" if response.success else "VOCABULARY_REJECTED")

    def query(self, label="", object_id="", include_stale=True, limit=100):
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

    def observe(self):
        # The object map is the source of known objects. Age and vocabulary do
        # not invalidate stored evidence; the mapper owns map-session changes.
        result = self.query(include_stale=True, limit=200)
        result.update(code="OBJECT_MAP", evidence="stored_confirmed_objects",
                      message=("Stored confirmed objects and their last known locations."
                               if result["objects"] else
                               "No confirmed objects stored in the current map session."))
        return result

    def close(self):
        self.cancel_navigation()
        self.executor.shutdown(timeout_sec=2.)
        self.thread.join(timeout=2.)
        self.node.destroy_node()
        if self.owns_context:
            self.context.try_shutdown()
