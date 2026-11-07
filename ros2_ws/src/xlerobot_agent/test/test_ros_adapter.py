"""Real DDS bindings with fake perception on domain 87; no API, GPU, or movement."""

import json
from threading import Thread
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from xlerobot_perception_interfaces.msg import InstanceMask, InstanceMaskArray, TrackedObject, TrackedObjectArray
from xlerobot_perception_interfaces.srv import QueryObjects, SetVocabulary

from xlerobot_agent.ros_adapter import RosAdapter
from xlerobot_agent.agent import Agent
from xlerobot_agent.demo import ScriptedProvider
from xlerobot_agent.tools import ToolError, ToolRegistry


def wait_for(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, "DDS fixture did not become ready"
        time.sleep(.01)


class PerceptionPeer(Node):
    def __init__(self, context):
        super().__init__("fake_perception", namespace="agent_test", context=context)
        self.classes, self.revision, self.session = ["cube"], 1, "session-one"
        self.confirmed = True
        self.fresh = True
        self.enabled = True
        self.last_stamp = self.get_clock().now().to_msg()
        self.requests = []
        self.publish_metadata_enabled = True
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.meta_pub = self.create_publisher(String, "yoloe/classes", latched)
        self.mask_pub = self.create_publisher(InstanceMaskArray, "yoloe/instance_masks", 5)
        self.object_pub = self.create_publisher(TrackedObjectArray, "object_map/objects", latched)
        self.create_service(SetVocabulary, "yoloe/set_vocabulary", self.set_vocabulary)
        self.create_service(QueryObjects, "object_map/query", self.query)
        self.create_timer(.02, self.tick)

    def metadata(self):
        self.meta_pub.publish(String(data=json.dumps(dict(model="fake-yoloe", revision=self.revision,
            active_classes=self.classes, active_class_ids=list(range(1, len(self.classes) + 1))))))

    def tick(self):
        if self.publish_metadata_enabled:
            self.metadata()
        self.object_pub.publish(TrackedObjectArray(session_id=self.session))
        if not self.enabled:
            return
        stamp = self.get_clock().now().to_msg()
        if self.fresh:
            self.last_stamp = stamp
        masks = InstanceMaskArray(vocabulary_revision=self.revision)
        masks.header.stamp = stamp
        # Always a raw detection, even when no object is confirmed.
        masks.instances = [InstanceMask(instance_id=999, class_name="tentative-secret", confidence=.9)]
        self.mask_pub.publish(masks)

    def set_vocabulary(self, request, response):
        self.classes = list(request.classes)
        self.revision += 1
        response.success, response.message = True, "Fake service accepted"
        response.revision, response.classes = self.revision, self.classes
        response.class_ids = list(range(1, len(self.classes) + 1))
        self.metadata()
        return response

    def query(self, request, response):
        self.requests.append(request)
        response.success, response.current_session_id = True, self.session
        response.database_path = "/private/path/must-not-be-returned.sqlite3"
        if self.confirmed and (not request.label or request.label == "cube"):
            item = TrackedObject(object_id="confirmed-cube-uuid", session_id=self.session,
                                 label="cube", observation_count=3, confidence=.9)
            item.header.stamp, item.header.frame_id = self.last_stamp, "map"
            item.position.x, item.position.z = 1., .8
            response.objects = [item]
        return response


@pytest.fixture
def ros_pair():
    context = Context()
    rclpy.init(context=context, domain_id=87)
    peer = PerceptionPeer(context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(peer)
    spin = Thread(target=executor.spin, daemon=True)
    spin.start()
    robot = None
    try:
        robot = RosAdapter(namespace="agent_test", use_sim_time=False, service_timeout_s=2., context=context)
        wait_for(lambda: robot.status()["vocabulary"] is not None and robot.query_client.service_is_ready()
                 and robot.vocabulary_client.service_is_ready() and peer.mask_pub.get_subscription_count())
        yield robot, peer
    finally:
        if robot:
            robot.close()
        executor.shutdown(timeout_sec=2.)
        spin.join(timeout=2.)
        peer.destroy_node()
        context.try_shutdown()


def test_real_services_and_fresh_confirmed_objects_only(ros_pair):
    robot, peer = ros_pair
    result = robot.set_vocabulary(["cube", "mug"])
    assert result["success"] and result["revision"] == 2
    result = robot.observe(.2)
    assert result["success"]
    assert result["count"] == 1
    assert result["objects"][0]["object_id"] == "confirmed-cube-uuid"
    assert "tentative-secret" not in json.dumps(result)
    assert "database_path" not in result
    assert all(not request.include_images and request.session_id == "" for request in peer.requests)
    assert robot.query(limit=1)["completeness"] == "unknown"


def test_scripted_model_runs_whole_loop_through_real_ros_bindings(ros_pair):
    robot, peer = ros_pair
    events = []
    agent = Agent(ScriptedProvider(), ToolRegistry(robot), emit=events.append)
    result = agent.run("Look for cubes and mugs")
    assert "OFFLINE SCRIPTED DEMO" in result
    assert peer.classes == ["cube", "mug"]
    query_results = [e["result"] for e in events if e["event"] == "tool_result"
                     and e["name"] == "query_objects"]
    assert query_results[0]["objects"][0]["object_id"] == "confirmed-cube-uuid"
    assert all(e["result"]["success"] for e in events if e["event"] == "tool_result")


def test_raw_detection_without_confirmation_is_not_exposed(ros_pair):
    robot, peer = ros_pair
    peer.confirmed = False
    result = robot.observe(.15)
    assert result["success"]
    assert result["objects"] == []
    assert result["perception_activity"] == "fresh_detector_frames"
    assert "No freshly observed confirmed" in result["message"]
    assert "tentative-secret" not in json.dumps(result)


def test_republished_old_object_and_offline_perception_are_distinct(ros_pair):
    robot, peer = ros_pair
    peer.fresh = False
    time.sleep(.05)
    result = robot.observe(.15)
    assert result["success"] and not result["objects"]
    assert robot.query()["count"] == 1  # Stored confirmed object still exists.
    peer.enabled = False
    time.sleep(.05)
    result = robot.observe(.15)
    assert not result["success"] and result["code"] == "NO_FRESH_OBSERVATION"
    assert not result["objects"]


def test_inactive_category_not_returned_as_observed(ros_pair):
    robot, peer = ros_pair
    robot.set_vocabulary(["mug"])
    result = robot.observe(.15)
    assert result["success"] and not result["objects"]
    assert robot.query(label="cube")["count"] == 1


@pytest.mark.parametrize("change", ["session", "revision"])
def test_observation_change_is_reported(ros_pair, change):
    robot, peer = ros_pair
    def alter():
        time.sleep(.08)
        if change == "session":
            peer.session = "new-session"
        else:
            peer.revision += 1
    thread = Thread(target=alter)
    thread.start()
    try:
        with pytest.raises(ToolError, match="changed"):
            robot.observe(.4)
    finally:
        thread.join()
