"""Real DDS supervisor/guard/action tests; synthetic sensors and base, no motors/API."""

import json
import math
import os
import signal
import subprocess
import sys
from threading import Thread
import time
from uuid import uuid4

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import GetCostmap
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry
from nvblox_msgs.msg import DistanceMapSlice
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.context import Context
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID
from xlerobot_navigation.guard import NavigationGuard
from xlerobot_navigation.supervisor import NavigationSupervisor
from xlerobot_navigation_interfaces.action import Navigate
from xlerobot_perception_interfaces.msg import TrackedObject, TrackedObjectArray
from xlerobot_perception_interfaces.srv import QueryObjects

from xlerobot_agent.navigation import NavigationClient
from xlerobot_agent.tools import ToolRegistry
from xlerobot_agent.demo import FakePerception
from xlerobot_agent.agent import Agent
from xlerobot_agent.providers.base import ToolCall, Turn


def wait_for(predicate, timeout=8):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, "Navigation fixture timed out"
        time.sleep(.02)


class NavigationPeer(Node):
    def __init__(self, context, namespace):
        super().__init__("navigation_peer", namespace=namespace, context=context)
        self.started = time.monotonic()
        self.clock = 10.
        self.paused = False
        self.session = "test-map-one"
        self.tracking = True
        self.operation = None
        self.received = []
        self.commands = []
        self.velocity = 0.
        self.x = 0.
        self.yaw = 0.
        self.terminate_after = .5
        self.reject = False
        self.abort = False
        self.ignore_cancel = False
        self.accept_delay = 0.
        self.lifecycle_active = True
        self.object_exists = True
        self.object_x = 1.5
        self.object_session = None
        self.costmap_unknown = False
        self.costmap_stale = False
        self.path_blocked = False
        self.path_delay = 0.
        self.paths = []
        self.change_map_after_plan = False
        self.create_service(QueryObjects, "object_map/query", self.query_object)
        self.create_service(GetCostmap, "global_costmap/get_costmap", self.get_costmap)
        self.create_service(GetParameters, "global_costmap/global_costmap/get_parameters", self.parameters)
        self.create_service(GetParameters, "controller_server/get_parameters", self.parameters)
        self.path_server = ActionServer(self, ComputePathToPose, "compute_path_to_pose", self.compute_path,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=ReentrantCallbackGroup())
        self.create_service(GetState, "bt_navigator/get_state", self.lifecycle)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "localization/pose", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_rect_raw", 10)
        self.map_pub = self.create_publisher(DistanceMapSlice, "nvblox_node/static_map_slice", 1)
        self.tracking_pub = self.create_publisher(VisualSlamStatus, "visual_slam/status", 10)
        self.session_pub = self.create_publisher(TrackedObjectArray, "object_map/objects",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.velocity_pub = self.create_publisher(TwistStamped, "cmd_vel_navigation", 1)
        self.create_subscription(TwistStamped, "cmd_vel", self.command, 10)
        self.server = ActionServer(self, NavigateToPose, "navigate_to_pose_internal", self.execute,
                                   goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=ReentrantCallbackGroup())
        self.create_timer(.02, self.tick)

    def accept(self, request):
        if self.accept_delay:
            time.sleep(self.accept_delay)
        self.received.append(request)
        return GoalResponse.REJECT if self.reject else GoalResponse.ACCEPT

    def lifecycle(self, request, response):
        response.current_state.id = 3 if self.lifecycle_active else 2
        return response

    def parameters(self, request, response):
        for name in request.names:
            if name == "footprint":
                response.values.append(ParameterValue(type=ParameterType.PARAMETER_STRING,
                    string_value='[[-0.356,-0.252],[0.064,-0.252],[0.064,0.252],[-0.356,0.252]]'))
            else:
                response.values.append(ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                    double_value=.02 if name == "footprint_padding" else .12))
        return response

    def query_object(self, request, response):
        response.success = True
        response.current_session_id = self.session
        if self.object_exists and request.object_id == "stored-cube":
            obj = TrackedObject(object_id="stored-cube", session_id=self.object_session or self.session,
                                label="cube", observation_count=5, stale=True)
            obj.header.frame_id = "map"
            obj.header.stamp.sec = 1  # Old observations remain valid last-known destinations.
            obj.position.x, obj.position.z = self.object_x, .8
            response.objects = [obj]
        return response

    def get_costmap(self, request, response):
        mapping = response.map
        mapping.header.frame_id = "map"
        mapping.metadata.resolution = .05
        mapping.metadata.size_x = mapping.metadata.size_y = 160
        mapping.metadata.origin.position.x = mapping.metadata.origin.position.y = -4.
        mapping.metadata.origin.orientation.w = 1.
        mapping.metadata.update_time.sec = int(self.clock) - (10 if self.costmap_stale else 0)
        mapping.metadata.update_time.nanosec = int((self.clock % 1)*1e9)
        data = [255 if self.costmap_unknown else 0] * (160*160)
        if not self.costmap_unknown:
            for y in range(72, 88):
                for x in range(102, 120):
                    data[y*160+x] = 254
        mapping.data = data
        return response

    async def compute_path(self, handle):
        self.paths.append(handle.request)
        if self.path_delay:
            time.sleep(self.path_delay)
        result = ComputePathToPose.Result()
        if handle.is_cancel_requested:
            handle.canceled()
        elif self.path_blocked:
            handle.abort()
            result.error_code = 1
        else:
            handle.succeed()
            result.path.header.frame_id = "map"
            result.path.poses = [handle.request.goal]
            if self.change_map_after_plan:
                self.costmap_unknown = True
        return result

    async def execute(self, handle):
        done = Future(executor=self.executor)
        self.operation = dict(handle=handle, started=time.monotonic(), done=done)
        await done
        if handle.is_cancel_requested:
            handle.canceled()
        elif self.abort:
            handle.abort()
        else:
            handle.succeed()
        return NavigateToPose.Result(error_msg="Synthetic blocked route" if self.abort else "")

    def command(self, message):
        self.velocity = message.twist.linear.x
        self.commands.append(self.velocity)

    def tick(self):
        if self.paused:
            return
        self.clock += .02
        stamp = Clock().clock
        stamp.sec, stamp.nanosec = int(self.clock), int((self.clock % 1) * 1e9)
        self.clock_pub.publish(Clock(clock=stamp))
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = "map", stamp
        self.x += self.velocity * .02
        pose.pose.position.x = self.x
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(self.yaw/2), math.cos(self.yaw/2)
        self.pose_pub.publish(pose)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.twist.twist.linear.x = self.velocity
        self.odom_pub.publish(odom)
        depth = Image()
        depth.header.stamp = stamp
        self.depth_pub.publish(depth)
        mapping = DistanceMapSlice(width=2, height=2, data=[1., 1., 1., 1.])
        mapping.header.stamp = stamp
        self.map_pub.publish(mapping)
        tracking = VisualSlamStatus(vo_state=1 if self.tracking else 0)
        tracking.header.stamp = stamp
        self.tracking_pub.publish(tracking)
        session = TrackedObjectArray(session_id=self.session)
        session.header.stamp = stamp
        self.session_pub.publish(session)
        command = TwistStamped()
        command.header.stamp = stamp
        op = self.operation
        if op:
            elapsed = time.monotonic() - op["started"]
            if ((op["handle"].is_cancel_requested and not self.ignore_cancel)
                    or elapsed > self.terminate_after):
                op["done"].set_result(True)
                self.operation = None
            else:
                command.twist.linear.x = .1
                op["handle"].publish_feedback(NavigateToPose.Feedback(
                    current_pose=pose, distance_remaining=.5))
        self.velocity_pub.publish(command)


@pytest.fixture
def navigation():
    context = Context()
    rclpy.init(context=context, domain_id=88)
    namespace = "agent_nav_" + uuid4().hex[:8]
    options = dict(context=context, namespace=namespace,
                   parameter_overrides=[Parameter("use_sim_time", value=True)])
    peer = NavigationPeer(context, namespace)
    supervisor = NavigationSupervisor(**options)
    guard = NavigationGuard(**dict(options, parameter_overrides=options["parameter_overrides"] +
        [Parameter("require_navigation_permit", value=True)]))
    node = Node("navigation_client_test", **options)
    events = []
    client = NavigationClient(node, emit=events.append)
    # Peer has a separate executor so a delayed Nav2 acceptance never stalls the watchdog.
    executor = SingleThreadedExecutor(context=context)
    peer_executor = MultiThreadedExecutor(num_threads=2, context=context)
    peer_executor.add_node(peer)
    for item in (supervisor, guard, node):
        executor.add_node(item)
    threads = [Thread(target=e.spin, daemon=True) for e in (executor, peer_executor)]
    for thread in threads:
        thread.start()
    try:
        wait_for(lambda: client.status().get("ready"))
        yield client, peer, supervisor, events
    finally:
        client.cancel()
        for ex in (executor, peer_executor):
            ex.shutdown(timeout_sec=3.)
        for thread in threads:
            thread.join(timeout=3.)
        client.client.destroy()
        supervisor.server.destroy()
        supervisor.nav.destroy()
        peer.server.destroy()
        peer.path_server.destroy()
        supervisor.approach.planner.destroy()
        for item in (node, supervisor, guard, peer):
            item.destroy_node()
        context.try_shutdown()


def run_goal(client, **kwargs):
    return client.navigate(x=.3, y=0., yaw=0., session_id=client.status()["session_id"],
                           timeout_s=kwargs.get("timeout_s", 20.))


def test_success_feedback_and_measured_stop(navigation):
    client, peer, supervisor, events = navigation
    result = run_goal(client)
    assert result["success"] and result["code"] == "SUCCEEDED"
    assert result["stop_confirmed"] and result["final_pose"]["frame_id"] == "map"
    assert any(v > 0 for v in peer.commands)
    assert peer.commands[-1] == 0.
    assert events and events[0]["event"] == "navigation_progress"
    assert peer.received[0].behavior_tree == ""
    assert not supervisor.fault


@pytest.mark.parametrize("mode,code", [("reject", "REJECTED"), ("abort", "FAILED")])
def test_backend_failures_are_not_success(navigation, mode, code):
    client, peer, _, _ = navigation
    setattr(peer, mode, True)
    result = run_goal(client)
    assert not result["success"] and result["code"] == code and result["stop_confirmed"]


@pytest.mark.parametrize("change,code", [("session", "MAP_CHANGED"), ("tracking", "NOT_READY"),
                                       ("timeout", "TIMED_OUT")])
def test_session_tracking_and_deadline_cancel_without_retry(navigation, change, code):
    client, peer, _, _ = navigation
    peer.terminate_after = 100.
    result = []
    worker = Thread(target=lambda: result.append(run_goal(client, timeout_s=3. if change == "timeout" else 20.)))
    worker.start()
    wait_for(lambda: any(v > 0 for v in peer.commands))
    if change == "session":
        peer.session = "test-map-two"
    elif change == "tracking":
        peer.tracking = False
    worker.join(timeout=8.)
    assert result and result[0]["code"] == code and result[0]["stop_confirmed"]
    assert len(peer.received) == 1 and peer.commands[-1] == 0.


def test_ctrl_c_cancels_and_stops_later_agent_calls(navigation):
    client, peer, _, _ = navigation
    peer.terminate_after = 100.
    normal_wait = client.wait
    interrupted = False

    def interrupt_wait(future, deadline, operation_id=None):
        nonlocal interrupted
        if client.active and operation_id and not interrupted:
            interrupted = True
            raise KeyboardInterrupt()
        return normal_wait(future, deadline, operation_id)

    client.wait = interrupt_wait

    class Robot(FakePerception):
        navigation_enabled = True
        navigation = client
        def navigate(self, **kwargs):
            return client.navigate(**kwargs)
        def observe(self):
            pytest.fail("Dispatched after Ctrl+C")

    class Model:
        def next_turn(self, history, schemas, instructions):
            if not any(t.role == "tool" for t in history):
                return Turn("assistant", calls=[ToolCall("p", "update_plan", '{"steps":["Navigate"]}')])
            return Turn("assistant", calls=[ToolCall("n", "navigate", json.dumps(dict(
                x=.3, y=0, yaw=0, session_id=peer.session, timeout_s=20))),
                ToolCall("late", "observe_scene", "{}")])

    agent = Agent(Model(), ToolRegistry(Robot()))
    assert agent.run("Navigate") is None
    assert client.interrupt_result["stop_confirmed"]
    assert peer.commands[-1] == 0.


def test_lost_agent_process_revokes_motion(navigation):
    client, peer, supervisor, _ = navigation
    peer.terminate_after = 100.
    script = """
import time
from threading import Thread
import rclpy
from rclpy.parameter import Parameter
from xlerobot_agent.navigation import NavigationClient
rclpy.init(domain_id=88)
node = rclpy.create_node('doomed_agent', namespace='agent_nav_test',
                        parameter_overrides=[Parameter('use_sim_time', value=True)])
client = NavigationClient(node)
Thread(target=lambda: rclpy.spin(node), daemon=True).start()
deadline = time.monotonic() + 8
while not client.status().get('ready'):
    assert time.monotonic() < deadline
    time.sleep(.02)
client.navigate(x=.3, y=0., yaw=0., session_id=client.status()['session_id'], timeout_s=20.)
"""
    script = script.replace("agent_nav_test", peer.get_namespace().strip("/"))
    process = subprocess.Popen([sys.executable, "-c", script], stdout=subprocess.DEVNULL,
                               stderr=subprocess.PIPE)
    try:
        wait_for(lambda: any(v > 0 for v in peer.commands))
        os.kill(process.pid, signal.SIGKILL)
        process.wait(timeout=3.)
        wait_for(lambda: supervisor.last_result is not None)
        assert supervisor.last_result["code"] == "CLIENT_LOST"
        assert supervisor.last_result["stop_confirmed"] and peer.commands[-1] == 0.
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3.)
        process.stderr.close()


def test_paused_simulation_revokes_permission_and_reports_unconfirmed_stop(navigation):
    client, peer, supervisor, _ = navigation
    peer.terminate_after = 100.
    result = []
    worker = Thread(target=lambda: result.append(run_goal(client)))
    worker.start()
    wait_for(lambda: any(v > 0 for v in peer.commands))
    peer.paused = True
    wait_for(lambda: peer.commands[-1] == 0.)
    worker.join(timeout=9.)
    assert result and result[0]["code"] == "STOP_UNCONFIRMED"
    assert not result[0]["stop_confirmed"] and supervisor.fault


def test_busy_old_session_and_invalid_goals_rejected_without_nav2_dispatch(navigation):
    client, peer, _, _ = navigation
    raw = ActionClient(client.node, Navigate, "agent_navigation/navigate")
    wait_for(raw.server_is_ready)
    def send(session, x=0.):
        request = Navigate.Goal(session_id=session, timeout_s=10.)
        request.pose.header.frame_id = "map"
        request.pose.pose.position.x = x
        request.pose.pose.orientation.w = 1.
        future = raw.send_goal_async(request)
        wait_for(future.done)
        return future.result()
    try:
        assert not send("old-map").accepted
        assert not send(peer.session, float("nan")).accepted
        first = send(peer.session)
        assert first.accepted
        assert not send(peer.session).accepted
        # No heartbeat means this accepted request must never dispatch to Nav2.
        result = first.get_result_async()
        wait_for(result.done)
        assert result.result().result.code == "CLIENT_LOST"
        assert not peer.received
    finally:
        raw.destroy()


def test_late_acceptance_is_canceled_without_permitting_motion(navigation):
    client, peer, _, _ = navigation
    peer.accept_delay = 1.5
    peer.terminate_after = 100.
    result = run_goal(client, timeout_s=1.)
    assert result["code"] == "TIMED_OUT"
    assert result["stop_confirmed"]
    assert not any(v > 0 for v in peer.commands)


def test_backward_clock_jump_invalidates_goal_and_blocks_new_motion(navigation):
    client, peer, supervisor, _ = navigation
    peer.terminate_after = 100.
    result = []
    worker = Thread(target=lambda: result.append(run_goal(client)))
    worker.start()
    wait_for(lambda: any(v > 0 for v in peer.commands))
    peer.clock -= 5.
    worker.join(timeout=8.)
    assert result and result[0]["code"] == "CLOCK_RESET" and result[0]["stop_confirmed"]
    assert supervisor.fault and not client.status()["ready"]


def test_unresponsive_backend_revokes_motion_and_latches_fault(navigation):
    client, peer, supervisor, _ = navigation
    peer.terminate_after = 100.
    peer.ignore_cancel = True
    result = run_goal(client, timeout_s=1.)
    assert result["code"] == "STOP_UNCONFIRMED" and result["stop_confirmed"]
    assert supervisor.fault and peer.commands[-1] == 0.


def test_inactive_lifecycle_blocks_navigation_even_when_action_exists(navigation):
    client, peer, _, _ = navigation
    peer.lifecycle_active = False
    wait_for(lambda: not client.status()["ready"])
    assert "not active" in client.status()["reason"]
    assert not peer.received


def run_object(client, object_id="stored-cube", timeout_s=20.):
    return client.navigate_to_object(object_id=object_id, standoff_m=.8,
                                    session_id=client.status()["session_id"], timeout_s=timeout_s)


def test_relative_destination_resolved_once_on_supervisor(navigation):
    client, peer, _, _ = navigation
    peer.yaw = math.pi/2
    wait_for(lambda: abs(client.status()["pose"]["yaw"]-math.pi/2) < .01)
    result = client.navigate_relative(forward_m=.5, left_m=.2, yaw_delta_rad=-math.pi/2,
                                     session_id=peer.session, timeout_s=20.)
    assert result["success"]
    assert result["resolved_pose"]["x"] == pytest.approx(-.2)
    assert result["resolved_pose"]["y"] == pytest.approx(.5)
    assert result["resolved_pose"]["yaw"] == pytest.approx(0.)
    assert len(peer.received) == 1


def test_object_approach_selects_clear_floor_pose_not_object_position(navigation):
    client, peer, _, _ = navigation
    result = run_object(client)
    assert result["success"] and result["stop_confirmed"]
    assert result["target"]["stale"] and result["target"]["object_id"] == "stored-cube"
    assert result["target"]["selected_standoff_m"] >= 1.2-1e-6
    assert result["target"]["clearance_envelope_radius_m"] > .63
    assert peer.paths and len(peer.received) == 1
    goal = peer.received[0].pose.pose
    assert goal.position.z == 0 and goal.position.x != peer.object_x
    assert result["resolved_pose"]["yaw"] == pytest.approx(math.atan2(-goal.position.y, peer.object_x-goal.position.x))


@pytest.mark.parametrize("failure,expected", [("missing", "OBJECT_NOT_FOUND"),
    ("session", "OBJECT_NOT_FOUND"), ("unknown", "NO_SAFE_APPROACH"),
    ("stale_map", "STALE_COSTMAP"), ("blocked", "NO_SAFE_APPROACH"),
    ("changed", "APPROACH_CHANGED")])
def test_object_resolution_failures_never_dispatch_motion(navigation, failure, expected):
    client, peer, _, _ = navigation
    if failure == "missing":
        peer.object_exists = False
    elif failure == "session":
        peer.object_session = "previous-map"
    elif failure == "unknown":
        peer.costmap_unknown = True
    elif failure == "stale_map":
        peer.costmap_stale = True
    elif failure == "blocked":
        peer.path_blocked = True
    elif failure == "changed":
        peer.change_map_after_plan = True
    result = run_object(client)
    assert result["code"] == expected and not result["success"]
    assert not peer.received and not any(v > 0 for v in peer.commands)


def test_cancel_during_object_planning_never_dispatches_late_motion(navigation):
    client, peer, _, _ = navigation
    peer.path_delay = .7
    results = []
    worker = Thread(target=lambda: results.append(run_object(client)))
    worker.start()
    wait_for(lambda: peer.paths)
    stopped = client.cancel()
    worker.join(timeout=8.)
    assert stopped["stop_confirmed"] and results[0]["code"] == "CANCELED"
    time.sleep(.8)
    assert not peer.received and not any(v > 0 for v in peer.commands)
