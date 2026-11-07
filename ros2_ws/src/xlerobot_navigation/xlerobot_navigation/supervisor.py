"""Own one Nav2 goal; revoke velocity permission independently of the agent.

Run only in the simulation launch's exclusive agent mode. Nav2's action is
remapped to navigate_to_pose_internal. The guard requires this node's permit.
All deadlines and leases use monotonic time, including when /clock pauses.
"""

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from std_msgs.msg import Bool, String
from xlerobot_navigation_interfaces.action import Navigate
from xlerobot_perception_interfaces.msg import TrackedObjectArray

from .guard import MappingStatus, fresh
from .approach import ObjectApproach, pose_message
from .destinations import DestinationError, relative_destination


def pose_data(message):
    p, q = message.pose.position, message.pose.orientation
    return dict(frame_id=message.header.frame_id, x=p.x, y=p.y,
                yaw=math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z)))


def valid_goal(request):
    if not math.isfinite(request.timeout_s) or not 1 <= request.timeout_s <= 300:
        return False
    if request.target_type == Navigate.Goal.RELATIVE:
        values = (request.forward_m, request.left_m, request.yaw_delta_rad)
        return (all(math.isfinite(v) for v in values) and abs(request.forward_m) <= 3
                and abs(request.left_m) <= 3 and abs(request.yaw_delta_rad) <= math.pi)
    if request.target_type == Navigate.Goal.OBJECT:
        return (0 < len(request.object_id) <= 128 and math.isfinite(request.standoff_m)
                and .5 <= request.standoff_m <= 2.)
    if request.target_type != Navigate.Goal.POINT:
        return False
    p, q = request.pose.pose.position, request.pose.pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w, request.timeout_s)
    return (all(math.isfinite(v) for v in values) and request.pose.header.frame_id == "map"
            and -8 < p.x < 8 and -8 < p.y < 8 and abs(p.z) < 1e-6
            and abs(q.x) < 1e-6 and abs(q.y) < 1e-6
            and abs(q.z*q.z + q.w*q.w - 1) < 1e-5
            and 1 <= request.timeout_s <= 300)


class NavigationSupervisor(MappingStatus):
    def __init__(self, **kwargs):
        super().__init__("navigation_supervisor", **kwargs)
        if not self.get_parameter("use_sim_time").value:
            raise ValueError("Agent navigation is currently simulation-only (use_sim_time must be true).")
        self.session = None
        self.pose = None
        self.odom = None
        self.operation = None
        self.reserved = False
        self.fault = None
        self.last_clock = 0.
        self.last_result = None
        self.results = {}
        self.lifecycle_active = False
        self.lifecycle_received = 0.
        self.lifecycle_pending = None
        self.lifecycle_client = self.create_client(GetState, "bt_navigator/get_state")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TrackedObjectArray, "object_map/objects", self.on_session, latched)
        self.create_subscription(PoseStamped, "localization/pose", self.on_pose, 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.create_subscription(String, "agent_navigation/heartbeat", self.on_heartbeat, 1)
        self.permit = self.create_publisher(Bool, "agent_navigation/permit", 1)
        self.state_pub = self.create_publisher(String, "agent_navigation/state", 1)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose_internal")
        self.approach = ObjectApproach(self)
        self.server = ActionServer(self, Navigate, "agent_navigation/navigate", self.execute,
                                   goal_callback=self.accept, cancel_callback=self.cancel,
                                   handle_accepted_callback=self.accepted)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_session(self, message):
        if self.session is not None and self.session != message.session_id:
            self.stop("MAP_CHANGED", "Map session changed; destination invalidated.")
        self.session = message.session_id or None
        self.record("session", message)

    def on_pose(self, message):
        p, q = message.pose.position, message.pose.orientation
        if (message.header.frame_id == "map"
                and all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                and abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) < .01):
            self.pose = message
            self.record("valid_pose", message)

    def on_odom(self, message):
        self.odom = message
        self.record("odom", message)
        op = self.operation
        if op and op["phase"] == "stopping":
            v, w = message.twist.twist.linear.x, message.twist.twist.angular.z
            # Require several fresh measured samples after revoking permission.
            if self.sample_fresh("odom", .5) and abs(v) < .01 and abs(w) < .02:
                if op["stationary_since"] is None:
                    op["stationary_since"] = time.monotonic()
            else:
                op["stationary_since"] = None

    def sample_fresh(self, name, timeout=2.):
        return fresh(self.samples.get(name), time.monotonic(),
                     self.get_clock().now().nanoseconds * 1e-9, timeout)

    def unavailable_reason(self):
        if self.fault:
            return self.fault
        if not self.ready() or not self.sample_fresh("valid_pose"):
            return "Mapping or localization is unavailable/stale."
        if not self.session or not self.sample_fresh("session"):
            return "Current object-map session is unavailable/stale."
        if not self.sample_fresh("odom", .5):
            return "Measured odometry is unavailable/stale."
        if not self.nav.server_is_ready():
            return "Nav2 action server unavailable."
        if not self.lifecycle_active or time.monotonic() - self.lifecycle_received > 2.:
            return "Nav2 navigator is not active or its lifecycle state is stale."
        return None

    def on_heartbeat(self, message):
        op = self.operation
        if op and message.data == op["id"] and op["phase"] != "stopping":
            op["heartbeat"] = time.monotonic()

    def accept(self, request):
        if (self.reserved or self.operation or self.unavailable_reason()
                or not valid_goal(request) or request.session_id != self.session):
            return GoalResponse.REJECT
        self.reserved = True
        return GoalResponse.ACCEPT

    def accepted(self, handle):
        now = time.monotonic()
        self.operation = dict(id=bytes(handle.goal_id.uuid).hex(), handle=handle,
                              finished=Future(executor=self.executor), started=now,
                              heartbeat=None, phase="starting", sent=None, backend=None,
                              result=None, cancel=None, sent_checked=False, stationary_since=None,
                              resolved_pose=None, target_details={})
        self.results[self.operation["id"]] = self.operation["finished"]
        self.reserved = False
        handle.execute()

    async def execute(self, handle):
        operation_id = bytes(handle.goal_id.uuid).hex()
        result = await self.results[operation_id]
        del self.results[operation_id]
        if result.code == "SUCCEEDED":
            handle.succeed()
        elif handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        return result

    def cancel(self, handle):
        if self.operation and bytes(handle.goal_id.uuid).hex() == self.operation["id"]:
            self.stop("CANCELED", "Navigation canceled by the client.")
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    def stop(self, code, message):
        op = self.operation
        if not op or op["phase"] == "stopping":
            return
        op.update(phase="stopping", code=code, message=message,
                  stop_deadline=time.monotonic() + 5., stationary_since=None)
        self.permit.publish(Bool(data=False))
        self.approach.cancel(op)

    def feedback(self, op, message):
        if self.operation is op and op["phase"] == "running":
            fb = message.feedback
            op["handle"].publish_feedback(Navigate.Feedback(
                current_pose=fb.current_pose, distance_remaining=fb.distance_remaining,
                elapsed_s=time.monotonic() - op["started"]))

    def finish(self, stopped, backend_done):
        op = self.operation
        if not stopped or not backend_done:
            self.fault = "Stop or backend termination unconfirmed; restart the navigation stack."
        code = op["code"] if stopped and backend_done else "STOP_UNCONFIRMED"
        result = Navigate.Result(code=code, message=op["message"], stop_confirmed=stopped,
                                 final_pose_valid=self.sample_fresh("valid_pose"))
        if code == "STOP_UNCONFIRMED":
            result.message += " " + self.fault
        if result.final_pose_valid:
            result.final_pose = self.pose
        if op["resolved_pose"] is not None:
            result.resolved_pose_valid = True
            result.resolved_pose = op["resolved_pose"]
        details = dict(op["target_details"])
        if details.get("kind") == "object" and result.final_pose_valid:
            target = details["object_position_m"]
            details["final_standoff_m"] = math.hypot(self.pose.pose.position.x-target[0],
                                                     self.pose.pose.position.y-target[1])
            details["observation_age_s_at_finish"] = (self.get_clock().now().nanoseconds*1e-9
                                                     - details["observation_stamp_s"])
        result.target_details_json = json.dumps(details, allow_nan=False)
        self.last_result = dict(operation_id=op["id"], code=code,
                                cause=op["code"], stop_confirmed=stopped)
        op["finished"].set_result(result)
        self.operation = None

    def send_navigation(self, op, pose):
        if not (-8 < pose.pose.position.x < 8 and -8 < pose.pose.position.y < 8):
            raise DestinationError("OUTSIDE_MAP", "Resolved destination is outside the configured navigation map.")
        op["resolved_pose"] = pose
        pose.header.stamp = self.get_clock().now().to_msg()
        op["sent"] = self.nav.send_goal_async(NavigateToPose.Goal(pose=pose),
            feedback_callback=lambda message: self.feedback(op, message))
        op["sent"].add_done_callback(lambda future: self.backend_accepted(op, future))
        op["nav_sent_at"] = time.monotonic()
        op["phase"] = "accepting"

    def resolve_destination(self, op):
        request = op["handle"].request
        if op["phase"] == "starting" and op["heartbeat"] is not None:
            start = pose_data(self.pose)
            if request.target_type == Navigate.Goal.OBJECT:
                self.approach.begin(op, start)
            elif request.target_type == Navigate.Goal.RELATIVE:
                goal = relative_destination(start, request.forward_m, request.left_m, request.yaw_delta_rad)
                op["target_details"] = dict(kind="relative", starting_pose=start,
                    forward_m=request.forward_m, left_m=request.left_m, yaw_delta_rad=request.yaw_delta_rad)
                self.send_navigation(op, pose_message(self, goal))
            else:
                op["target_details"] = dict(kind="point")
                self.send_navigation(op, request.pose)
        elif op["phase"] in ("querying_object", "finding_approach", "planning_approach", "checking_approach"):
            if op.get("planning_rejected"):
                raise DestinationError("PLANNER_REJECTED", "Nav2 rejected the path-planning request; no movement dispatched.")
            resolved = self.approach.advance(op)
            if resolved is not None:
                self.send_navigation(op, resolved)

    def tick(self):
        now = time.monotonic()
        if self.lifecycle_pending is not None and self.lifecycle_pending.done():
            response = self.lifecycle_pending.result()
            self.lifecycle_active = response is not None and response.current_state.id == 3
            self.lifecycle_received = now
            self.lifecycle_pending = None
        if (self.lifecycle_pending is None and now - self.lifecycle_received > .5
                and self.lifecycle_client.service_is_ready()):
            self.lifecycle_pending = self.lifecycle_client.call_async(GetState.Request())
        sim_now = self.get_clock().now().nanoseconds * 1e-9
        if sim_now < self.last_clock - .1:
            self.stop("CLOCK_RESET", "Simulation clock moved backward; restart navigation.")
            self.fault = "Simulation clock reset; restart the navigation stack."
        self.last_clock = sim_now
        op = self.operation
        if op:
            request = op["handle"].request
            reason = self.unavailable_reason()
            if request.session_id != self.session:
                self.stop("MAP_CHANGED", "Map session changed; destination invalidated.")
            elif reason:
                self.stop("NOT_READY", reason)
            elif now - op["started"] >= request.timeout_s:
                self.stop("TIMED_OUT", "Navigation wall-clock deadline exceeded.")
            elif now - (op["heartbeat"] or op["started"]) > 1.:
                self.stop("CLIENT_LOST", "Agent heartbeat expired.")
            try:
                self.resolve_destination(op)
            except DestinationError as error:
                self.stop(error.code, str(error))
            if op["phase"] == "accepting" and now - op["nav_sent_at"] > 5.:
                self.stop("ACCEPTANCE_TIMEOUT", "Nav2 did not acknowledge the goal in time.")
            if op["result"] is not None and op["result"].done() and op["phase"] != "stopping":
                response = op["result"].result()
                success = response.status == GoalStatus.STATUS_SUCCEEDED
                self.stop("SUCCEEDED" if success else "FAILED",
                          "Reached the requested base pose." if success else
                          f"Nav2 terminated with status {response.status}: {response.result.error_msg}")
            if op["phase"] == "stopping":
                if op["backend"] is not None and op["cancel"] is None and not op["result"].done():
                    op["cancel"] = op["backend"].cancel_goal_async()
                backend_done = (op["sent"] is None or
                                (op["sent_checked"] and op["backend"] is None) or
                                (op["result"] is not None and op["result"].done()))
                stopped = (op["stationary_since"] is not None and
                           now - op["stationary_since"] >= .25 and self.sample_fresh("odom", .5))
                if (backend_done and stopped) or now >= op["stop_deadline"]:
                    self.finish(stopped, backend_done)
        op = self.operation
        self.permit.publish(Bool(data=bool(op and op["phase"] == "running")))
        reason = self.unavailable_reason()
        self.state_pub.publish(String(data=json.dumps(dict(
            simulation_only=True, ready=reason is None and op is None and not self.reserved,
            reason=reason or ("Navigation already active." if op or self.reserved else None),
            session_id=self.session, frame_id="map",
            pose=pose_data(self.pose) if self.pose and self.sample_fresh("valid_pose") else None,
            operation_id=op["id"] if op else None, phase=op["phase"] if op else "idle",
            last_result=self.last_result, max_duration_s=300,
            limits=dict(linear_m_s=.1, angular_rad_s=.2)), allow_nan=False)))

    def backend_accepted(self, op, future):
        handle = future.result()
        op["sent_checked"] = True
        if not handle.accepted:
            if self.operation is op:
                self.stop("REJECTED", "Nav2 rejected the goal.")
            return
        op["backend"] = handle
        op["result"] = handle.get_result_async()
        if self.operation is not op or op["phase"] == "stopping":
            op["cancel"] = handle.cancel_goal_async()
        else:
            op["phase"] = "running"


def main():
    rclpy.init()
    node = NavigationSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.context.ok():
            node.permit.publish(Bool(data=False))
        node.destroy_node()
        rclpy.try_shutdown()
