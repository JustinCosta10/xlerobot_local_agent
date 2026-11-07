"""Blocking agent facade over the independently supervised ROS action."""

import json
import math
from threading import Lock
import time
from uuid import uuid4

from rclpy.action import ActionClient
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID
from xlerobot_navigation_interfaces.action import Navigate

from .tools import ToolError


def pose_data(message):
    p, q = message.pose.position, message.pose.orientation
    return dict(frame_id=message.header.frame_id, x=p.x, y=p.y,
                yaw=math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z)))


class NavigationClient:
    def __init__(self, node, emit=None):
        self.node = node
        self.emit = emit or (lambda event: None)
        self.lock = Lock()
        self.state = None
        self.received_at = 0.
        self.pending = None
        self.active = None
        self.result = None
        self.last_feedback = 0.
        self.interrupt_result = None
        self.client = ActionClient(node, Navigate, "agent_navigation/navigate")
        self.heartbeat = node.create_publisher(String, "agent_navigation/heartbeat", 1)
        node.create_subscription(String, "agent_navigation/state", self.on_state, 1)

    def on_state(self, message):
        try:
            state = json.loads(message.data)
            if not isinstance(state, dict) or state.get("simulation_only") is not True:
                return
        except ValueError:
            return
        with self.lock:
            self.state, self.received_at = state, time.monotonic()

    def status(self):
        with self.lock:
            if self.state is None or time.monotonic() - self.received_at > .5:
                return dict(success=False, ready=False, code="SUPERVISOR_UNAVAILABLE",
                            message="Start sim_navigation.launch.py with agent_navigation:=true.")
            state = dict(self.state, success=True)
        if not self.client.server_is_ready():
            state.update(ready=False, reason="Navigation supervisor action is unavailable.")
        return state

    def feedback(self, message):
        # Terminal feedback is informational; it never initiates an LLM request.
        now = time.monotonic()
        if now - self.last_feedback >= 1.:
            self.last_feedback = now
            fb = message.feedback
            self.emit(dict(event="navigation_progress", pose=pose_data(fb.current_pose),
                           distance_remaining_m=fb.distance_remaining, elapsed_s=fb.elapsed_s))

    def wait(self, future, deadline, operation_id=None):
        next_heartbeat = 0.
        while not future.done():
            now = time.monotonic()
            if now >= deadline or not self.node.context.ok():
                raise ToolError("RESPONSE_TIMEOUT", "Navigation response unavailable; cancellation requested.")
            if operation_id and now >= next_heartbeat:
                self.heartbeat.publish(String(data=operation_id))
                next_heartbeat = now + .1
            time.sleep(.02)
        return future.result()

    def navigate(self, x, y, yaw, session_id, timeout_s):
        if self.pending is not None or self.active is not None:
            raise ToolError("BUSY", "A navigation request is already pending or active.")
        state = self.status()
        if not state.get("ready"):
            raise ToolError("NOT_READY", state.get("reason") or state.get("message", "Navigation unavailable."))
        if session_id != state.get("session_id"):
            raise ToolError("MAP_CHANGED", "Read get_navigation_state and use its current session_id.")
        if not self.client.server_is_ready():
            raise ToolError("NOT_READY", "Navigation supervisor action is unavailable.")
        request = Navigate.Goal(session_id=session_id, timeout_s=float(timeout_s))
        request.pose.header.frame_id = "map"
        request.pose.header.stamp = self.node.get_clock().now().to_msg()
        request.pose.pose.position.x, request.pose.pose.position.y = float(x), float(y)
        request.pose.pose.orientation.z = math.sin(yaw / 2)
        request.pose.pose.orientation.w = math.cos(yaw / 2)
        operation_id = uuid4()
        self.interrupt_result = None
        self.last_feedback = 0.
        try:
            self.pending = self.client.send_goal_async(request, feedback_callback=self.feedback,
                goal_uuid=UUID(uuid=list(operation_id.bytes)))
            self.active = self.wait(self.pending, time.monotonic() + 5., operation_id.hex)
            self.pending = None
            if not self.active.accepted:
                return dict(success=False, code="REJECTED", operation_id=operation_id.hex,
                            message="Supervisor rejected goal; inspect navigation state before retrying.")
            self.result = self.active.get_result_async()
            response = self.wait(self.result, time.monotonic() + timeout_s + 7., operation_id.hex)
            result = response.result
            return dict(success=result.code == "SUCCEEDED", code=result.code, message=result.message,
                        operation_id=operation_id.hex, stop_confirmed=result.stop_confirmed,
                        final_pose=pose_data(result.final_pose) if result.final_pose_valid else None)
        except ToolError as error:
            self.interrupt_result = self.cancel()
            return dict(success=False, code=error.code, message=str(error),
                        operation_id=operation_id.hex, cancellation=self.interrupt_result,
                        stop_confirmed=self.interrupt_result.get("stop_confirmed", False))
        except BaseException:
            # Stop renewing the lease before attempting cancellation. Even a second
            # Ctrl+C or SIGKILL cannot leave the supervisor renewing this goal.
            self.interrupt_result = self.cancel()
            raise
        finally:
            self.pending = self.active = self.result = None

    def cancel(self):
        try:
            if self.pending is not None:
                pending = self.pending
                def cancel_late(future):
                    handle = future.result()
                    if handle.accepted:
                        handle.cancel_goal_async()
                pending.add_done_callback(cancel_late)
            if self.active is not None and self.active.accepted:
                if self.result is None:
                    self.result = self.active.get_result_async()
                self.active.cancel_goal_async()
                response = self.wait(self.result, time.monotonic() + 6.)
                return dict(success=response.result.stop_confirmed and response.result.code != "STOP_UNCONFIRMED",
                            code=response.result.code,
                            stop_confirmed=response.result.stop_confirmed)
            return dict(success=self.pending is None, code="NO_ACTIVE_GOAL" if self.pending is None
                        else "CANCEL_PENDING", stop_confirmed=False)
        except (Exception, KeyboardInterrupt):
            return dict(success=False, code="STOP_UNCONFIRMED", stop_confirmed=False,
                        message="Cancellation unconfirmed; supervisor lease expires independently.")
