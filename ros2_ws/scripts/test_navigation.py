#!/usr/bin/env python3
"""Exercise actual Isaac depth -> nvblox -> Nav2 -> wheel motion, without real motors."""

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from nvblox_msgs.msg import DistanceMapSlice
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data


def check(process):
    rclpy.init()
    node = rclpy.create_node("navigation_acceptance", parameter_overrides=[Parameter("use_sim_time", value=True)])
    data = {}
    for key, topic, msg_type in (
        ("pose", "/xlerobot_test/sim_ground_truth", PoseStamped),
        ("odom", "/xlerobot_test/odom", Odometry),
        ("slice", "/xlerobot_test/nvblox_node/static_map_slice", DistanceMapSlice),
        ("path", "/xlerobot_test/plan", NavPath),
    ):
        node.create_subscription(msg_type, topic, lambda m, key=key: data.update({key: m}), qos_profile_sensor_data)
    client = ActionClient(node, NavigateToPose, "/xlerobot_test/navigate_to_pose")
    lifecycle = node.create_client(GetState, "/xlerobot_test/bt_navigator/get_state")
    state_request = None

    def poll_state():
        nonlocal state_request
        if state_request is not None:
            if not state_request.done():
                return
            data["active"] = state_request.result().current_state.id == 3
        if lifecycle.service_is_ready():
            state_request = lifecycle.call_async(GetState.Request())

    node.create_timer(0.5, poll_state)

    def wait(predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("Navigation launch exited; inspect log/navigation-test.log")
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f"Timed out: {label}; received {list(data)}")

    def goal(x, y, angle=math.pi):
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = "map"
        request.pose.header.stamp = node.get_clock().now().to_msg()
        request.pose.pose.position.x = float(x)
        request.pose.pose.position.y = float(y)
        request.pose.pose.orientation.z = math.sin(angle / 2)
        request.pose.pose.orientation.w = math.cos(angle / 2)
        future = client.send_goal_async(request)
        wait(future.done, 15, "goal acceptance")
        handle = future.result()
        assert handle.accepted, "Nav2 rejected the action request"
        return handle

    try:
        wait(lambda: data.get("active") and client.server_is_ready()
             and all(k in data for k in ("pose", "slice", "odom")),
             300, "live mapping and navigation startup")
        distances = data["slice"]
        assert distances.header.frame_id == "map"
        known = [d for d in distances.data if d != distances.unknown_value and math.isfinite(d)]
        assert known and min(known) < 0.1 and max(known) > 0.5, "No reconstructed obstacles/free space"
        print(f"PASS: nvblox reconstructed {len(known)} known map cells", flush=True)
        start = data["pose"].pose.position
        handle = goal(-1.1, 0.0)
        result = handle.get_result_async()
        wait(result.done, 240, "reach visible goal")
        response = result.result()
        assert response.status == GoalStatus.STATUS_SUCCEEDED, f"Navigation failed: {response}"
        finish = data["pose"].pose.position
        error = math.hypot(finish.x + 1.1, finish.y)
        travel = math.hypot(finish.x - start.x, finish.y - start.y)
        assert error < 0.20 and travel > 0.65, (error, travel)
        assert len(data["path"].poses) > 2
        print(f"PASS: planned and drove {travel:.2f} m; endpoint error {error:.3f} m", flush=True)
        blocked = goal(-2.4, 0.0).get_result_async()  # center of the reconstructed box
        wait(blocked.done, 30, "reject occupied goal")
        assert blocked.result().status == GoalStatus.STATUS_ABORTED, blocked.result()
        print("PASS: goal inside obstacle aborted", flush=True)
        handle = goal(-0.3, 0.0, 0.0)
        canceled = handle.cancel_goal_async()
        wait(canceled.done, 10, "cancel goal")
        result = handle.get_result_async()
        wait(result.done, 10, "canceled result")
        assert result.result().status == GoalStatus.STATUS_CANCELED, result.result()
        wait(lambda: abs(data["odom"].twist.twist.linear.x) < 0.015
             and abs(data["odom"].twist.twist.angular.z) < 0.03, 5, "stop after cancel")
        print("PASS: cancellation stops the base", flush=True)
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--rviz", action="store_true", help="also open the configured RViz goal interface")
    parser.add_argument("--domain-id", type=int, default=96)
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 101:
        parser.error("domain-id must be in 0..101")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    workspace = Path(__file__).resolve().parents[1]
    log_path = workspace / "log/navigation-test.log"
    log_path.parent.mkdir(exist_ok=True)
    with log_path.open("w") as log:
        process = subprocess.Popen([
            "ros2", "launch", "xlerobot_navigation", "sim_navigation.launch.py",
            "namespace:=xlerobot_test", f"rviz:={str(args.rviz).lower()}", "foxglove:=false",
            f"headless:={str(not args.gui).lower()}",
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            check(process)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=40)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=10)


if __name__ == "__main__":
    main()
