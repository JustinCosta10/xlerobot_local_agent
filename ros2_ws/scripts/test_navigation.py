#!/usr/bin/env python3
"""Exercise actual Isaac depth -> nvblox -> Nav2 -> wheel motion, without real motors."""

import argparse
from collections import deque
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from nvblox_msgs.msg import DistanceMapSlice
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState


def check_wheel_feedback(messages, label):
    """Compare native velocity feedback with encoder displacement, including wraps."""
    # Clock publishing can stamp two physics updates identically. Retain the
    # last joint sample at each stamp before integrating measured velocities.
    unique = {m.header.stamp.sec + m.header.stamp.nanosec * 1e-9: m for m in messages}
    times = sorted(unique)
    assert len(times) > 30 and times[-1] - times[0] > 2, "Insufficient wheel feedback samples"
    errors = []
    for name in ("left_wheel_joint", "right_wheel_joint"):
        positions, velocities = [], []
        for stamp in times:
            message = unique[stamp]
            index = message.name.index(name)
            positions.append(message.position[index])
            velocities.append(message.velocity[index])
        # Both 2pi and Isaac's 4pi angle wraps leave the physical increment intact.
        delta = np.diff(positions)
        displacement = np.arctan2(np.sin(delta), np.cos(delta)).sum()
        velocity_integral = sum(.5 * (a+b) * dt for a,b,dt in
                                zip(velocities, velocities[1:], np.diff(times)))
        error = abs(velocity_integral-displacement) / (times[-1]-times[0])
        errors.append(error)
        assert error < .03, f"{label}: {name} velocity/encoder disagreement {error:.4f} rad/s"
    print(f"PASS: {label} wheel velocity agrees with encoder motion; "
          f"maximum mean error {max(errors):.4f} rad/s", flush=True)


def check(process, idle_seconds):
    rclpy.init()
    node = rclpy.create_node("navigation_acceptance", parameter_overrides=[Parameter("use_sim_time", value=True)])
    data = {}
    pose_stamps = deque(maxlen=12000)
    turn_samples = None

    def receive(key, message):
        data[key] = message
        if key == "estimate":
            pose_stamps.append(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)
        if turn_samples is not None and key in turn_samples:
            turn_samples[key].append(message)
    for key, topic, msg_type in (
        ("pose", "/xlerobot_test/sim_ground_truth", PoseStamped),
        ("estimate", "/xlerobot_test/localization/pose", PoseStamped),
        ("odom", "/xlerobot_test/odom", Odometry),
        ("joints", "/xlerobot_test/joint_states", JointState),
        ("slice", "/xlerobot_test/nvblox_node/static_map_slice", DistanceMapSlice),
        ("path", "/xlerobot_test/plan", NavPath),
    ):
        node.create_subscription(msg_type, topic, lambda m, key=key: receive(key, m), 10)
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

    world_to_map = (0.0, 0.0, 0.0)

    def mapped(x, y):
        dx, dy, heading = world_to_map
        return (dx + math.cos(heading) * x - math.sin(heading) * y,
                dy + math.sin(heading) * x + math.cos(heading) * y)

    def yaw(pose):
        q = pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))

    def goal(x, y, angle=math.pi):
        x, y = mapped(x, y)
        angle += world_to_map[2]
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

    def turn_speed(speed):
        setter = node.create_client(SetParameters, "/xlerobot_test/controller_server/set_parameters")
        try:
            wait(setter.service_is_ready, 10, "controller parameter service")
            request = SetParameters.Request(parameters=[Parameter(
                "FollowPath.rotate_to_heading_angular_vel", value=speed).to_parameter_msg()])
            future = setter.call_async(request)
            wait(future.done, 10, "set isolated test turn speed")
            assert all(r.successful for r in future.result().results), future.result()
        finally:
            node.destroy_client(setter)

    try:
        wait(lambda: data.get("active") and client.server_is_ready()
             and all(k in data for k in ("pose", "estimate", "slice", "odom")),
             300, "live mapping and navigation startup")
        truth_consumers = node.get_subscriptions_info_by_topic("/xlerobot_test/sim_ground_truth")
        assert all(info.node_name == "navigation_acceptance" for info in truth_consumers), \
            "Ground truth must only be consumed by this independent test observer"
        # Ground truth is used ONLY by this observer, never by localization.
        actual, estimate = data["pose"].pose, data["estimate"].pose
        heading = yaw(estimate) - yaw(actual)
        c, s = math.cos(heading), math.sin(heading)
        world_to_map = (estimate.position.x - c*actual.position.x + s*actual.position.y,
                        estimate.position.y - s*actual.position.x - c*actual.position.y, heading)
        print("PASS: cuVSLAM localization ready; ground truth only consumed by test observer", flush=True)
        idle_start = node.get_clock().now().nanoseconds * 1e-9
        idle_pose = data["estimate"].pose.position
        turn_samples = {key: [] for key in ("pose", "estimate", "odom", "joints")}
        pose_stamps.clear()
        wait(lambda: node.get_clock().now().nanoseconds * 1e-9 >= idle_start + idle_seconds,
             max(60, idle_seconds * 6), "stationary visual tracking")
        current = data["estimate"].pose.position
        drift = math.sqrt((current.x-idle_pose.x)**2 + (current.y-idle_pose.y)**2 +
                          (current.z-idle_pose.z)**2)
        deltas = sorted(1000*(b-a) for a,b in zip(pose_stamps, list(pose_stamps)[1:]) if b > a)
        assert len(deltas) >= 30, "Too few visual poses for camera timing check"
        p95 = deltas[int(.95*(len(deltas)-1))]
        print(f"Stationary for {idle_seconds:g} sim seconds: drift {drift:.3f} m; "
              f"visual frame intervals median {statistics.median(deltas):.1f}, "
              f"p95 {p95:.1f}, max {max(deltas):.1f} ms", flush=True)
        assert drift < .10, f"Stationary visual drift: {drift:.3f} m"
        assert p95 <= 50, f"Visual frames exceed the 50 ms threshold: p95 {p95:.1f} ms"
        stationary, turn_samples = turn_samples, None
        check_wheel_feedback(stationary["joints"], "Stationary")
        mean_v = statistics.mean(m.twist.twist.linear.x for m in stationary["odom"])
        mean_w = statistics.mean(m.twist.twist.angular.z for m in stationary["odom"])
        assert abs(mean_v) < .003 and abs(mean_w) < .005, \
            f"Stationary odometry bias: {mean_v:.4f} m/s, {mean_w:.4f} rad/s"
        print(f"PASS: stationary odometry bias {mean_v:.4f} m/s, {mean_w:.4f} rad/s", flush=True)
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
        estimated = data["estimate"].pose.position
        expected_x, expected_y = mapped(finish.x, finish.y)
        localization_error = math.hypot(estimated.x - expected_x, estimated.y - expected_y)
        print(f"Measured travel {travel:.3f} m, goal error {error:.3f} m, "
              f"localization error {localization_error:.3f} m", flush=True)
        assert error < 0.20 and travel > 0.65, (error, travel)
        assert len(data["path"].poses) > 2
        print(f"PASS: planned and drove {travel:.2f} m; endpoint error {error:.3f} m", flush=True)
        assert localization_error < .20, f"Localization drift: {localization_error:.3f} m"
        print(f"PASS: localization error {localization_error:.3f} m against independent ground truth", flush=True)
        blocked = goal(-2.4, 0.0).get_result_async()  # center of the reconstructed table
        wait(blocked.done, 30, "reject occupied goal")
        assert blocked.result().status == GoalStatus.STATUS_ABORTED, blocked.result()
        print("PASS: goal inside obstacle aborted", flush=True)
        # Exercise both signs and a long turn; the old small positive turn missed
        # biased wheel feedback that trapped the controller's acceleration limit.
        for offset in (math.pi / 2, -1.0):
            turn_heading = math.pi + offset
            # The negative turn deliberately exceeds the 30-second translation
            # timeout, proving that angular progress keeps the goal alive.
            slow_turn = offset < 0
            turn_speed(.06 if slow_turn else .15)
            turn_samples = {key: [] for key in ("pose", "estimate", "odom", "joints")}
            started_turn = time.monotonic()
            started_sim = node.get_clock().now().nanoseconds * 1e-9
            turned = goal(finish.x, finish.y, turn_heading).get_result_async()
            wait(turned.done, 180, "turn in place")
            assert turned.result().status == GoalStatus.STATUS_SUCCEEDED, turned.result()
            if slow_turn:
                assert node.get_clock().now().nanoseconds * 1e-9 - started_sim > 30, \
                    "Slow turn did not exercise the progress timeout"
            samples, turn_samples = turn_samples, None
            check_wheel_feedback(samples["joints"], f"Turn to {offset:+.2f} rad")
            turn_error = abs(math.atan2(math.sin(yaw(data["pose"].pose) - turn_heading),
                                        math.cos(yaw(data["pose"].pose) - turn_heading)))
            assert turn_error < .25, f"Actual heading error: {turn_error:.3f} rad"
            print(f"PASS: turn to {offset:+.2f} rad completed in {time.monotonic()-started_turn:.1f}s; "
                  f"actual heading error {turn_error:.3f} rad", flush=True)
        turn_speed(.15)
        # From the sideways heading, require alignment followed by forward travel.
        start = data["pose"].pose.position
        started_sim = node.get_clock().now().nanoseconds * 1e-9
        returned = goal(-.3, 0.0, 0.0).get_result_async()
        wait(lambda: math.hypot(data["pose"].pose.position.x-start.x,
                               data["pose"].pose.position.y-start.y) > .08,
             90, "forward travel after initial alignment")
        delay = node.get_clock().now().nanoseconds * 1e-9 - started_sim
        assert delay < 20, f"Initial alignment delayed forward travel by {delay:.1f} sim seconds"
        wait(returned.done, 120, "reach return goal after alignment")
        assert returned.result().status == GoalStatus.STATUS_SUCCEEDED, returned.result()
        print(f"PASS: aligned with path and started forward travel in {delay:.1f} sim seconds", flush=True)
        handle = goal(-1.1, 0.0, math.pi)
        canceled = handle.cancel_goal_async()
        wait(canceled.done, 10, "cancel goal")
        result = handle.get_result_async()
        wait(result.done, 10, "canceled result")
        assert result.result().status == GoalStatus.STATUS_CANCELED, result.result()
        wait(lambda: abs(data["odom"].twist.twist.linear.x) < 0.015
             and abs(data["odom"].twist.twist.angular.z) < 0.03, 5, "stop after cancel")
        print("PASS: cancellation stops the base", flush=True)

        # A short turn neither exercises several wheel revolutions nor observes
        # every wall. Check wrapping and visual drift throughout a complete turn.
        turn_samples = {key: [] for key in ("pose", "estimate", "odom", "joints")}
        drive = node.create_publisher(TwistStamped, "/xlerobot_test/cmd_vel", 10)
        last_heading = yaw(data["pose"].pose)
        accumulated = 0.0
        last_command = 0.0

        def full_turn():
            nonlocal accumulated, last_heading, last_command
            heading = yaw(data["pose"].pose)
            accumulated += math.atan2(math.sin(heading-last_heading), math.cos(heading-last_heading))
            last_heading = heading
            if accumulated >= 2 * math.pi:
                return True
            if time.monotonic() - last_command >= 0.05:
                command = TwistStamped()
                command.header.stamp = node.get_clock().now().to_msg()
                command.twist.angular.z = 0.3
                drive.publish(command)
                last_command = time.monotonic()
            return False

        try:
            wait(full_turn, 180, "measured full 360-degree rotation")
        finally:
            stop = TwistStamped()
            stop.header.stamp = node.get_clock().now().to_msg()
            drive.publish(stop)
        stopped_at = node.get_clock().now().nanoseconds
        wait(lambda: node.get_clock().now().nanoseconds - stopped_at >= 3_000_000_000,
             30, "settle after full rotation")
        samples, turn_samples = turn_samples, None
        check_wheel_feedback(samples["joints"], "Full rotation")
        assert len(samples["odom"]) > 100 and len(samples["joints"]) > 100, \
            "Insufficient wheel feedback during full rotation"

        def stamp(message):
            return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

        truth = samples["pose"]
        times = [stamp(m) for m in truth]
        estimates = [m for m in samples["estimate"] if times[0] <= stamp(m) <= times[-1]]
        assert len(estimates) > 100, "Too few visual poses during full rotation"
        assert stamp(estimates[-1]) - stamp(estimates[0]) > 0.9 * (times[-1] - times[0]), \
            "Localization did not cover the full rotation"
        gaps = np.diff([stamp(m) for m in estimates])
        assert max(gaps) < 0.2, f"Localization stalled during rotation: {max(gaps):.3f} s"
        x = np.interp([stamp(m) for m in estimates], times, [m.pose.position.x for m in truth])
        y = np.interp([stamp(m) for m in estimates], times, [m.pose.position.y for m in truth])
        headings = np.interp([stamp(m) for m in estimates], times,
                             np.unwrap([yaw(m.pose) for m in truth]))
        errors, heading_errors = [], []
        for message, tx, ty, heading in zip(estimates, x, y, headings):
            ex, ey = mapped(tx, ty)
            errors.append(math.hypot(message.pose.position.x-ex, message.pose.position.y-ey))
            angle = yaw(message.pose) - heading - world_to_map[2]
            heading_errors.append(abs(math.atan2(math.sin(angle), math.cos(angle))))
        assert max(errors) < .15, f"Full-turn localization error: {max(errors):.3f} m"
        assert max(heading_errors) < .15, f"Full-turn heading error: {max(heading_errors):.3f} rad"
        for before, after in zip(samples["odom"], samples["odom"][1:]):
            dt = stamp(after) - stamp(before)
            # Isaac can publish two controller samples within one clock tick.
            # Still check their pose increments; duplicate stamps cannot hide jumps.
            assert 0 <= dt < .2, f"Invalid or missing wheel odometry: interval {dt:.3f} s"
            p, q = before.pose.pose, after.pose.pose
            # Generous margins for measured wheel jitter, but reject wrap jumps.
            assert math.hypot(q.position.x-p.position.x, q.position.y-p.position.y) < .03 + .2*dt, \
                "Wheel odometry position jumped during rotation"
            angle = yaw(q) - yaw(p)
            assert abs(math.atan2(math.sin(angle), math.cos(angle))) < .1 + .7*dt, \
                "Wheel odometry heading jumped during rotation"
        wraps = 0
        for name in ("left_wheel_joint", "right_wheel_joint"):
            positions = [m.position[m.name.index(name)] for m in samples["joints"]]
            wraps += sum(abs(b-a) > 6 for a, b in zip(positions, positions[1:]))
        print(f"PASS: full 360-degree rotation; max localization error {max(errors):.3f} m, "
              f"heading error {max(heading_errors):.3f} rad; continuous odometry across "
              f"{wraps} raw wheel-angle wraps", flush=True)
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--rviz", action="store_true", help="also open the configured RViz goal interface")
    parser.add_argument("--domain-id", type=int, default=96)
    parser.add_argument("--idle-seconds", type=float, default=15,
                        help="Simulation seconds to check stationary cuVSLAM drift and frame timing")
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 101:
        parser.error("domain-id must be in 0..101")
    if args.idle_seconds < 2:
        parser.error("idle-seconds must be at least 2")
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
            check(process, args.idle_seconds)
            log.flush()
            assert "Failed to make progress" not in log_path.read_text(), \
                "Navigation retried after a progress failure; inspect log/navigation-test.log"
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=40)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=10)
            log.flush()


if __name__ == "__main__":
    main()
