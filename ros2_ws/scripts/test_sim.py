#!/usr/bin/env python3
"""Exercise the same ROS interfaces against Isaac or mock.

This script never selects a real physical device. --check-motion sends a head
trajectory, restores its starting pose, and verifies wheel motion and timeout.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=["isaac", "mock"], default="isaac")
    parser.add_argument("--check-motion", action="store_true")
    parser.add_argument("--domain-id", type=int, default=91)
    parser.add_argument("--sensors", choices=["true", "false"], default="false")
    parser.add_argument("--gui", action="store_true", help="show Isaac and its Action Graph editor")
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 101:
        parser.error("--domain-id must be between 0 and 101")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    workspace = Path(__file__).resolve().parents[1]
    log_path = workspace / f"log/{args.backend}-test.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    arguments = [f"backend:={args.backend}"]
    args.state_log = workspace / "log/isaac-physics-state.jsonl"
    if args.backend == "isaac":
        args.state_log.unlink(missing_ok=True)
        arguments += [f"state_log:={args.state_log}"]
    print(f"Launch output: {log_path}", flush=True)
    with log_path.open("w") as log:
        process = subprocess.Popen([
            "ros2", "launch", "xlerobot_bringup", "robot.launch.py",
            f"headless:={str(not args.gui).lower()}", f"sensors:={args.sensors}",
            "namespace:=xlerobot_test", "foxglove:=false", *arguments,
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            check_feedback(args, process)
        finally:
            if process.poll() is None:
                # Signal launch once; it forwards SIGINT to its children.
                # Signalling the whole group too can interrupt cleanup twice.
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=35)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)


def check_feedback(args, process):
    rclpy.init(args=[])
    node = rclpy.create_node("interface_acceptance_test", parameter_overrides=[
        Parameter("use_sim_time", value=args.backend == "isaac")])
    received = {"diagnostic_count": 0, "frames": set()}
    wheel_travel, previous_wheels = {}, {}

    def on_joints(message):
        for name in ("left_wheel_joint", "right_wheel_joint"):
            if name in message.name:
                angle = message.position[message.name.index(name)]
                if name in previous_wheels:
                    wheel_travel[name] += math.remainder(angle - previous_wheels[name], 2 * math.pi)
                else:
                    wheel_travel[name] = 0.0
                previous_wheels[name] = angle
        received["joints"] = message

    node.create_subscription(JointState, "/xlerobot_test/joint_states",
                             on_joints, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/xlerobot_test/odom",
                             lambda msg: received.update(odom=msg), qos_profile_sensor_data)
    node.create_subscription(TFMessage, "/tf", lambda msg: received["frames"].update(
        (tf.header.frame_id, tf.child_frame_id) for tf in msg.transforms), qos_profile_sensor_data)
    node.create_subscription(TFMessage, "/tf_static", lambda msg: received["frames"].update(
        (tf.header.frame_id, tf.child_frame_id) for tf in msg.transforms),
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    if args.backend == "isaac" and args.sensors == "true":
        for key, topic, kind in (
                ("color", "/camera/color/image_raw", Image),
                ("depth", "/camera/depth/image_rect_raw", Image),
                ("color_info", "/camera/color/camera_info", CameraInfo),
                ("depth_info", "/camera/depth/camera_info", CameraInfo),
                ("points", "/camera/depth/points", PointCloud2)):
            node.create_subscription(kind, topic, lambda msg, key=key: received.update({key: msg}),
                                     qos_profile_sensor_data)

    def on_diagnostics(msg):
        for status in msg.status:
            if status.name == "/xlerobot_test/robot_interface":
                received["status"] = status
                received["diagnostic_count"] += 1

    node.create_subscription(DiagnosticArray, "/diagnostics", on_diagnostics, 10)
    action = ActionClient(node, FollowJointTrajectory, "/xlerobot_test/head_controller/follow_joint_trajectory")
    base = node.create_publisher(TwistStamped, "/xlerobot_test/cmd_vel", 10)
    plain_base = node.create_publisher(Twist, "/cmd_vel", 10)
    controllers = node.create_client(ListControllers, "/xlerobot_test/controller_manager/list_controllers")

    def wait_for(predicate, timeout=15.0, publish=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("Robot launch exited; see the test log")
            if publish:
                publish()
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return
        wheels = {name: position(name) for name in ("left_wheel_joint", "right_wheel_joint")} if "joints" in received else {}
        raise RuntimeError(f"Timed out waiting for interface: {received.get('status')}; "
                           f"wheels={wheels}")

    def position(joint):
        state = received["joints"]
        return state.position[state.name.index(joint)]

    def trajectory(target):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["head_pan_joint", "head_tilt_joint"]
        goal.trajectory.points = [JointTrajectoryPoint(positions=target, velocities=[0.0, 0.0],
                                                      time_from_start=Duration(sec=2))]
        future = action.send_goal_async(goal)
        wait_for(future.done)
        handle = future.result()
        assert handle.accepted, "Head trajectory was rejected"
        result = handle.get_result_async()
        wait_for(result.done, timeout=25)
        response = result.result()
        assert response.status == GoalStatus.STATUS_SUCCEEDED, str(response.result)
        assert response.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        wait_for(lambda: all(abs(position(name) - expected) < 0.03
                            for name, expected in zip(goal.trajectory.joint_names, target)))

    def send_base(speed, plain=False, yaw=0.0):
        if plain:
            message = Twist()
            message.linear.x = speed
            message.angular.z = yaw
            plain_base.publish(message)
            return
        message = TwistStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.twist.linear.x = speed
        message.twist.angular.z = yaw
        base.publish(message)

    def physics_pose():
        samples = args.state_log.read_text().splitlines()
        # Ignore a partially written last line while Isaac is appending.
        for line in reversed(samples):
            try:
                return json.loads(line)["body"]
            except json.JSONDecodeError:
                continue
        raise AssertionError("No independent physics measurements")

    def heading(pose):
        x, y, z, w = pose[3:]
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    try:
        wait_for(lambda: "joints" in received and "odom" in received
                 and received["diagnostic_count"] >= 2
                 and received["status"].level == DiagnosticStatus.OK
                 and {("odom", "base_link"), ("head_pan_link", "head_tilt_link")}
                     <= received["frames"], timeout=150)
        assert len(received["joints"].name) == 16
        assert ("odom", "base_link") in received["frames"]
        assert ("head_pan_link", "head_tilt_link") in received["frames"]
        assert received["odom"].header.frame_id == "odom"
        assert received["odom"].child_frame_id == "base_link"
        wait_for(controllers.service_is_ready)
        result = controllers.call_async(ListControllers.Request())
        wait_for(result.done)
        active = {item.name for item in result.result().controller if item.state == "active"}
        assert active == {"joint_state_broadcaster", "left_arm_controller", "right_arm_controller",
                          "head_controller", "gripper_controller", "base_controller"}, active
        print("PASS: 16 joints, shared TF/odometry, six active controllers, and healthy diagnostics", flush=True)
        if args.backend == "isaac":
            hardware = node.create_client(ListHardwareComponents,
                                          "/xlerobot_test/controller_manager/list_hardware_components")
            wait_for(hardware.service_is_ready)
            result = hardware.call_async(ListHardwareComponents.Request())
            wait_for(result.done)
            components = result.result().component
            assert len(components) == 1, components
            assert components[0].plugin_name == "isaacsim_ros2_control/IsaacSimSystem", components
            assert not node.get_publishers_info_by_topic("/xlerobot_test/backend/velocity_command")
            assert not node.get_publishers_info_by_topic("/xlerobot_test/backend/cmd_vel")
            hardware.destroy()
            print("PASS: native IsaacSimSystem owns the joints; no command transport relay", flush=True)
            if args.sensors == "true":
                camera_messages = ("color", "depth", "color_info", "depth_info", "points")
                camera_frames = {("head_camera_link", "camera_link"),
                                 ("camera_link", "camera_color_frame"),
                                 ("camera_link", "camera_depth_frame"),
                                 ("camera_color_frame", "camera_color_optical_frame"),
                                 ("camera_depth_frame", "camera_depth_optical_frame")}
                wait_for(lambda: all(key in received for key in camera_messages)
                         and camera_frames <= received["frames"], timeout=90)
                for key in camera_messages:
                    assert received[key].header.stamp.sec or received[key].header.stamp.nanosec
                    frame = "camera_color_optical_frame" if key.startswith("color") else "camera_depth_optical_frame"
                    assert received[key].header.frame_id == frame, (key, received[key].header.frame_id)
                for key in ("color", "depth", "color_info", "depth_info"):
                    assert (received[key].width, received[key].height) == (640, 480)
                assert received["color"].data and received["depth"].data and received["points"].data
                clock_publishers = node.get_publishers_info_by_topic("/clock")
                assert len(clock_publishers) == 1, [(p.node_namespace, p.node_name) for p in clock_publishers]
                assert ("chassis", "base_link") not in received["frames"]
                print("PASS: RGB, depth, camera info, point cloud, camera TF, and one clock publisher", flush=True)
            else:
                assert not node.get_publishers_info_by_topic("/camera/color/image_raw")
        if args.check_motion:
            wait_for(action.server_is_ready)
            initial = [position("head_pan_joint"), position("head_tilt_joint")]
            target = [initial[0] + (0.1 if initial[0] < 3.0 else -0.1), initial[1]]
            trajectory(target)
            trajectory(initial)
            print("PASS: same FollowJointTrajectory action moves the head and restores its pose", flush=True)
            for plain, speed, yaw, label in [
                    (True, 0.04, 0.0, "/cmd_vel forward"),
                    (False, 0.04, 0.0, "stamped cmd_vel forward"),
                    (True, -0.04, 0.0, "/cmd_vel reverse"),
                    (False, 0.0, 0.15, "stamped cmd_vel turn")]:
                wheels = ("left_wheel_joint", "right_wheel_joint")
                wheel_start = dict(wheel_travel)
                body_start = physics_pose() if args.backend == "isaac" else None
                start = node.get_clock().now().nanoseconds
                wait_for(lambda: node.get_clock().now().nanoseconds - start >= 5_000_000_000,
                         timeout=45, publish=lambda: send_base(speed, plain=plain, yaw=yaw))
                travel = {joint: wheel_travel[joint] - wheel_start[joint] for joint in wheels}
                # A short initial twitch used to pass this check even when the
                # wheels stalled. Require sustained travel over five seconds.
                expected_sign = (1, -1) if yaw else ((1, 1) if speed > 0 else (-1, -1))
                minimum = 1.5 if yaw else 2.0
                assert all(travel[joint] * sign > minimum for joint, sign in zip(wheels, expected_sign)), \
                    f"{label}: sustained wheel motion failed: {travel}"
                if body_start is not None:
                    body_end = physics_pose()
                    assert abs(body_end[2] - body_start[2]) < 0.04, "Body tipped/sank during driving"
                    if yaw:
                        turn = math.remainder(heading(body_end) - heading(body_start), 2 * math.pi)
                        assert turn > 0.4, f"Wheels moved but body did not turn left: {turn} rad"
                        print(f"  Physics body yaw change: {turn:.3f} rad", flush=True)
                    else:
                        dx, dy = body_end[0] - body_start[0], body_end[1] - body_start[1]
                        angle = heading(body_start)
                        forward = -dx * math.cos(angle) - dy * math.sin(angle)
                        assert forward * math.copysign(1, speed) > 0.12, \
                            f"Wheels moved but body did not travel: {forward} m"
                        print(f"  Physics body travel: {forward:.3f} m; wheels: {travel}", flush=True)
                # The adapter must not repeat old commands and defeat this timeout.
                stopped_after = time.monotonic() + 1.5
                wait_for(lambda: time.monotonic() > stopped_after
                         and abs(received["odom"].twist.twist.linear.x) < 0.01
                         and abs(received["odom"].twist.twist.angular.z) < 0.02)
                print(f"PASS: {label} drives both wheels and stops after command timeout", flush=True)
    finally:
        if rclpy.ok():
            send_base(0.0)
        action.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
