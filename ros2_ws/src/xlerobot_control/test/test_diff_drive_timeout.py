"""Check the compiled controller's braking and warning transitions on mock wheels."""

import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import (
    ConfigureController, ListHardwareComponents, LoadController, SwitchController,
)
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.context import Context
from rclpy.duration import Duration as ClockDuration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
import xacro


def test_timeout_warns_once_brakes_and_rearms(tmp_path):
    namespace = "timeout_test"
    share = Path(get_package_share_directory("xlerobot_description"))
    description = xacro.process_file(
        str(share / "urdf/robot.urdf.xacro"),
        mappings={"backend": "mock", "namespace": namespace}).toxml()
    config = Path(get_package_share_directory("xlerobot_control")) / "config/controllers.yaml"
    binary = Path(get_package_prefix("controller_manager")) / "lib/controller_manager/ros2_control_node"
    context = Context()
    rclpy.init(context=context, domain_id=95)
    node = Node("timeout_test_peer", namespace=namespace, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    description_pub = node.create_publisher(
        String, "robot_description", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    commands = node.create_publisher(TwistStamped, "cmd_vel", 1)
    odometry = []
    node.create_subscription(Odometry, "odom", lambda msg: odometry.append(msg), 10)
    log_path = tmp_path / "manager.log"
    with log_path.open("w") as log:
        process = subprocess.Popen(
            [str(binary), "--ros-args", "-r", f"__ns:=/{namespace}",
             "--params-file", str(config)],
            env={**os.environ, "ROS_DOMAIN_ID": "95"}, stdout=log, stderr=subprocess.STDOUT)

        def spin(seconds, publish=None):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                assert process.poll() is None, log_path.read_text()
                if publish:
                    publish()
                executor.spin_once(timeout_sec=0.02)

        def call(service_type, name, request):
            client = node.create_client(service_type, f"controller_manager/{name}")
            try:
                assert client.wait_for_service(timeout_sec=15), log_path.read_text()
                future = client.call_async(request)
                end = time.monotonic() + 10
                while not future.done() and time.monotonic() < end:
                    spin(0.02)
                assert future.done(), log_path.read_text()
                return future.result()
            finally:
                node.destroy_client(client)

        def switch(activate=(), deactivate=()):
            result = call(SwitchController, "switch_controller", SwitchController.Request(
                activate_controllers=list(activate), deactivate_controllers=list(deactivate),
                strictness=SwitchController.Request.STRICT, timeout=Duration(sec=5)))
            assert result.ok, log_path.read_text()

        def warning_count():
            return log_path.read_text().count("Velocity command timed out. Braking.")

        def drive():
            message = TwistStamped()
            message.header.stamp = node.get_clock().now().to_msg()
            message.twist.linear.x = 0.04
            commands.publish(message)

        def speed():
            assert odometry, log_path.read_text()
            return odometry[-1].twist.twist.linear.x

        try:
            spin(1.0, lambda: description_pub.publish(String(data=description)))
            hardware = call(ListHardwareComponents, "list_hardware_components",
                            ListHardwareComponents.Request())
            assert hardware.component and hardware.component[0].state.label == "active"
            assert call(LoadController, "load_controller",
                        LoadController.Request(name="base_controller")).ok, log_path.read_text()
            assert call(ConfigureController, "configure_controller",
                        ConfigureController.Request(name="base_controller")).ok, log_path.read_text()
            switch(activate=["base_controller"])

            # Initial idle times out once, even after several original warning intervals.
            spin(2.5)
            assert warning_count() == 1, log_path.read_text()
            assert abs(speed()) < 1e-6

            for expected_count in (2, 3):
                spin(1.0, drive)
                assert speed() > 0.03, "Fresh commands must still drive the wheels"
                assert warning_count() == expected_count - 1, log_path.read_text()
                drive()
                spin(0.25)
                assert speed() > 0.03, "A command must remain valid before its 0.5 s timeout"
                spin(2.25)
                assert abs(speed()) < 1e-6, "Stale commands must still brake the wheels"
                assert warning_count() == expected_count, log_path.read_text()

            # A rejected old message cannot rearm the timeout warning or move the base.
            stale = TwistStamped()
            stale.header.stamp = (node.get_clock().now() - ClockDuration(seconds=2)).to_msg()
            stale.twist.linear.x = 0.1
            commands.publish(stale)
            spin(1.2)
            assert abs(speed()) < 1e-6
            assert warning_count() == 3, log_path.read_text()

            # Lifecycle restart begins a new timeout episode.
            switch(deactivate=["base_controller"])
            switch(activate=["base_controller"])
            spin(2.5)
            assert abs(speed()) < 1e-6
            assert warning_count() == 4, log_path.read_text()
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            executor.shutdown()
            node.destroy_node()
            context.shutdown()
