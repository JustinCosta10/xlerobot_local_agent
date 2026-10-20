"""Exercise the compiled plugin through controller_manager, including bus loss."""
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from controller_manager_msgs.srv import ListHardwareComponents
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import xacro
import yaml


def test_measured_startup_complete_feedback_and_latched_stale_error(tmp_path):
    share = Path(get_package_share_directory("xlerobot_description"))
    model = yaml.safe_load((share / "config/joints.yaml").read_text())["joints"]
    description = xacro.process_file(str(share / "urdf/robot.urdf.xacro"),
                                    mappings={"backend": "hardware", "namespace": "transport_test"}).toxml()
    context = Context()
    rclpy.init(context=context, domain_id=94)
    peer = Node("transport_peer", namespace="transport_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(peer)
    description_pub = peer.create_publisher(String, "robot_description",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    feedback = peer.create_publisher(JointState, "backend/joint_states", qos_profile_sensor_data)
    positions, velocities = [], []
    peer.create_subscription(JointState, "backend/position_command", positions.append, 10)
    peer.create_subscription(JointState, "backend/velocity_command", velocities.append, 10)
    hardware = peer.create_client(ListHardwareComponents, "controller_manager/list_hardware_components")
    valid = JointState(name=list(model), position=[0.1] * len(model), velocity=[0.0] * len(model))
    malformed = JointState(name=valid.name[:-1], position=valid.position[:-1], velocity=valid.velocity[:-1])
    env = {**os.environ, "ROS_DOMAIN_ID": "94"}
    log = (tmp_path / "manager.log").open("w")
    binary = Path(get_package_prefix("controller_manager")) / "lib/controller_manager/ros2_control_node"
    process = subprocess.Popen([str(binary),
                                "--ros-args", "-r", "__ns:=/transport_test"],
                               env=env, stdout=log, stderr=subprocess.STDOUT)

    def spin(seconds, message=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            assert process.poll() is None, (tmp_path / "manager.log").read_text()
            description_pub.publish(String(data=description))
            if message:
                feedback.publish(message)
            executor.spin_once(timeout_sec=0.02)

    def components():
        assert hardware.wait_for_service(timeout_sec=3)
        future = hardware.call_async(ListHardwareComponents.Request())
        end = time.monotonic() + 3
        while not future.done() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
        assert future.done()
        return future.result().component

    try:
        spin(1.5, malformed)
        assert not positions and not velocities, "Partial feedback must not activate hardware"
        spin(1.5, valid)
        assert positions and velocities
        assert all(abs(value - 0.1) < 1e-9 for value in positions[-1].position), "Startup must hold measured positions"
        assert list(velocities[-1].velocity) == [0.0, 0.0]
        assert components()[0].state.label == "active"
        spin(1.5, malformed)
        assert components()[0].state.label != "active", "Partial feedback concealed a stale backend"
        count = len(positions)
        spin(0.5, valid)
        assert len(positions) == count, "Fresh feedback must not silently reactivate faulted hardware"
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        log.close()
        executor.shutdown()
        peer.destroy_node()
        context.shutdown()
