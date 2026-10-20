"""Wall-clock serial transport; controllers and TF live in the shared stack."""
from pathlib import Path
import time
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
import yaml

from .config import load_config
from .transport import CommandBuffer, ServoTransport


class ServoBridge(Node):
    def __init__(self):
        super().__init__("servo_bridge")
        self.declare_parameter("hardware_config", "")
        if self.get_parameter("use_sim_time").value:
            raise ValueError("Physical hardware requires use_sim_time:=false")
        model = Path(get_package_share_directory("xlerobot_description")) / "config/joints.yaml"
        buses, self.joints, rate, timeout = load_config(
            self.get_parameter("hardware_config").value, yaml.safe_load(model.read_text())["joints"])
        self.commands = CommandBuffer(self.joints, timeout)
        self.feedback = self.create_publisher(JointState, "backend/joint_states", qos_profile_sensor_data)
        for mode, field in (("position", "position"), ("velocity", "velocity")):
            self.create_subscription(JointState, f"backend/{mode}_command",
                lambda msg, mode=mode, field=field: self.commands.receive(
                    mode, msg.name, getattr(msg, field), time.monotonic()), 1)
        self.transport = ServoTransport(buses, self.joints)
        self.create_timer(1.0 / rate, self.cycle, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info("Calibrated feedback ready; waiting for both controller command streams")

    def cycle(self):
        if self.transport.enabled and not self.commands.fresh(time.monotonic()):
            raise RuntimeError("Controller command watchdog expired")
        positions, velocities = self.transport.read()
        message = JointState(name=[joint.name for joint in self.joints], position=positions, velocity=velocities)
        message.header.stamp = self.get_clock().now().to_msg()
        self.feedback.publish(message)
        if self.commands.fresh(time.monotonic()):
            if not self.transport.enabled:
                self.transport.activate()
                self.get_logger().info("Physical transport active")
            self.transport.write(self.commands.commands["position"], self.commands.commands["velocity"])


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            for error in node.transport.close():
                node.get_logger().error(f"Stop delivery failed: {error}")
            node.destroy_node()
        rclpy.try_shutdown()
