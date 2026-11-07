"""Shared robot interfaces; select only the backend and its clock at launch."""

import os
from pathlib import Path
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction
from launch.actions import GroupAction, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace
import xacro
import yaml


def stop_launch(context, reason):
    # Multiple critical processes exit during teardown. Request shutdown once
    # across both the bringup and navigation launch descriptions.
    if context.is_shutdown or context.launch_configurations.get("xlerobot_shutdown_requested"):
        return []
    context.launch_configurations["xlerobot_shutdown_requested"] = "true"
    return [EmitEvent(event=Shutdown(reason=reason))]


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    backend = value("backend")
    namespace = value("namespace").strip("/")
    if not re.fullmatch(r"[A-Za-z_]\w*(/[A-Za-z_]\w*)*", namespace):
        raise RuntimeError("namespace must be a nonempty ROS namespace")
    sim_time = backend == "isaac"
    share = Path(get_package_share_directory("xlerobot_bringup"))
    description_share = Path(get_package_share_directory("xlerobot_description"))
    control_share = Path(get_package_share_directory("xlerobot_control"))
    controllers = str(control_share / "config/controllers.yaml")
    joint_config = yaml.safe_load((description_share / "config/joints.yaml").read_text())["joints"]
    description = xacro.process_file(str(description_share / "urdf/robot.urdf.xacro"),
                                     mappings={"backend": backend, "namespace": namespace}).toxml()
    actions = []
    exclusive = value("exclusive_navigation") == "true"
    cleanup = None
    if exclusive:
        if backend != "isaac" or value("start_sim") != "true":
            raise RuntimeError("Exclusive agent navigation requires this launch to start Isaac")
        config = yaml.safe_load(Path(controllers).read_text())
        args = config["/**/controller_manager"]["ros__parameters"]["base_controller"]["node_options_args"]
        args[args.index("~/cmd_vel:=cmd_vel")] = "~/cmd_vel:=cmd_vel_guarded"
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", prefix="xlerobot-agent-", delete=False) as stream:
            yaml.safe_dump(config, stream)
            controllers = stream.name

        def cleanup(_event, _context):
            Path(controllers).unlink(missing_ok=True)
            return []

    def critical(process, label):
        actions.extend([RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=lambda _event, ctx: stop_launch(ctx, f"{label} exited"))), process])

    if backend == "isaac" and value("start_sim") == "true":
        python = Path(value("isaac_sim_path")) / "python.sh"
        asset = Path(value("asset"))
        if not python.is_file() or not asset.is_file():
            raise RuntimeError("Source scripts/env.sh or set isaac_sim_path:=... and asset:=...")
        actions.append(LogInfo(msg=f"XLeRobot DD asset: {asset.resolve()}"))
        script = Path(get_package_share_directory("xlerobot_isaac")) / "scripts/run_isaac.py"
        command = ["bash", str(script.with_suffix(".sh")), str(python.parent), str(script),
                   "--asset", str(asset.resolve()), "--namespace", namespace,
                   "--description", str(description_share / "urdf/kinematics.urdf"),
                   "--controllers", controllers]
        command += [f"--{option}" for option in ("headless", "sensors") if value(option) == "true"]
        if value("navigation_scene") == "true":
            command.append("--navigation-scene")
        if value("state_log"):
            command += ["--state-log", value("state_log")]
        simulator = ExecuteProcess(cmd=command, output="screen", sigterm_timeout="20", sigkill_timeout="10")
        if cleanup is not None:
            # Isaac reads the YAML after initializing Kit. Keep it until Isaac
            # exits, even if another node fails during that initialization.
            actions.append(RegisterEventHandler(OnProcessExit(target_action=simulator, on_exit=cleanup)))
        critical(simulator, "Isaac Sim")

    critical(Node(package="robot_state_publisher", executable="robot_state_publisher",
                  namespace=namespace, parameters=[{"robot_description": description,
                                                    "use_sim_time": sim_time}], output="screen"),
             "Robot state publisher")
    # Isaac 6.2 hosts its own controller manager through isaacsim.ros2.control.
    # The mock backend uses the external ROS controller manager.
    if backend != "isaac":
        critical(Node(package="controller_manager", executable="ros2_control_node",
                      namespace=namespace, parameters=[controllers, {"use_sim_time": sim_time}],
                      output="screen"), "Controller manager")
    if not exclusive:
        critical(Node(package="xlerobot_control", executable="cmd_vel_adapter", namespace=namespace,
                      parameters=[{"use_sim_time": sim_time}],
                      output="screen"), "cmd_vel adapter")
    # Both managers already load controllers.yaml and pass it to their controllers.
    # Keep remaps there too: the spawner's -p/--controller-ros-args paths use
    # unbounded set_parameters calls, which can hang on a lost DDS response.
    spawner = Node(package="controller_manager", executable="spawner", namespace=namespace,
                   arguments=["joint_state_broadcaster", "left_arm_controller", "right_arm_controller",
                              "head_controller", "gripper_controller", "base_controller",
                              "-c", f"/{namespace}/controller_manager",
                              "--activate-as-group", "--controller-manager-timeout", "150",
                              "--service-call-timeout", value("controller_service_timeout"), "--switch-timeout", "30"],
                   output="screen")

    def check_spawn(event, _context):
        if event.returncode != 0:
            return stop_launch(_context, "Controller activation failed")
        return [LogInfo(msg=("Robot controllers ready: exclusive supervised navigation" if exclusive else
                             f"Robot controllers ready: /cmd_vel (Twist) or /{namespace}/cmd_vel (TwistStamped)"))]

    actions += [RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=check_spawn)), spawner,
                Node(package="xlerobot_diagnostics", executable="bridge_monitor", namespace=namespace,
                     parameters=[str(share / "config/bridge.yaml"), {
                         "use_sim_time": sim_time, "require_clock": sim_time,
                         "expected_joints": list(joint_config)}], output="screen")]
    if value("foxglove") == "true":
        foxglove_share = Path(get_package_share_directory("foxglove_bridge"))
        actions.append(GroupAction([
            PushROSNamespace(namespace),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(str(foxglove_share / "launch/foxglove_bridge_launch.xml")),
                launch_arguments={"address": value("foxglove_address"),
                                  "port": value("foxglove_port"),
                                  "use_sim_time": str(sim_time).lower()}.items()),
        ]))
    if value("rviz") == "true":
        actions.append(Node(package="rviz2", executable="rviz2", namespace=namespace,
                            arguments=["-d", str(share / "rviz/simulation.rviz")],
                            parameters=[{"use_sim_time": sim_time}], output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="isaac", choices=["isaac", "mock"]),
        DeclareLaunchArgument("exclusive_navigation", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("namespace", default_value="xlerobot"),
        DeclareLaunchArgument("isaac_sim_path", default_value=os.environ.get("ISAAC_SIM_PATH", str(Path.home() / "isaacsim"))),
        DeclareLaunchArgument("asset", default_value=os.environ.get("XLEROBOT_ASSET", "")),
        DeclareLaunchArgument("start_sim", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("sensors", default_value="true", choices=["true", "false"],
                              description="Load the D455 and camera publishing graph; false excludes both"),
        DeclareLaunchArgument("navigation_scene", default_value="false", choices=["true", "false"],
                              description="Add navigation obstacles and publish Isaac ground-truth base poses"),
        DeclareLaunchArgument("controller_service_timeout", default_value="150",
                              description="Seconds before retrying a controller-manager service request"),
        DeclareLaunchArgument("foxglove", default_value="true", choices=["true", "false"],
                              description="Start Foxglove Bridge for ROS visualization"),
        DeclareLaunchArgument("foxglove_address", default_value="0.0.0.0",
                              description="Foxglove WebSocket bind address"),
        DeclareLaunchArgument("foxglove_port", default_value="8765",
                              description="Foxglove WebSocket port"),
        DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("state_log", default_value="",
                              description="Optional Isaac physics measurements JSONL path"),
        OpaqueFunction(function=setup),
    ])
