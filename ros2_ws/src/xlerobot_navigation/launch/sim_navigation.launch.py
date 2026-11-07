"""Isaac robot, live nvblox mapping, Nav2, and RViz clicked goals."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import ReplaceString


def check_agent_interfaces():
    try:
        from rclpy.type_support import check_for_type_support
        from xlerobot_navigation_interfaces.action import Navigate
        check_for_type_support(Navigate)
    except (ImportError, RuntimeError) as error:
        raise RuntimeError("Agent navigation interfaces are not available in this terminal. "
                           "From ros2_ws, run: source scripts/env.sh. If the package has not been built, "
                           "run ./scripts/build.sh --packages-select xlerobot_navigation_interfaces "
                           "xlerobot_navigation xlerobot_agent xlerobot_bringup, then source scripts/env.sh again.") from error


def stop_launch(context, reason):
    if context.is_shutdown or context.launch_configurations.get("xlerobot_shutdown_requested"):
        return []
    context.launch_configurations["xlerobot_shutdown_requested"] = "true"
    return [EmitEvent(event=Shutdown(reason=reason))]


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    namespace = value("namespace").strip("/")
    agent_navigation = value("agent_navigation") == "true"
    if agent_navigation:
        check_agent_interfaces()
    if agent_navigation and (value("yoloe") != "true" or value("object_map") != "true"):
        raise RuntimeError("agent_navigation requires yoloe and object_map for map-session tracking")
    share = Path(get_package_share_directory("xlerobot_navigation"))
    bringup = Path(get_package_share_directory("xlerobot_bringup"))
    # Resolve dependencies before starting a potentially expensive simulation.
    for package in ("nvblox_ros", "nvblox_nav2", "nav2_bt_navigator",
                    "nav2_smac_planner", "nav2_regulated_pure_pursuit_controller",
                    "isaac_ros_visual_slam"):
        get_package_share_directory(package)
    params = ReplaceString(source_file=str(share / "config/nav2.yaml"),
                           replacements={"__NS__": namespace})
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / "launch/robot.launch.py")),
        launch_arguments={"backend": "isaac", "namespace": namespace,
                          "sensors": "true", "navigation_scene": "true",
                          "controller_service_timeout": "20",
                          "exclusive_navigation": value("agent_navigation"),
                          "headless": value("headless"), "rviz": "false",
                          "foxglove": value("foxglove")}.items())]

    if value("yoloe") == "true":
        perception = Path(get_package_share_directory("xlerobot_perception"))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(perception / "launch/yoloe.launch.py")),
            launch_arguments={"namespace": namespace, "use_sim_time": "true",
                              "image_topic": value("yoloe_image_topic"),
                              "visualize": value("yoloe_visualize"),
                              "object_map": value("object_map"),
                              "object_map_config": value("object_map_config"),
                              "object_database": value("object_database"),
                              "depth_topic": value("object_depth_topic"),
                              "camera_info_topic": value("object_camera_info_topic"),
                              "config": value("yoloe_config")}.items()))

    def critical(node, label):
        actions.extend([RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=lambda _event, ctx: stop_launch(ctx, f"{label} exited"))), node])

    critical(ComposableNodeContainer(
        name="cuvslam_container", namespace=namespace, package="rclcpp_components",
        executable="component_container", output="screen",
        composable_node_descriptions=[ComposableNode(
            name="visual_slam", namespace=namespace, package="isaac_ros_visual_slam",
            plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
            parameters=[str(share / "config/cuvslam.yaml")],
            remappings=[("visual_slam/image_0", "/camera/color/image_raw"),
                        ("visual_slam/camera_info_0", "/camera/color/camera_info"),
                        ("visual_slam/depth_0", "cuvslam/aligned_depth")])]), "cuVSLAM")
    critical(Node(package="xlerobot_navigation", executable="cuvslam_depth_conversion",
                  namespace=namespace, parameters=[{"use_sim_time": True}], output="screen"),
             "cuVSLAM depth conversion")
    critical(Node(package="xlerobot_navigation", executable="vslam_localization", namespace=namespace,
                  parameters=[{"use_sim_time": True}], output="screen"), "Localization")
    critical(Node(package="xlerobot_navigation", executable="navigation_guard", namespace=namespace,
                  parameters=[{"use_sim_time": True, "require_navigation_permit": agent_navigation}],
                  remappings=[("cmd_vel", "cmd_vel_guarded")] if agent_navigation else [],
                  output="screen"), "Navigation guard")
    if agent_navigation:
        critical(Node(package="xlerobot_navigation", executable="navigation_supervisor", namespace=namespace,
                      parameters=[{"use_sim_time": True}], output="screen"), "Navigation supervisor")
    critical(Node(package="nvblox_ros", executable="nvblox_node", name="nvblox_node", namespace=namespace,
                  parameters=[str(share / "config/nvblox.yaml")],
                  remappings=[("camera_0/depth/image", "/camera/depth/image_rect_raw"),
                              ("camera_0/depth/camera_info", "/camera/depth/camera_info"),
                              ("camera_0/color/image", "/camera/color/image_raw"),
                              ("camera_0/color/camera_info", "/camera/color/camera_info")],
                  output="screen"), "nvblox")
    servers = [("nav2_controller", "controller_server"),
               ("nav2_planner", "planner_server"),
               ("nav2_behaviors", "behavior_server"),
               ("nav2_bt_navigator", "bt_navigator")]
    for package, executable in servers:
        overrides = {"use_sim_time": True}
        if executable == "bt_navigator":
            overrides["default_nav_to_pose_bt_xml"] = str(share / "behavior_trees/navigate.xml")
        remappings = [("cmd_vel", "cmd_vel_navigation")]
        if agent_navigation:
            remappings.append(("navigate_to_pose", "navigate_to_pose_internal"))
        critical(Node(package=package, executable=executable, name=executable, namespace=namespace,
                      parameters=[params, overrides],
                      remappings=remappings, output="screen"), executable)
    manager = Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                   name="lifecycle_manager_navigation", namespace=namespace,
                   parameters=[{"use_sim_time": True, "autostart": True,
                                "node_names": [name for _, name in servers],
                                "bond_timeout": 10.0}], output="screen")
    ready = Node(package="xlerobot_navigation", executable="wait_for_mapping", namespace=namespace,
                 parameters=[{"use_sim_time": True}], output="screen")

    def activate(event, _context):
        if _context.is_shutdown or _context.launch_configurations.get("xlerobot_shutdown_requested"):
            return []
        if event.returncode != 0:
            return stop_launch(_context, "Live mapping did not become ready")
        return [manager]

    actions.extend([RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=activate)), ready])
    if value("rviz") == "true":
        rviz_config = ReplaceString(source_file=str(share / "rviz/navigation.rviz"),
                                    replacements={"__NS__": namespace})
        actions.append(Node(package="rviz2", executable="rviz2", namespace=namespace,
                            parameters=[{"use_sim_time": True}],
                            arguments=["-d", rviz_config], output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="xlerobot"),
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("agent_navigation", default_value="false", choices=["true", "false"],
                              description="Exclusive supervised agent motion; disables manual velocity and RViz goals"),
        DeclareLaunchArgument("foxglove", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("yoloe", default_value="true", choices=["true", "false"],
                              description="Start YOLOE detection with live vocabulary and class segmentation"),
        DeclareLaunchArgument("yoloe_image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("yoloe_visualize", default_value="true", choices=["true", "false"],
                              description="Run the separate Foxglove segmentation renderer"),
        DeclareLaunchArgument("yoloe_config", default_value=str(
            Path(get_package_share_directory("xlerobot_perception")) / "config/yoloe.yaml")),
        DeclareLaunchArgument("object_map", default_value="true", choices=["true", "false"],
                              description="Track and store segmented objects in SQLite (requires yoloe)"),
        DeclareLaunchArgument("object_map_config", default_value=str(
            Path(get_package_share_directory("xlerobot_perception")) / "config/object_mapping.yaml")),
        DeclareLaunchArgument("object_database", default_value=os.environ.get(
            "XLEROBOT_OBJECT_DATABASE", str(Path.home() / ".ros/xlerobot/objects.sqlite3"))),
        DeclareLaunchArgument("object_depth_topic", default_value="/camera/aligned_depth_to_color/image_raw_float"),
        DeclareLaunchArgument("object_camera_info_topic", default_value="/camera/color/camera_info"),
        OpaqueFunction(function=setup),
    ])
