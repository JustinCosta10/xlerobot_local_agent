"""Isaac robot, live nvblox mapping, Nav2, and RViz clicked goals."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    namespace = value("namespace").strip("/")
    share = Path(get_package_share_directory("xlerobot_navigation"))
    bringup = Path(get_package_share_directory("xlerobot_bringup"))
    # Resolve dependencies before starting a potentially expensive simulation.
    for package in ("nvblox_ros", "nvblox_nav2", "nav2_bt_navigator",
                    "nav2_smac_planner", "nav2_regulated_pure_pursuit_controller"):
        get_package_share_directory(package)
    params = ReplaceString(source_file=str(share / "config/nav2.yaml"),
                           replacements={"__NS__": namespace})
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / "launch/robot.launch.py")),
        launch_arguments={"backend": "isaac", "namespace": namespace,
                          "sensors": "true", "navigation_scene": "true",
                          "controller_service_timeout": "20",
                          "headless": value("headless"), "rviz": "false",
                          "foxglove": value("foxglove")}.items())]

    def critical(node, label):
        actions.extend([RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=[EmitEvent(event=Shutdown(reason=f"{label} exited"))])), node])

    critical(Node(package="xlerobot_navigation", executable="sim_localization", namespace=namespace,
                  parameters=[{"use_sim_time": True}], output="screen"), "Simulation localization")
    critical(Node(package="xlerobot_navigation", executable="navigation_guard", namespace=namespace,
                  parameters=[{"use_sim_time": True}], output="screen"), "Navigation guard")
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
        critical(Node(package=package, executable=executable, name=executable, namespace=namespace,
                      parameters=[params, overrides],
                      remappings=[("cmd_vel", "cmd_vel_navigation")], output="screen"), executable)
    manager = Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                   name="lifecycle_manager_navigation", namespace=namespace,
                   parameters=[{"use_sim_time": True, "autostart": True,
                                "node_names": [name for _, name in servers],
                                "bond_timeout": 10.0}], output="screen")
    ready = Node(package="xlerobot_navigation", executable="wait_for_mapping", namespace=namespace,
                 parameters=[{"use_sim_time": True}], output="screen")

    def activate(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="Live mapping did not become ready"))]
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
        DeclareLaunchArgument("foxglove", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=setup),
    ])
