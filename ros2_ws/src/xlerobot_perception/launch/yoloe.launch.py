"""Camera-independent YOLOE pipeline. This launch does not start a camera."""

import os
from pathlib import Path
import shlex
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    python = Path(value("python"))
    model = Path(value("model"))
    encoder = Path(value("text_encoder"))
    if not python.is_file() or not model.is_file() or not encoder.is_file():
        raise RuntimeError("YOLOE runtime/weights missing. Run bash scripts/install_yoloe.sh, "
                           "then source scripts/env.sh; or provide python:=, model:= and text_encoder:= paths.")
    parameters = [value("config"), {"use_sim_time": value("use_sim_time") == "true"}]
    nodes = [Node(
        package="xlerobot_perception", executable="yoloe_node", name="yoloe",
        namespace=value("namespace"), output="screen", prefix=shlex.quote(str(python)),
        parameters=[*parameters, {"model": str(model), "text_encoder": str(encoder),
                                 "image_topic": value("image_topic")}],
        additional_env={"YOLO_AUTOINSTALL": "false"})]
    if value("visualize") == "true":
        nodes.append(Node(package="xlerobot_perception", executable="yoloe_visualizer",
                          name="yoloe_visualizer", namespace=value("namespace"),
                          parameters=parameters, output="screen"))
    if value("object_map") == "true":
        nodes.append(Node(package="xlerobot_perception", executable="object_mapper",
                          name="object_mapper", namespace=value("namespace"), output="screen",
                          parameters=[value("object_map_config"), {
                              "use_sim_time": value("use_sim_time") == "true",
                              "database_path": value("object_database"),
                              "depth_topic": value("depth_topic"),
                              "camera_info_topic": value("camera_info_topic")}]))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory("xlerobot_perception"))
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="xlerobot"),
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("visualize", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("config", default_value=str(share / "config/yoloe.yaml")),
        DeclareLaunchArgument("object_map", default_value="false", choices=["true", "false"],
                              description="Store objects using aligned depth and camera-to-map TF"),
        DeclareLaunchArgument("object_map_config", default_value=str(share / "config/object_mapping.yaml")),
        DeclareLaunchArgument("object_database", default_value=os.environ.get(
            "XLEROBOT_OBJECT_DATABASE", str(Path.home() / ".ros/xlerobot/objects.sqlite3"))),
        DeclareLaunchArgument("depth_topic", default_value="/camera/aligned_depth_to_color/image_raw_float"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info"),
        DeclareLaunchArgument("python", default_value=os.environ.get("XLEROBOT_YOLOE_PYTHON", sys.executable)),
        DeclareLaunchArgument("model", default_value=os.environ.get(
            "XLEROBOT_YOLOE_MODEL", "yoloe-11s-seg.pt")),
        DeclareLaunchArgument("text_encoder", default_value=os.environ.get(
            "XLEROBOT_YOLOE_TEXT_ENCODER", "mobileclip_blt.ts")),
        OpaqueFunction(function=setup),
    ])
