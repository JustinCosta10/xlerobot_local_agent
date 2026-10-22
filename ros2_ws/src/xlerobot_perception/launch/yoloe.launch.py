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
    if not python.is_file() or not model.is_file():
        raise RuntimeError("YOLOE runtime/weights missing. Run bash scripts/install_yoloe.sh, "
                           "then source scripts/env.sh; or provide python:= and model:= paths.")
    parameters = [value("config"), {"use_sim_time": value("use_sim_time") == "true"}]
    nodes = [Node(
        package="xlerobot_perception", executable="yoloe_node", name="yoloe",
        namespace=value("namespace"), output="screen", prefix=shlex.quote(str(python)),
        parameters=[*parameters, {"model": str(model), "image_topic": value("image_topic")}],
        additional_env={"YOLO_AUTOINSTALL": "false"})]
    if value("visualize") == "true":
        nodes.append(Node(package="xlerobot_perception", executable="yoloe_visualizer",
                          name="yoloe_visualizer", namespace=value("namespace"),
                          parameters=parameters, output="screen"))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory("xlerobot_perception"))
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="xlerobot"),
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("visualize", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("config", default_value=str(share / "config/yoloe.yaml")),
        DeclareLaunchArgument("python", default_value=os.environ.get("XLEROBOT_YOLOE_PYTHON", sys.executable)),
        DeclareLaunchArgument("model", default_value=os.environ.get(
            "XLEROBOT_YOLOE_MODEL", "yoloe-11s-seg-pf.pt")),
        OpaqueFunction(function=setup),
    ])
