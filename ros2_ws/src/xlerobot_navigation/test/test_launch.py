"""Launch failure regressions; construct actions without starting ROS or Isaac."""

import builtins
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.events import Shutdown
from launch.events.process import ProcessExited
import pytest


def launch_module(package, filename):
    path = Path(get_package_share_directory(package)) / "launch" / filename
    spec = importlib.util.spec_from_file_location(package + "_launch_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_missing_interface_fails_preflight_with_resourcing_instruction(monkeypatch):
    module = launch_module("xlerobot_navigation", "sim_navigation.launch.py")
    normal_import = builtins.__import__

    def unavailable(name, *args, **kwargs):
        if name == "xlerobot_navigation_interfaces.action":
            raise ModuleNotFoundError("No module named 'xlerobot_navigation_interfaces'")
        return normal_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", unavailable)
    with pytest.raises(RuntimeError, match="source scripts/env.sh"):
        module.check_agent_interfaces()


def test_controller_yaml_survives_shutdown_until_isaac_exits(monkeypatch, tmp_path):
    module = launch_module("xlerobot_bringup", "robot.launch.py")
    monkeypatch.setattr(module.tempfile, "tempdir", str(tmp_path))
    context = LaunchContext()
    context.launch_configurations.update(exclusive_navigation="true", foxglove="false", rviz="false")
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    actions = module.setup(context)
    controller_files = list(tmp_path.glob("xlerobot-agent-*.yaml"))
    assert len(controller_files) == 1
    config = controller_files[0]
    assert "~/cmd_vel:=cmd_vel_guarded" in config.read_text()
    handlers = [a.event_handler for a in actions if isinstance(a, RegisterEventHandler)]
    context._set_is_shutdown(True)
    shutdown = Shutdown(reason="Supervisor failed during Isaac startup")
    for handler in handlers:
        if handler.matches(shutdown):
            handler.handle(shutdown, context)
    assert config.is_file(), "Shutdown deleted config before the simulator finished reading it"
    simulator = next(a for a in actions if type(a) is ExecuteProcess)
    exited = ProcessExited(action=simulator, returncode=1, pid=1, name="isaac",
                           cmd=["unused"], cwd=None, env=None)
    for handler in handlers:
        if handler.matches(exited):
            assert not handler.handle(exited, context), "Emitted another shutdown during teardown"
    assert not config.exists()


def test_critical_exits_request_only_one_shutdown_across_launches():
    bringup = launch_module("xlerobot_bringup", "robot.launch.py")
    navigation = launch_module("xlerobot_navigation", "sim_navigation.launch.py")
    context = LaunchContext()
    assert len(navigation.stop_launch(context, "Supervisor failed")) == 1
    assert not bringup.stop_launch(context, "Isaac exited")
    assert not navigation.stop_launch(context, "Controller exited")


@pytest.mark.parametrize("overrides, expected_address, expected_port", [
    ({}, "127.0.0.1", "8765"),
    ({"foxglove_address": "192.0.2.10", "foxglove_port": "8766"}, "192.0.2.10", "8766"),
])
def test_navigation_preserves_local_bridge_default_and_explicit_remote_override(
        overrides, expected_address, expected_port):
    navigation = launch_module("xlerobot_navigation", "sim_navigation.launch.py")
    bringup = launch_module("xlerobot_bringup", "robot.launch.py")
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in navigation.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    robot = next(action for action in navigation.setup(context)
                 if isinstance(action, IncludeLaunchDescription))

    robot_context = LaunchContext()
    robot_context.launch_configurations.update(dict(robot.launch_arguments))
    for action in bringup.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(robot_context)
    bridge_group = next(action for action in bringup.setup(robot_context)
                        if isinstance(action, GroupAction))
    bridge = next(action for action in bridge_group.get_sub_entities()
                  if isinstance(action, IncludeLaunchDescription))
    bridge_arguments = dict(bridge.launch_arguments)
    assert bridge_arguments["address"] == expected_address
    assert bridge_arguments["port"] == expected_port

    direct_context = LaunchContext()
    for action in bringup.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(direct_context)
    assert direct_context.launch_configurations["foxglove_address"] == "127.0.0.1"
