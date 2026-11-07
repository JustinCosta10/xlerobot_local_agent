"""Launch failure regressions; construct actions without starting ROS or Isaac."""

import builtins
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
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
