"""Interactive terminal entry point; no network calls occur until a user instruction."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

import yaml

from .agent import Agent
from .tools import ToolRegistry


def default_config():
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("xlerobot_agent")) / "config/agent.yaml"
    except (ImportError, LookupError):
        return Path(__file__).resolve().parents[1] / "config/agent.yaml"


def load_config(path):
    with open(path) as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("Configuration must be a YAML mapping.")
    for name in ("provider", "model", "endpoint", "namespace"):
        if not isinstance(config.get(name), str) or not config[name].strip():
            raise ValueError(f"{name} must be a nonempty string.")
    for name in ("request_timeout_s", "service_timeout_s"):
        value = config.get(name)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be a positive finite number.")
    for name in ("max_output_tokens", "max_tool_rounds"):
        if type(config.get(name)) is not int or config[name] < 1:
            raise ValueError(f"{name} must be a positive integer.")
    if type(config.get("use_sim_time")) is not bool:
        raise ValueError("use_sim_time must be true or false.")
    if type(config.get("enable_navigation", False)) is not bool:
        raise ValueError("enable_navigation must be true or false.")
    if config.get("reasoning_effort") not in (None, "", "none", "minimal", "low", "medium", "high", "xhigh", "max"):
        raise ValueError("Invalid reasoning_effort; use a value supported by your chosen model.")
    return config


class Events:
    def __init__(self, trace=None):
        self.stream = None
        if trace:
            path = Path(trace)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.stream = path.open("a", encoding="utf-8")

    def __call__(self, event):
        kind = event["event"]
        if self.stream:
            self.stream.write(json.dumps(dict(time=datetime.now(timezone.utc).isoformat(), **event),
                                         allow_nan=False) + "\n")
            self.stream.flush()
        if kind == "plan":
            print("Plan:\n" + "\n".join(f"  {i}. {s}" for i, s in enumerate(event["steps"], 1)), flush=True)
        elif kind == "tool_call":
            print(f"Call: {event['name']} {event['arguments']}", flush=True)
        elif kind == "tool_result":
            print("Result: " + json.dumps(event["result"], ensure_ascii=True), flush=True)
        elif kind in ("assistant", "stopped", "error"):
            print(event["text"], flush=True)
        elif kind == "session":
            print(f"Mode: {event['mode']} | model: {event['model']} | namespace: {event['namespace']}", flush=True)
        elif kind == "navigation_progress":
            print(f"Navigation: {event['distance_remaining_m']:.2f} m remaining; "
                  f"{event['elapsed_s']:.1f} s elapsed", flush=True)

    def close(self):
        if self.stream:
            self.stream.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description="XLeRobot perception and optional supervised simulation navigation.")
    parser.add_argument("--config", type=Path, default=default_config())
    parser.add_argument("--model", help="OpenAI model ID; overrides YAML")
    parser.add_argument("--namespace", help="ROS namespace; overrides YAML")
    parser.add_argument("--enable-navigation", action="store_true", help="Enable supervised simulation navigation tools")
    clock = parser.add_mutually_exclusive_group()
    clock.add_argument("--wall-time", action="store_true", help="Use wall time instead of simulation time")
    clock.add_argument("--sim-time", action="store_true")
    parser.add_argument("--demo", action="store_true", help="Scripted model + fake perception; no API or ROS")
    parser.add_argument("--fake-perception", action="store_true", help="Real OpenAI model with fake perception; uses API credits")
    parser.add_argument("--once", metavar="INSTRUCTION", help="Run one instruction and exit")
    parser.add_argument("--trace", type=Path, help="Append visible events to JSONL (no credentials or reasoning)")
    args = parser.parse_args(argv)
    robot = events = None
    try:
        config = load_config(args.config)
        if args.model:
            config["model"] = args.model
        if args.namespace:
            config["namespace"] = args.namespace
        if args.wall_time or args.sim_time:
            config["use_sim_time"] = args.sim_time
        enable_navigation = args.enable_navigation or config.get("enable_navigation", False)
        if enable_navigation and (not config["use_sim_time"] or args.demo or args.fake_perception):
            raise ValueError("Navigation requires live ROS simulation; incompatible with --wall-time, --demo, --fake-perception.")
        if args.demo:
            from .demo import ScriptedProvider
            provider = ScriptedProvider()
        else:
            if config["provider"] != "openai":
                raise ValueError("Only the openai provider is implemented; --demo uses a scripted test provider.")
            from .providers.openai import OpenAIProvider
            provider = OpenAIProvider(config["model"], endpoint=config["endpoint"],
                                      timeout_s=config["request_timeout_s"],
                                      max_output_tokens=config["max_output_tokens"],
                                      reasoning_effort=config.get("reasoning_effort"))
        if args.demo or args.fake_perception:
            from .demo import FakePerception
            robot = FakePerception()
        else:
            from .ros_adapter import RosAdapter
            robot = RosAdapter(config["namespace"], config["use_sim_time"], config["service_timeout_s"],
                               enable_navigation=enable_navigation)
        mode = "scripted_model_fake_perception" if args.demo else (
            "openai_fake_perception" if args.fake_perception else "openai_live_ros_perception")
        events = Events(args.trace)
        if enable_navigation:
            robot.navigation.emit = events
        events(dict(event="session", mode=mode, model="scripted" if args.demo else config["model"],
                    namespace=config["namespace"], use_sim_time=config["use_sim_time"],
                    max_tool_rounds=config["max_tool_rounds"], max_output_tokens=config["max_output_tokens"]))
        agent = Agent(provider, ToolRegistry(robot), max_tool_rounds=config["max_tool_rounds"], emit=events)
        if args.demo:
            print("Offline fixed script: always demonstrates cubes/mugs; it does not interpret your words.")
        if args.once:
            return 0 if agent.run(args.once) is not None else 1
        print("Enter a robot request. /new clears conversation; /quit exits. Ctrl+C interrupts.\n"
              "Last accepted YOLOE vocabulary stays active. " +
              ("Supervised simulation navigation enabled; Ctrl+C cancels movement."
               if enable_navigation else "No motion tools are available."))
        try:
            import readline  # noqa: F401 — terminal editing/history, never persisted
        except ImportError:
            pass
        while True:
            try:
                instruction = input("You: ").strip()
            except EOFError:
                break
            except KeyboardInterrupt:
                print("\nUse /quit to exit.")
                continue
            if instruction == "/quit":
                break
            if instruction == "/new":
                agent.reset()
                events(dict(event="stopped", text="Conversation cleared; shared perception is unchanged."))
            elif instruction:
                try:
                    agent.run(instruction)
                except Exception as error:
                    # Keep the terminal usable, but don't replay possibly applied operations.
                    agent.reset()
                    events(dict(event="error", text=f"{error}\nConversation cleared; task not resumed."))
        return 0
    except (ValueError, RuntimeError, OSError, ImportError) as error:
        print(f"Agent setup/request failed: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        if robot:
            robot.close()
        if events:
            events.close()


if __name__ == "__main__":
    raise SystemExit(main())
