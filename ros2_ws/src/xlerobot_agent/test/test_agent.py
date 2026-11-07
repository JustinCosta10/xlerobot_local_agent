import json

import pytest

from xlerobot_agent.agent import Agent
from xlerobot_agent.demo import FakePerception, ScriptedProvider
from xlerobot_agent.providers.base import ToolCall, Turn
from xlerobot_agent.tools import ToolRegistry


def call(name, args, call_id="call-1"):
    return Turn("assistant", calls=[ToolCall(call_id, name, json.dumps(args))])


def test_model_chooses_followup_using_actual_result_and_matching_call_id():
    class Model:
        def next_turn(self, history, schemas, instructions):
            results = [t for t in history if t.role == "tool"]
            if not results:
                return call("update_plan", {"steps": ["Query confirmed objects and use their IDs."]}, "plan")
            if len(results) == 1:
                return call("query_objects", dict(label="cube", object_id=None, include_stale=None, limit=None), "q1")
            if len(results) == 2:
                assert results[-1].call_id == "q1"
                actual_id = json.loads(results[-1].text)["objects"][0]["object_id"]
                return call("query_objects", dict(label=None, object_id=actual_id, include_stale=None, limit=1), "q2")
            assert results[-1].call_id == "q2"
            assert json.loads(results[-1].text)["objects"][0]["object_id"] == "fake-cube-1"
            return Turn("assistant", text="Found confirmed object fake-cube-1.")

    events = []
    agent = Agent(Model(), ToolRegistry(FakePerception()), emit=events.append)
    assert "fake-cube-1" in agent.run("Which cube is known?")
    names = [e["name"] for e in events if e["event"] == "tool_call"]
    assert names == ["update_plan", "query_objects", "query_objects"]


def test_stored_objects_and_locations_reach_model_without_a_new_sighting():
    robot = FakePerception()
    robot.observed_at -= 60 * 10**9
    robot.set_vocabulary(["blue cube"])

    class Model:
        def next_turn(self, history, schemas, instructions):
            results = [t for t in history if t.role == "tool"]
            if not results:
                return call("update_plan", {"steps": ["Read stored objects and locations."]}, "plan")
            if len(results) == 1:
                return call("observe_scene", {}, "map")
            assert results[-1].call_id == "map"
            result = json.loads(results[-1].text)
            assert result["success"] and result["count"] == 2
            cube = next(item for item in result["objects"] if item["label"] == "cube")
            assert cube["object_id"] == "fake-cube-1"
            assert cube["position_m"] == [1., .2, .8]
            assert cube["stale"] and cube["observation_age_s"] >= 60
            return Turn("assistant", text="Stored cube at [1.0, 0.2, 0.8] in map.")

    agent = Agent(Model(), ToolRegistry(robot))
    assert "Stored cube" in agent.run("Where is the cube?")


@pytest.mark.parametrize("include_stale,count", [(None, 1), (True, 1), (False, 0)])
def test_query_defaults_to_stored_records_regardless_of_age(include_stale, count):
    robot = FakePerception()
    robot.observed_at -= 60 * 10**9
    registry = ToolRegistry(robot)
    registry.plan = ["Read stored cube"]
    result = registry.dispatch("query_objects", json.dumps(dict(
        label="cube", object_id=None, include_stale=include_stale, limit=None)))
    assert result["success"] and result["count"] == count


@pytest.mark.parametrize("name,args", [
    ("set_perception_vocabulary", '{"classes": []}'),
    ("set_perception_vocabulary", '{"classes": [" Cube", "cube"]}'),
    ("set_perception_vocabulary", '{"classes": "cube"}'),
    ("observe_scene", '{"duration_s": NaN}'),
    ("observe_scene", '{"duration_s": 1e999}'),
    ("observe_scene", '{"duration_s": true}'),
    ("observe_scene", '{"duration_s": null}'),
    ("query_objects", '{"label": "cube", "object_id": null, "include_stale": null, "limit": 201}'),
    ("query_objects", '{}'),
    ("query_objects", '{broken'),
])
def test_invalid_arguments_never_reach_robot(name, args):
    class Forbidden:
        navigation_enabled = False
        def __getattr__(self, _name):
            pytest.fail("Invalid arguments dispatched")
    registry = ToolRegistry(Forbidden())
    registry.plan = ["Observe"]
    assert registry.dispatch(name, args)["code"] == "INVALID_ARGUMENT"


def test_allowlist_and_plan_gate_and_existing_normalization():
    robot = FakePerception()
    registry = ToolRegistry(robot)
    assert registry.dispatch("navigate_to_object", "{}")["code"] == "UNKNOWN_TOOL"
    assert registry.dispatch("observe_scene", '{}')["code"] == "PLAN_REQUIRED"
    registry.dispatch("update_plan", '{"steps": ["Set categories"]}')
    result = registry.dispatch("set_perception_vocabulary", '{"classes": [" CUBE ", "new category"]}')
    assert result["classes"] == ["cube", "new category"]


def test_interrupt_fills_remaining_call_results_without_dispatch_and_followup_can_run():
    class Robot(FakePerception):
        def observe(self):
            raise KeyboardInterrupt()
        def set_vocabulary(self, classes):
            pytest.fail("Call after interruption dispatched")

    class Model:
        def next_turn(self, history, tools, instructions):
            if not any(t.role == "tool" for t in history):
                return call("update_plan", {"steps": ["Observe"]})
            return Turn("assistant", calls=[ToolCall("observe", "observe_scene", '{}'),
                                            ToolCall("late", "set_perception_vocabulary", '{"classes": ["cube"]}')])

    agent = Agent(Model(), ToolRegistry(Robot()))
    assert agent.run("Observe") is None
    results = {t.call_id: json.loads(t.text) for t in agent.history if t.role == "tool"}
    assert results["observe"]["code"] == "INTERRUPTED"
    assert results["late"]["code"] == "INTERRUPTED"
    agent.reset()
    assert not agent.history and not agent.registry.plan


def test_round_limit_stops_calls_and_reset_does_not_reset_vocabulary():
    agent = Agent(ScriptedProvider(), ToolRegistry(FakePerception()), max_tool_rounds=1)
    assert agent.run("demo") is None
    assert json.loads(agent.history[-1].text)["code"] == "BUDGET_EXHAUSTED"
    agent.registry.robot.set_vocabulary(["bowl"])
    agent.reset()
    assert agent.registry.robot.status()["vocabulary"]["active_classes"] == ["bowl"]


def test_interrupt_during_model_request_stops_without_dispatch():
    class Model:
        def next_turn(self, *args):
            raise KeyboardInterrupt()
    events = []
    agent = Agent(Model(), ToolRegistry(FakePerception()), emit=events.append)
    assert agent.run("look") is None
    assert not any(e["event"] == "tool_call" for e in events)
    assert events[-1]["event"] == "stopped"


def test_offline_demo_runs_and_labels_evidence():
    agent = Agent(ScriptedProvider(), ToolRegistry(FakePerception()))
    assert "OFFLINE SCRIPTED DEMO" in agent.run("Demo")
    assert "OFFLINE SCRIPTED DEMO" in agent.run("Follow-up")


@pytest.mark.parametrize("field,value", [("x", 8), ("y", -8), ("yaw", 4), ("x", True),
                                        ("session_id", ""), ("timeout_s", 0), ("timeout_s", 301)])
def test_invalid_navigation_arguments_never_dispatch(field, value):
    class Robot(FakePerception):
        navigation_enabled = True
        def navigate(self, **kwargs):
            pytest.fail("Invalid navigation dispatched")
    registry = ToolRegistry(Robot())
    registry.plan = ["Navigate"]
    args = dict(x=0, y=0, yaw=0, session_id="map-one", timeout_s=None)
    args[field] = value
    assert registry.dispatch("navigate", json.dumps(args))["code"] == "INVALID_ARGUMENT"


def test_navigation_opt_in_plan_gate_and_emergency_tools():
    class Robot(FakePerception):
        navigation_enabled = True
        def get_navigation_state(self):
            return dict(success=True, session_id="map-one")
        def cancel_navigation(self):
            return dict(success=True, code="CANCELED")
        def navigate(self, **kwargs):
            return dict(success=True, arguments=kwargs)
    assert ToolRegistry(FakePerception()).dispatch("navigate", "{}")["code"] == "UNKNOWN_TOOL"
    registry = ToolRegistry(Robot())
    assert registry.dispatch("get_navigation_state", "{}")["session_id"] == "map-one"
    assert registry.dispatch("cancel_navigation", "{}")["code"] == "CANCELED"
    args = json.dumps(dict(x=0, y=0, yaw=0, session_id="map-one", timeout_s=None))
    assert registry.dispatch("navigate", args)["code"] == "PLAN_REQUIRED"
    registry.plan = ["Navigate"]
    assert registry.dispatch("navigate", args)["arguments"]["timeout_s"] == 120.


@pytest.mark.parametrize("name,args,expected", [
    ("navigate_relative", dict(forward_m=.5, left_m=0, yaw_delta_rad=None,
                               session_id="map-one", timeout_s=None), {"yaw_delta_rad": 0.}),
    ("navigate_to_object", dict(object_id="known-uuid", standoff_m=None,
                                session_id="map-one", timeout_s=None), {"standoff_m": .8}),
])
def test_new_navigation_tools_are_exposed_only_with_motion_enabled_and_use_defaults(name, args, expected):
    class Robot(FakePerception):
        navigation_enabled = True
        def navigate_relative(self, **kwargs):
            return dict(success=True, arguments=kwargs)
        def navigate_to_object(self, **kwargs):
            return dict(success=True, arguments=kwargs)
    assert ToolRegistry(FakePerception()).dispatch(name, json.dumps(args))["code"] == "UNKNOWN_TOOL"
    registry = ToolRegistry(Robot())
    assert {"navigate", "navigate_relative", "navigate_to_object"} <= {s["name"] for s in registry.schemas}
    assert registry.dispatch(name, json.dumps(args))["code"] == "PLAN_REQUIRED"
    registry.plan = ["Navigate using a grounded destination"]
    result = registry.dispatch(name, json.dumps(args))
    assert result["success"] and result["arguments"]["timeout_s"] == 120.
    for field, value in expected.items():
        assert result["arguments"][field] == value


@pytest.mark.parametrize("name,args", [
    ("navigate_relative", dict(forward_m=True, left_m=0, yaw_delta_rad=0, session_id="map", timeout_s=None)),
    ("navigate_relative", dict(forward_m=4, left_m=0, yaw_delta_rad=0, session_id="map", timeout_s=None)),
    ("navigate_relative", dict(forward_m=0, left_m=0, yaw_delta_rad=4, session_id="map", timeout_s=None)),
    ("navigate_to_object", dict(object_id="", standoff_m=None, session_id="map", timeout_s=None)),
    ("navigate_to_object", dict(object_id="cube", standoff_m=.1, session_id="map", timeout_s=None)),
])
def test_new_navigation_invalid_arguments_never_reach_robot(name, args):
    class Robot:
        navigation_enabled = True
        def __getattr__(self, _):
            pytest.fail("Invalid navigation arguments dispatched")
    registry = ToolRegistry(Robot())
    registry.plan = ["Navigate"]
    assert registry.dispatch(name, json.dumps(args))["code"] == "INVALID_ARGUMENT"
