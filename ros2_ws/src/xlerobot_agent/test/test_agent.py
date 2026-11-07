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


@pytest.mark.parametrize("name,args", [
    ("set_perception_vocabulary", '{"classes": []}'),
    ("set_perception_vocabulary", '{"classes": [" Cube", "cube"]}'),
    ("set_perception_vocabulary", '{"classes": "cube"}'),
    ("observe_scene", '{"duration_s": NaN}'),
    ("observe_scene", '{"duration_s": 1e999}'),
    ("observe_scene", '{"duration_s": true}'),
    ("query_objects", '{"label": "cube", "object_id": null, "include_stale": null, "limit": 201}'),
    ("query_objects", '{}'),
    ("query_objects", '{broken'),
])
def test_invalid_arguments_never_reach_robot(name, args):
    class Forbidden:
        def __getattr__(self, _name):
            pytest.fail("Invalid arguments dispatched")
    registry = ToolRegistry(Forbidden())
    registry.plan = ["Observe"]
    assert registry.dispatch(name, args)["code"] == "INVALID_ARGUMENT"


def test_allowlist_and_plan_gate_and_existing_normalization():
    robot = FakePerception()
    registry = ToolRegistry(robot)
    assert registry.dispatch("navigate_to_object", "{}")["code"] == "UNKNOWN_TOOL"
    assert registry.dispatch("observe_scene", '{"duration_s": null}')["code"] == "PLAN_REQUIRED"
    registry.dispatch("update_plan", '{"steps": ["Set categories"]}')
    result = registry.dispatch("set_perception_vocabulary", '{"classes": [" CUBE ", "new category"]}')
    assert result["classes"] == ["cube", "new category"]


def test_interrupt_fills_remaining_call_results_without_dispatch_and_followup_can_run():
    class Robot(FakePerception):
        def observe(self, duration_s):
            raise KeyboardInterrupt()
        def set_vocabulary(self, classes):
            pytest.fail("Call after interruption dispatched")

    class Model:
        def next_turn(self, history, tools, instructions):
            if not any(t.role == "tool" for t in history):
                return call("update_plan", {"steps": ["Observe"]})
            return Turn("assistant", calls=[ToolCall("observe", "observe_scene", '{"duration_s": 1}'),
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
