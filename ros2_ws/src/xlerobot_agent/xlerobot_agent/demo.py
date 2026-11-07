"""Explicitly scripted offline fixtures, never presented as a real model or camera."""

import json
import time

from xlerobot_perception.vocabulary import Vocabulary

from .providers.base import ToolCall, Turn


class FakePerception:
    def __init__(self):
        self.vocabulary = Vocabulary()
        self.vocabulary.activate(["cube", "mug"])
        self.observed_at = time.time_ns()

    def status(self):
        return dict(execution_mode="fake_perception", vocabulary={
            "model": "fake", "revision": self.vocabulary.revision,
            "active_classes": self.vocabulary.active}, session_id="fake-session",
                    objects="confirmed_only", motion_tools=False)

    def set_vocabulary(self, classes):
        self.vocabulary.activate(classes)
        return dict(success=True, code="VOCABULARY_ACTIVE", message="FAKE vocabulary active",
                    mechanism="runtime_text_prompts", classes=self.vocabulary.active,
                    class_ids=self.vocabulary.class_ids, revision=self.vocabulary.revision)

    def query(self, label="", object_id="", include_stale=True, limit=100):
        now = time.time_ns()
        stale = now - self.observed_at > 10 * 10**9
        objects = [dict(object_id=f"fake-{name}-1", session_id="fake-session", label=name,
                        tracking_state="confirmed", frame_id="map", position_m=pos,
                        observation_stamp_ns=self.observed_at,
                        observation_age_s=(now - self.observed_at) / 1e9,
                        stale=stale, observation_count=3)
                   for name, pos in [("cube", [1., .2, .8]), ("mug", [1., .5, .8])]
                   if (not label or name == label) and (not object_id or object_id == f"fake-{name}-1")
                   and (include_stale or not stale)]
        return dict(success=True, session_id="fake-session", objects=objects[:limit],
                    count=len(objects[:limit]), limit=limit,
                    completeness="unknown" if len(objects) >= limit else "complete",
                    scope="FAKE confirmed records")

    def observe(self):
        result = self.query(include_stale=True, limit=200)
        result.update(code="OBJECT_MAP", evidence="fake_stored_confirmed_objects",
                      message="FAKE stored confirmed objects; no camera or waiting used.")
        return result

    def close(self):
        pass


class ScriptedProvider:
    """Fixed demonstration, irrespective of wording. Real interpretation uses OpenAI."""

    def next_turn(self, conversation, tools, instructions):
        # Start a new fixed script at each actual user instruction (before metadata).
        start = max(i for i, turn in enumerate(conversation)
                    if turn.role == "user" and not turn.text.startswith("Current perception metadata"))
        results = [turn for turn in conversation[start:] if turn.role == "tool"]
        steps = [
            ("update_plan", {"steps": ["Set cube and mug prompts.", "Observe confirmed objects.",
                                      "Query cube details and report the returned evidence."]}),
            ("set_perception_vocabulary", {"classes": ["cube", "mug"]}),
            ("observe_scene", {}),
            ("query_objects", {"label": "cube", "object_id": None, "include_stale": None, "limit": None}),
        ]
        if results and not json.loads(results[-1].text).get("success"):
            return Turn("assistant", text="Scripted demo stopped after a tool error: " + results[-1].text)
        if len(results) < len(steps):
            name, args = steps[len(results)]
            return Turn("assistant", calls=[ToolCall(f"demo-{len(conversation)}", name, json.dumps(args))])
        result = json.loads(results[-1].text)
        return Turn("assistant", text=f"OFFLINE SCRIPTED DEMO: query returned {result['count']} "
                    "fake confirmed cube(s). This is not a real model or live perception result.")
