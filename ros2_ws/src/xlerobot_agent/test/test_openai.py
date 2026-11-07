import json

import pytest

from xlerobot_agent.providers.base import Turn
from xlerobot_agent.providers.openai import OpenAIProvider, ProviderError
from xlerobot_agent.tools import SCHEMAS


def test_responses_continuation_and_strict_tools_without_network():
    sent = []
    opaque = {"type": "reasoning", "id": "r1", "summary": [], "encrypted_content": "opaque-fixture"}
    function = {"type": "function_call", "call_id": "c1", "name": "observe_scene",
                "arguments": '{"duration_s": 2}'}
    def transport(payload):
        sent.append(payload)
        if len(sent) == 1:
            return {"status": "completed", "output": [opaque, function]}
        return {"status": "completed", "output": [{"type": "message", "role": "assistant",
                "content": [{"type": "output_text", "text": "No freshly confirmed objects."}]}]}
    provider = OpenAIProvider("test-model", transport=transport)
    history = [Turn("user", text="Observe")]
    turn = provider.next_turn(history, SCHEMAS, "policy")
    assert turn.calls[0].call_id == "c1"
    history += [turn, Turn("tool", call_id="c1", text=json.dumps({"objects": []}))]
    assert provider.next_turn(history, SCHEMAS, "policy").text == "No freshly confirmed objects."
    assert sent[1]["input"][1:3] == [opaque, function]
    assert sent[1]["input"][3]["call_id"] == "c1"
    assert sent[0]["parallel_tool_calls"] is False
    assert sent[0]["store"] is False
    assert all(t["strict"] for t in sent[0]["tools"])


@pytest.mark.parametrize("response", [
    {"status": "incomplete", "output": [{"type": "function_call"}]},
    {"status": "completed", "output": []},
    {"status": "completed", "output": [{"type": "function_call", "call_id": "c"}]},
])
def test_partial_or_malformed_response_never_dispatches(response):
    provider = OpenAIProvider("test", transport=lambda _: response)
    with pytest.raises(ProviderError):
        provider.next_turn([Turn("user", text="test")], SCHEMAS, "policy")


def test_missing_key_is_local_error(monkeypatch):
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    with pytest.raises(ProviderError, match="OPENAI_API_KEY"):
        OpenAIProvider("test")
