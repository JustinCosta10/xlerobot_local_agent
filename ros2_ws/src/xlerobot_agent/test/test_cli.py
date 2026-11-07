import json

from xlerobot_agent.cli import main


def test_demo_never_constructs_openai_or_ros(monkeypatch, tmp_path, capsys):
    from xlerobot_agent.providers import openai
    def forbidden(*args, **kwargs):
        raise AssertionError("Offline demo tried to construct OpenAI")
    monkeypatch.setattr(openai, "OpenAIProvider", forbidden)
    trace = tmp_path / "trace.jsonl"
    assert main(["--demo", "--once", "look", "--trace", str(trace)]) == 0
    events = [json.loads(line) for line in trace.read_text().splitlines()]
    assert events[0]["mode"] == "scripted_model_fake_perception"
    assert any(event["event"] == "plan" for event in events)
    assert "OFFLINE SCRIPTED DEMO" in capsys.readouterr().out
    assert all("provider_items" not in event for event in events)


def test_terminal_followup_new_and_quit(monkeypatch, capsys):
    inputs = iter(["look", "follow up", "/new", "/quit"])
    monkeypatch.setattr("builtins.input", lambda _: next(inputs))
    assert main(["--demo"]) == 0
    output = capsys.readouterr().out
    assert output.count("OFFLINE SCRIPTED DEMO") == 2
    assert "Conversation cleared" in output
