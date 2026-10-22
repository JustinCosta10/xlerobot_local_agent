import pytest

from xlerobot_diagnostics.health import BridgeHealth


def healthy():
    health = BridgeHealth()
    health.clock(0, 0.0)
    health.clock(10_000_000, 0.1)
    health.joints(["head_pan_joint"], [0.0], 0.1)
    health.transforms([("world", "chassis")], 0.1)
    return health


def test_requires_all_streams_and_advancing_time():
    health = BridgeHealth()
    assert len(health.problems(0)) == 3
    health.clock(0, 0)
    health.joints(["joint"], [0.0], 0)
    health.transforms([("world", "chassis")], 0)
    assert health.problems(0) == ["simulation clock is not advancing"]
    health.clock(1, 0.1)
    assert not health.problems(0.1)


def test_paused_clock_detected_even_if_messages_keep_arriving():
    health = healthy()
    health.clock(10_000_000, 4)
    health.joints(["joint"], [0.0], 4)
    health.transforms([("world", "chassis")], 4)
    assert health.problems(4) == ["simulation clock is not advancing"]


def test_rewind_requires_new_advancing_sample():
    health = healthy()
    health.clock(0, 0.2)
    assert "simulation clock is not advancing" in health.problems(0.2)
    health.clock(1, 0.3)
    assert not health.problems(0.3)


def test_stale_streams_use_wall_time():
    health = healthy()
    assert not health.problems(0.2)
    assert len(health.problems(4.0)) == 4


@pytest.mark.parametrize("names,positions", [
    ([], []), (["a", "a"], [0.0, 0.0]), ([""], [0.0]),
    (["a"], []), (["a"], [float("nan")]), (["a"], [float("inf")]),
])
def test_rejects_invalid_joint_feedback(names, positions):
    health = healthy()
    health.joints(names, positions, 0.2)
    assert health.problems(0.2)
    health.joints(["a"], [0.0], 0.3)
    assert not health.problems(0.3)


def test_unrelated_tf_does_not_pass():
    health = BridgeHealth()
    health.transforms([("world", "other_robot"), ("chassis", "chassis")], 0)
    assert "tf" not in health.seen


def test_mock_needs_no_clock():
    health = BridgeHealth(require_clock=False, expected_joints=["a", "b"])
    health.joints(["a", "b"], [0.0, 0.0], 0.0)
    health.transforms([("base_link", "chassis")], 0.0)
    assert not health.problems(0.1)
    health.joints(["a", "b"], [0.0, 0.0], 4.0)
    health.transforms([("base_link", "chassis")], 4.0)
    assert not health.problems(4.0)


@pytest.mark.parametrize("seconds", [0, -1, float("nan"), float("inf")])
def test_stale_timeout_validation(seconds):
    with pytest.raises(ValueError):
        BridgeHealth(seconds)
