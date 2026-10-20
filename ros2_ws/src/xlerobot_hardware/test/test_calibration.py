from dataclasses import replace
import math
from pathlib import Path

import pytest
import yaml

from xlerobot_hardware.config import Joint, RAD_PER_TICK, load_config
from xlerobot_hardware.transport import CommandBuffer, decode, encode


def test_calibrated_radians_and_mirrored_servo():
    joint = Joint("a", "left", 1, "position", 2048, -1, 1000, 3000, 0.5, -1, 1)
    assert joint.target(math.pi / 4) == 1536
    assert joint.position(1536) == pytest.approx(math.pi / 4)
    assert joint.speed(20) == round(-0.5 / RAD_PER_TICK)
    for value in (float("nan"), float("inf"), -2, 2):
        with pytest.raises(ValueError):
            joint.target(value)


@pytest.mark.parametrize("value", [-3073, -1, 0, 1, 3073])
def test_sign_magnitude(value):
    assert decode(encode(value)) == value
    if value < 0:
        assert encode(value) == 0x8000 | -value


def test_watchdog_requires_both_complete_streams():
    position = Joint("a", "left", 1, "position", 2048, 1, 0, 4095, 0.5, -1, 1)
    velocity = replace(position, name="wheel", id=2, mode="velocity")
    buffer = CommandBuffer([position, velocity], 0.5)
    buffer.receive("position", ["a"], [0.1], 0)
    assert not buffer.fresh(0)
    buffer.receive("velocity", ["wheel"], [0.0], 0)
    assert buffer.fresh(0.1)
    buffer.receive("position", ["a"], [0.2], 1)
    assert not buffer.fresh(1)
    for names, values in [([], []), (["a", "a"], [0, 0]), (["a"], [math.nan]), (["a"], [2])]:
        with pytest.raises(ValueError):
            buffer.receive("position", names, values, 2)
    assert buffer.seen["position"] == 1


def test_template_cannot_connect_without_calibration(tmp_path):
    template = Path(__file__).parents[1] / "config/hardware.example.yaml"
    data = yaml.safe_load(template.read_text())
    model = {name: {"command_interface": "velocity" if "wheel" in name else "position",
                    "velocity": 6 if "wheel" in name else 1, "lower": -3.2, "upper": 3.2}
             for name in data["joints"]}
    with pytest.raises(ValueError, match="serial-device path"):
        load_config(template, model)
    for i, bus in enumerate(data["buses"].values()):
        bus["port"] = f"/tmp/emulator-{i}"
    for joint in data["joints"].values():
        joint.update(zero_tick=2048, direction=1, min_tick=100, max_tick=4000)
    path = tmp_path / "hardware.yaml"
    path.write_text(yaml.safe_dump(data))
    assert len(load_config(path, model)[1]) == 16
    data["joints"]["Pitch_L"]["id"] = data["joints"]["Rotation_L"]["id"]
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="Duplicate"):
        load_config(path, model)
