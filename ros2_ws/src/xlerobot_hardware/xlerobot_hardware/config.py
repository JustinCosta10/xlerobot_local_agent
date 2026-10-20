"""Explicit calibration in URDF radians; no local checkout or auto-discovery."""
from dataclasses import dataclass
import math
from pathlib import Path
import yaml

RAD_PER_TICK = math.tau / 4096


@dataclass(frozen=True)
class Joint:
    name: str
    bus: str
    id: int
    mode: str
    zero_tick: int
    direction: int
    min_tick: int
    max_tick: int
    max_velocity: float
    lower: float
    upper: float

    def position(self, tick):
        return self.direction * (tick - self.zero_tick) * RAD_PER_TICK

    def target(self, radians):
        if not math.isfinite(radians) or not self.lower <= radians <= self.upper:
            raise ValueError(f"{self.name}: target {radians} outside calibrated/URDF limits [{self.lower}, {self.upper}]")
        return max(self.min_tick, min(self.max_tick,
                                    round(self.zero_tick + self.direction * radians / RAD_PER_TICK)))

    def speed(self, radians_per_second):
        if not math.isfinite(radians_per_second):
            raise ValueError(f"{self.name}: non-finite velocity")
        limited = max(-self.max_velocity, min(self.max_velocity, radians_per_second))
        return round(self.direction * limited / RAD_PER_TICK)


def finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number, not {value!r}")
    return value


def integer(value, label, lower, upper):
    if type(value) is not int or not lower <= value <= upper:
        raise ValueError(f"{label} must be an integer in [{lower}, {upper}], not {value!r}")
    return value


def load_config(path, model):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or data.get("driver") != "sts3215":
        raise ValueError("hardware config must declare driver: sts3215")
    buses = data.get("buses", {})
    if not isinstance(buses, dict) or not buses:
        raise ValueError("At least one explicitly configured bus is required")
    ports = set()
    for name, bus in buses.items():
        port = bus.get("port")
        if not isinstance(port, str) or not port.startswith("/"):
            raise ValueError(f"buses.{name}.port must be an explicit absolute serial-device path")
        resolved = str(Path(port).resolve())
        if resolved in ports:
            raise ValueError("Bus ports must refer to different devices")
        ports.add(resolved)
        if bus.get("baudrate") not in (19200, 38400, 57600, 115200, 128000, 250000, 500000, 1000000):
            raise ValueError(f"buses.{name}.baudrate is unsupported")
    if set(data.get("joints", {})) != set(model):
        raise ValueError("Hardware configuration must map exactly the 16 model joints")
    joints, addresses = [], set()
    for name, limits in model.items():
        item = data["joints"][name]
        bus = item.get("bus")
        if bus not in buses:
            raise ValueError(f"{name}: unknown bus {bus!r}")
        servo = integer(item.get("id"), f"{name}.id", 1, 253)
        if (bus, servo) in addresses:
            raise ValueError(f"Duplicate motor address: {bus}/{servo}")
        addresses.add((bus, servo))
        direction = item.get("direction")
        if type(direction) is not int or direction not in (-1, 1):
            raise ValueError(f"{name}.direction must be explicitly calibrated to -1 or +1")
        zero = integer(item.get("zero_tick"), f"{name}.zero_tick", 0, 4095)
        low = integer(item.get("min_tick"), f"{name}.min_tick", 0, 4095)
        high = integer(item.get("max_tick"), f"{name}.max_tick", 0, 4095)
        if low >= high:
            raise ValueError(f"{name}: min_tick must be below max_tick")
        speed = finite(item.get("max_velocity"), f"{name}.max_velocity")
        if not 0 < speed <= min(limits["velocity"], 3073 * RAD_PER_TICK):
            raise ValueError(f"{name}: max_velocity exceeds URDF or servo limit")
        mode = limits["command_interface"]
        ends = sorted(direction * (tick - zero) * RAD_PER_TICK for tick in (low, high))
        lower = max(limits.get("lower", -math.inf), ends[0])
        upper = min(limits.get("upper", math.inf), ends[1])
        if lower >= upper:
            raise ValueError(f"{name}: calibration and URDF have no overlapping range")
        joints.append(Joint(name, bus, servo, mode, zero, direction, low, high, speed, lower, upper))
    rate = finite(data.get("update_rate", 30.0), "update_rate")
    timeout = finite(data.get("command_timeout", 0.5), "command_timeout")
    if not 1 <= rate <= 100 or not 2 / rate <= timeout <= 2:
        raise ValueError("update_rate must be 1..100 Hz; command_timeout must be 2 periods..2 seconds")
    return buses, joints, float(rate), float(timeout)
