"""STS3215 register transport. Never modifies IDs, homing, modes, or EEPROM.

Register/encoding reference: LeRobot v0.4.0 motors/feetech/tables.py and the
Feetech STS manual. Positions and speeds use 4096 ticks/revolution and bit-15
sign-magnitude, not signed two's-complement.
"""
import math
import time

from .config import RAD_PER_TICK


def decode(value):
    return -(value & 0x7fff) if value & 0x8000 else value


def encode(value):
    if not -32767 <= value <= 32767:
        raise ValueError("Value cannot be represented in STS sign-magnitude format")
    return abs(value) | (0x8000 if value < 0 else 0)


class ServoTransport:
    def __init__(self, buses, joints, sdk=None):
        if sdk is None:
            import scservo_sdk as sdk
        self.sdk = sdk
        self.packet = sdk.PacketHandler(0)
        self.joints = joints
        self.ports = {}
        self.enabled = False
        self.raw = {}
        self.wheel_ticks = {}
        self.previous_wheel_ticks = {}
        self.last_read = None
        try:
            for name, settings in buses.items():
                port = sdk.PortHandler(settings["port"])
                if not port.setBaudRate(settings["baudrate"]):
                    raise RuntimeError(f"Cannot open bus {name}")
                self.ports[name] = port
                # Bound serial writes, and keep SDK receive deadlines monotonic.
                port.ser.write_timeout = 0.05
                port.getCurrentTime = lambda: time.monotonic() * 1000
            # Read-only preflight for ALL motors before issuing any commands.
            for joint in joints:
                model = self.read_register(joint, 3, 2)
                mode = self.read_register(joint, 33, 1)
                if model != 777 or mode != (1 if joint.mode == "velocity" else 0):
                    raise RuntimeError(f"{joint.name}: expected STS3215 (777) in {joint.mode} mode; got model={model}, mode={mode}")
            self.read()
            for joint in joints:
                if joint.mode == "position":
                    joint.target(joint.position(self.raw[joint.name]))
        except Exception:
            self.close()
            raise

    def check(self, result, error=0):
        if result != self.sdk.COMM_SUCCESS or error:
            raise RuntimeError(f"Servo communication failed: {self.packet.getTxRxResult(result)}; motor error={error}")

    def read_register(self, joint, address, size):
        method = self.packet.read1ByteTxRx if size == 1 else self.packet.read2ByteTxRx
        value, result, error = method(self.ports[joint.bus], joint.id, address)
        self.check(result, error)
        return value

    def write_register(self, joint, address, value, size=2):
        method = self.packet.write1ByteTxRx if size == 1 else self.packet.write2ByteTxRx
        self.check(*method(self.ports[joint.bus], joint.id, address, value))

    def read(self):
        raw, speeds = {}, {}
        for bus, port in self.ports.items():
            members = [joint for joint in self.joints if joint.bus == bus]
            ids = [joint.id for joint in members]
            self.check(self.packet.syncReadTx(port, 56, 4, ids, len(ids)))
            for joint in members:
                data, result, error = self.packet.readRx(port, joint.id, 4)
                self.check(result, error)
                if len(data) != 4:
                    raise RuntimeError(f"Incomplete feedback for {joint.name}")
                raw[joint.name] = decode(data[0] | data[1] << 8)
                speeds[joint.name] = decode(data[2] | data[3] << 8)
        # Commit only complete feedback. Wheel phase wraps every revolution.
        positions, velocities = [], []
        for joint in self.joints:
            tick = raw[joint.name]
            if joint.mode == "velocity":
                phase = tick % 4096
                previous = self.previous_wheel_ticks.get(joint.name, phase)
                delta = (phase - previous + 2048) % 4096 - 2048
                self.wheel_ticks[joint.name] = self.wheel_ticks.get(joint.name, 0) + delta
                self.previous_wheel_ticks[joint.name] = phase
                positions.append(joint.direction * self.wheel_ticks[joint.name] * RAD_PER_TICK)
            else:
                positions.append(joint.position(tick))
            velocities.append(joint.direction * speeds[joint.name] * RAD_PER_TICK)
        self.raw = raw
        self.last_read = time.monotonic()
        return positions, velocities

    def activate(self):
        # Seed each target from feedback before enabling any motor. Keep position
        # servos torque-enabled on shutdown to support the arms against gravity.
        if self.last_read is None or time.monotonic() - self.last_read > 0.5:
            raise RuntimeError("Cannot enable torque from stale feedback")
        for joint in self.joints:
            if joint.mode == "position":
                joint.target(joint.position(self.raw[joint.name]))
                self.write_register(joint, 44, 0)
                self.write_register(joint, 46, max(1, round(joint.max_velocity / RAD_PER_TICK)))
                self.write_register(joint, 42, encode(self.raw[joint.name]))
            else:
                self.write_register(joint, 46, 0)
        self.enabled = True  # Also clean up if enabling only some motors succeeds.
        for joint in self.joints:
            self.write_register(joint, 40, 1, size=1)

    def write(self, positions, velocities):
        # Validate the complete command before sending the first packet.
        goals = {joint.name: joint.target(positions[joint.name]) if joint.mode == "position"
                 else joint.speed(velocities[joint.name]) for joint in self.joints}
        for bus, port in self.ports.items():
            for mode, address in (("position", 42), ("velocity", 46)):
                members = [joint for joint in self.joints if joint.bus == bus and joint.mode == mode]
                if not members:
                    continue
                group = self.sdk.GroupSyncWrite(port, self.packet, address, 2)
                for joint in members:
                    value = encode(goals[joint.name])
                    if not group.addParam(joint.id, [value & 255, value >> 8]):
                        raise RuntimeError(f"Cannot encode command for {joint.name}")
                self.check(group.txPacket())

    def stop(self):
        errors = []
        if self.enabled:
            # Best effort per motor: one disconnected bus must not prevent stops
            # being sent to reachable wheels or holds to reachable position joints.
            for joint in sorted(self.joints, key=lambda joint: joint.mode != "velocity"):
                try:
                    address = 46 if joint.mode == "velocity" else 42
                    value = 0 if joint.mode == "velocity" else encode(self.raw[joint.name])
                    self.write_register(joint, address, value)
                except Exception as error:
                    errors.append(f"{joint.name}: {error}")
            self.enabled = False
        return errors

    def close(self):
        errors = self.stop()
        for port in self.ports.values():
            port.closePort()
        self.ports.clear()
        return errors


class CommandBuffer:
    def __init__(self, joints, timeout):
        self.joints = joints
        self.timeout = timeout
        self.commands, self.seen = {}, {}

    def receive(self, mode, names, values, now):
        expected = {joint.name for joint in self.joints if joint.mode == mode}
        if len(names) != len(values) or len(set(names)) != len(names) or set(names) != expected:
            raise ValueError(f"{mode}: expected one value for each joint in {sorted(expected)}")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Non-finite hardware command")
        command = dict(zip(names, values))
        for joint in self.joints:
            if joint.mode == mode and mode == "position":
                joint.target(command[joint.name])
        self.commands[mode] = command
        self.seen[mode] = now

    def fresh(self, now):
        return all(mode in self.seen and now - self.seen[mode] <= self.timeout
                   for mode in ("position", "velocity"))
