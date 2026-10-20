"""Test-only STS3215 serial emulator using pseudo terminals, never real devices.

Exercises the actual SDK wire protocol. It is not a motor dynamics model and is
deliberately unavailable as a production launch backend.
"""
import os
from pathlib import Path
import pty
import select
import tempfile
import threading
import time
import yaml


class ServoEmulator:
    def __init__(self, template):
        self.directory = tempfile.TemporaryDirectory(prefix="xlerobot-servo-test-")
        self.fds, self.buffers, self.motors = {}, {}, {}
        self.running = True
        self.respond = True
        data = yaml.safe_load(Path(template).read_text())
        for bus, settings in data["buses"].items():
            master, slave = pty.openpty()
            self.fds[bus] = (master, slave)
            self.buffers[master] = bytearray()
            self.motors[master] = {}
            settings["port"] = os.ttyname(slave)
        for name, joint in data["joints"].items():
            joint.update(zero_tick=2048, direction=1, min_tick=0, max_tick=4095)
            master = self.fds[joint["bus"]][0]
            registers = bytearray(128)
            registers[3:5] = (777).to_bytes(2, "little")
            registers[33] = int("wheel" in name)
            initial = 2128 if name == "head_pan_joint" else 2048
            registers[42:44] = initial.to_bytes(2, "little")
            registers[56:58] = initial.to_bytes(2, "little")
            self.motors[master][joint["id"]] = {"registers": registers, "phase": float(initial),
                                                 "time": time.monotonic()}
        self.config = Path(self.directory.name) / "hardware.yaml"
        self.config.write_text(yaml.safe_dump(data))
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    @staticmethod
    def update(motor):
        registers = motor["registers"]
        now = time.monotonic()
        dt = now - motor["time"]
        motor["time"] = now
        speed = 0
        if registers[40]:
            if registers[33] == 1:
                speed = int.from_bytes(registers[46:48], "little")
                signed = -(speed & 0x7fff) if speed & 0x8000 else speed
                motor["phase"] = (motor["phase"] + signed * dt) % 4096
            else:
                motor["phase"] = float(int.from_bytes(registers[42:44], "little"))
        registers[56:58] = int(motor["phase"]).to_bytes(2, "little")
        registers[58:60] = speed.to_bytes(2, "little")

    @staticmethod
    def reply(fd, servo, payload=b""):
        data = bytes([servo, len(payload) + 2, 0]) + payload
        os.write(fd, b"\xff\xff" + data + bytes([(~sum(data)) & 255]))

    def packet(self, fd, packet):
        servo, instruction, params = packet[2], packet[4], packet[5:-1]
        motors = self.motors[fd]
        if instruction == 0x82:  # synchronous read
            address, size = params[:2]
            for servo in params[2:]:
                motor = motors[servo]
                self.update(motor)
                self.reply(fd, servo, motor["registers"][address:address + size])
        elif instruction == 0x83:  # synchronous write, no status response
            address, size = params[:2]
            for offset in range(2, len(params), size + 1):
                motor = motors[params[offset]]
                self.update(motor)
                motor["registers"][address:address + size] = params[offset + 1:offset + 1 + size]
        elif instruction == 2:
            address, size = params
            self.update(motors[servo])
            self.reply(fd, servo, motors[servo]["registers"][address:address + size])
        elif instruction == 3:
            address = params[0]
            self.update(motors[servo])
            motors[servo]["registers"][address:address + len(params) - 1] = params[1:]
            self.reply(fd, servo)

    def run(self):
        while self.running:
            readable, _, _ = select.select(list(self.buffers), [], [], 0.05)
            for fd in readable:
                buffer = self.buffers[fd]
                buffer.extend(os.read(fd, 4096))
                while len(buffer) >= 6:
                    if buffer[:2] != b"\xff\xff":
                        del buffer[0]
                        continue
                    size = buffer[3] + 4
                    if len(buffer) < size:
                        break
                    packet = bytes(buffer[:size])
                    del buffer[:size]
                    if self.respond and (sum(packet[2:]) & 255) == 255:
                        self.packet(fd, packet)

    def close(self):
        self.running = False
        self.thread.join(timeout=2)
        for master, slave in self.fds.values():
            os.close(master)
            os.close(slave)
        self.directory.cleanup()
