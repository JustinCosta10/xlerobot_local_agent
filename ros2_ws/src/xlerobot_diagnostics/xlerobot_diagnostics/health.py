"""Bridge health based on wall time, so a stopped simulation can be diagnosed."""

import math


class BridgeHealth:
    def __init__(self, stale_after=3.0, base_frame="chassis", require_clock=True,
                 expected_joints=(), require_backend_feedback=False):
        if not math.isfinite(stale_after) or stale_after <= 0:
            raise ValueError("stale_after must be a positive finite number")
        self.stale_after = stale_after
        self.base_frame = base_frame
        self.require_clock = require_clock
        self.require_backend_feedback = require_backend_feedback
        self.expected_joints = set(expected_joints)
        self.seen = {}
        self.clock_ns = None
        self.last_clock_advance = None
        self.joint_error = ""
        self.joint_names = []

    def clock(self, nanoseconds, now):
        self.seen["clock"] = now
        # A rewind begins a new epoch; require another advancing sample.
        if self.clock_ns is not None and nanoseconds < self.clock_ns:
            self.last_clock_advance = None
        elif self.clock_ns is not None and nanoseconds > self.clock_ns:
            self.last_clock_advance = now
        self.clock_ns = nanoseconds

    def joints(self, names, positions, now):
        self.seen["joint_states"] = now
        self.joint_names = list(names)
        if not names or any(not name for name in names) or len(set(names)) != len(names):
            self.joint_error = "joint names are empty or duplicated"
        elif len(names) != len(positions) or not all(math.isfinite(p) for p in positions):
            self.joint_error = "joint positions are missing or non-finite"
        elif not self.expected_joints.issubset(names):
            self.joint_error = "expected joints are missing"
        else:
            self.joint_error = ""

    def transforms(self, parent_child_pairs, now):
        if any(parent and child == self.base_frame and parent != child
               for parent, child in parent_child_pairs):
            self.seen["tf"] = now

    def backend_feedback(self, names, positions, velocities, now):
        if (names and len(set(names)) == len(names)
                and len(names) == len(positions) == len(velocities)
                and self.expected_joints.issubset(names)
                and all(math.isfinite(value) for value in (*positions, *velocities))):
            self.seen["backend_feedback"] = now

    def problems(self, now):
        problems = []
        streams = ["joint_states", "tf"]
        if self.require_clock:
            streams.insert(0, "clock")
        if self.require_backend_feedback:
            streams.append("backend_feedback")
        for stream in streams:
            if stream not in self.seen:
                problems.append(f"waiting for {stream}")
            elif now - self.seen[stream] > self.stale_after:
                problems.append(f"{stream} stale (backend paused or disconnected)")
        if self.require_clock and "clock" in self.seen and (
            self.last_clock_advance is None
            or now - self.last_clock_advance > self.stale_after
        ):
            problems.append("simulation clock is not advancing")
        if self.joint_error:
            problems.append(self.joint_error)
        return problems
