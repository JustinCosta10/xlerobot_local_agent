"""Bounded timestamp caches for delayed inference and original camera data."""

import time


def stamp_ns(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


class StampCache:
    def __init__(self, capacity, max_wait):
        self.capacity, self.max_wait = capacity, max_wait
        self.frames = {}

    def put(self, message):
        self.frames[stamp_ns(message)] = (time.monotonic(), message)
        while len(self.frames) > self.capacity:
            del self.frames[min(self.frames)]

    def get(self, stamp):
        entry = self.frames.get(stamp)
        return None if entry is None else entry[1]

    def nearest(self, stamp, tolerance):
        if not self.frames:
            return None
        key = min(self.frames, key=lambda value: abs(value - stamp))
        return self.get(key) if abs(key - stamp) <= tolerance else None

    def calibration(self, stamp, tolerance):
        # Camera calibration can be latched/slow, but must not come from a later
        # camera configuration. Prefer an exact/near capture match when present.
        near = self.nearest(stamp, tolerance)
        if near is not None:
            return near
        older = [key for key in self.frames if key <= stamp]
        return self.get(max(older)) if older else None

    def expire(self):
        cutoff = time.monotonic() - self.max_wait
        self.frames = {key: value for key, value in self.frames.items() if value[0] >= cutoff}

    def discard_through(self, stamp):
        self.frames = {key: value for key, value in self.frames.items() if key > stamp}

    def clear(self):
        self.frames.clear()
