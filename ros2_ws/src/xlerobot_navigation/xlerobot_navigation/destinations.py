"""Destination geometry, independent of ROS transport and model output."""

import json
import math

import numpy as np


class DestinationError(RuntimeError):
    def __init__(self, code, message):
        self.code = code
        super().__init__(message)


def relative_destination(start, forward, left, turn):
    c, s = math.cos(start["yaw"]), math.sin(start["yaw"])
    yaw = start["yaw"] + turn
    return dict(x=start["x"] + c*forward - s*left,
                y=start["y"] + s*forward + c*left,
                yaw=math.atan2(math.sin(yaw), math.cos(yaw)))


def approach_radius(footprint_text, padding, xy_tolerance):
    """Circle encloses the padded footprint at every heading plus goal tolerance.

    The extra 5 cm is clearance from occupied cells. Inflation costs below lethal
    remain traversable; inflating the robot by the costmap's inflation radius too
    would double-count obstacle clearance.
    """
    try:
        polygon = np.asarray(json.loads(footprint_text), dtype=float)
        if (polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3
                or not np.isfinite(polygon).all() or not math.isfinite(padding)
                or not math.isfinite(xy_tolerance) or padding < 0 or xy_tolerance < 0):
            raise ValueError()
        radius = float(np.hypot(*(np.abs(polygon) + padding).T).max())
        if radius <= 0 or radius > 2 or xy_tolerance > .5:
            raise ValueError()
        return radius + xy_tolerance + .05
    except (ValueError, TypeError):
        raise DestinationError("INVALID_FOOTPRINT", "Cannot validate the configured robot footprint and goal tolerance.")


class ApproachGrid:
    def __init__(self, message):
        meta = message.metadata
        q = meta.origin.orientation
        if (message.header.frame_id != "map" or not math.isfinite(meta.resolution)
                or meta.resolution <= 0 or not meta.size_x or not meta.size_y
                or not all(math.isfinite(v) for v in (meta.origin.position.x, meta.origin.position.y,
                                                      q.x, q.y, q.z, q.w))
                or abs(q.x) + abs(q.y) + abs(q.z) > 1e-6 or abs(abs(q.w)-1) > 1e-6
                or len(message.data) != meta.size_x * meta.size_y):
            raise DestinationError("INVALID_COSTMAP", "Expected a complete, axis-aligned Nav2 costmap in map.")
        self.resolution = float(meta.resolution)
        self.x, self.y = meta.origin.position.x, meta.origin.position.y
        self.width, self.height = meta.size_x, meta.size_y
        self.data = np.asarray(message.data, dtype=np.uint8).reshape(self.height, self.width)

    def safe(self, x, y, radius):
        # Check every cell intersecting the enclosing disk, including its interior.
        # Unknown cells and any part outside the map are not accepted as free.
        r = self.resolution
        x0, x1 = math.floor((x-radius-self.x)/r), math.floor((x+radius-self.x)/r)
        y0, y1 = math.floor((y-radius-self.y)/r), math.floor((y+radius-self.y)/r)
        if x0 < 0 or y0 < 0 or x1 >= self.width or y1 >= self.height:
            return False
        cx = self.x + (np.arange(x0, x1+1) + .5)*r
        cy = self.y + (np.arange(y0, y1+1) + .5)*r
        dx = np.maximum(np.abs(cx-x) - r/2, 0)
        dy = np.maximum(np.abs(cy-y) - r/2, 0)
        overlap = dy[:, None]**2 + dx[None, :]**2 <= radius**2
        if np.any(self.data[y0:y1+1, x0:x1+1][overlap] >= 254):
            return False
        mx, my = math.floor((x-self.x)/r), math.floor((y-self.y)/r)
        return self.data[my, mx] < 253

    def candidates(self, target, start, standoff, radius):
        bearing = math.atan2(start["y"]-target[1], start["x"]-target[0])
        rings = []
        for distance in [standoff + .2*i for i in range(5) if standoff + .2*i <= 2.000001]:
            ring = []
            for i in range(24):
                angle = bearing + i*math.pi/12
                x, y = target[0]+distance*math.cos(angle), target[1]+distance*math.sin(angle)
                if not (-8 < x < 8 and -8 < y < 8) or not self.safe(x, y, radius):
                    continue
                ring.append(dict(x=x, y=y, yaw=math.atan2(target[1]-y, target[0]-x)))
            ring.sort(key=lambda p: math.hypot(p["x"]-start["x"], p["y"]-start["y"]))
            rings.append(ring)
        # Try the preferred ring first, but don't let an unreachable ring consume
        # the whole search budget before considering farther safe candidates.
        candidates = []
        for offset in range(0, 24, 3):
            for ring in rings:
                candidates.extend(ring[offset:offset+3])
        return candidates[:24]
