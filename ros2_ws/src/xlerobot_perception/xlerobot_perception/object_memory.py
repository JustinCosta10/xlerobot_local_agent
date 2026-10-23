"""Session-scoped object association and transactional SQLite observation history."""

from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
import json
from pathlib import Path
import sqlite3
from uuid import uuid4

import numpy as np
from scipy.optimize import linear_sum_assignment

from .object_geometry import Observation


@dataclass
class Track:
    object_id: str
    observation: Observation
    first_ns: int
    last_ns: int
    hits: int = 1
    confirmed: bool = False
    history_ns: int = -1
    pending: list = field(default_factory=list)
    crop: bytes = b""
    crop_score: float = -1.0
    crop_stamp_ns: int = 0
    crop_frame: str = ""


def details(observation):
    return {"extent": observation.extent.tolist(), "spread": observation.spread.tolist(),
            "valid_fraction": observation.valid_fraction, "point_count": observation.point_count,
            "camera_frame": observation.camera_frame}


class ObjectMemory:
    """One writer, called by the mapping node's mutually exclusive callbacks.

    Each frame uses a one-to-one assignment within a same-label distance gate.
    This is spatial association, not visual re-identification of identical items.
    """

    def __init__(self, path, frame="map", min_observations=3, association_distance=.08,
                 confirmation_window=2.0, stale_after=10.0, history_period=1.0):
        self.path = str(Path(path).expanduser().resolve())
        Path(self.path).parent.mkdir(parents=True, exist_ok=True)
        self.frame = frame
        self.min_observations = min_observations
        self.association_distance = association_distance
        self.confirmation_window_ns = round(confirmation_window * 1e9)
        self.stale_ns = round(stale_after * 1e9)
        self.history_period_ns = round(history_period * 1e9)
        # The ROS executor may spin on a different thread from construction;
        # all database operations still run serially in the default callback group.
        self.db = sqlite3.connect(self.path, timeout=2, check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.db.execute("PRAGMA foreign_keys=ON")
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA synchronous=NORMAL")
        version = self.db.execute("PRAGMA user_version").fetchone()[0]
        if version not in (0, 1):
            self.db.close()
            raise ValueError(f"Unsupported object database version: {version}")
        self.db.executescript("""
            CREATE TABLE IF NOT EXISTS sessions (
                session_id TEXT PRIMARY KEY, frame_id TEXT NOT NULL,
                created_utc TEXT NOT NULL, start_stamp_ns INTEGER NOT NULL, reason TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS objects (
                object_id TEXT PRIMARY KEY, session_id TEXT NOT NULL REFERENCES sessions(session_id),
                label TEXT NOT NULL, x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL,
                confidence REAL NOT NULL, first_seen_ns INTEGER NOT NULL, last_seen_ns INTEGER NOT NULL,
                observation_count INTEGER NOT NULL, metadata TEXT NOT NULL,
                crop_jpeg BLOB, crop_stamp_ns INTEGER NOT NULL, crop_frame TEXT NOT NULL);
            CREATE INDEX IF NOT EXISTS objects_by_session_label ON objects(session_id, label);
            CREATE TABLE IF NOT EXISTS observations (
                observation_id INTEGER PRIMARY KEY,
                object_id TEXT NOT NULL REFERENCES objects(object_id), stamp_ns INTEGER NOT NULL,
                x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL,
                confidence REAL NOT NULL, metadata TEXT NOT NULL,
                UNIQUE(object_id, stamp_ns));
            PRAGMA user_version=1;
        """)
        self.session_id = ""
        self.tracks = {}
        self.last_stamp_ns = -1

    def new_session(self, stamp, reason):
        self.session_id = str(uuid4())
        self.tracks.clear()
        self.last_stamp_ns = -1
        with self.db:
            self.db.execute("INSERT INTO sessions VALUES (?, ?, ?, ?, ?)", (
                self.session_id, self.frame, datetime.now(timezone.utc).isoformat(), stamp, reason))
        return self.session_id

    def process(self, stamp, observations):
        if not self.session_id:
            raise RuntimeError("Start an object-memory session before processing frames")
        if stamp <= self.last_stamp_ns:
            return False
        self.last_stamp_ns = stamp
        self.tracks = {key: track for key, track in self.tracks.items()
                       if track.confirmed or stamp - track.first_ns <= self.confirmation_window_ns}
        tracks = list(self.tracks.values())
        matches = {}
        if tracks and observations:
            costs = np.full((len(tracks), len(observations)), 1e6)
            for i, track in enumerate(tracks):
                for j, observation in enumerate(observations):
                    distance = np.linalg.norm(track.observation.position - observation.position)
                    if track.observation.label == observation.label and distance <= self.association_distance:
                        costs[i, j] = distance
            rows, columns = linear_sum_assignment(costs)
            matches = {j: tracks[i] for i, j in zip(rows, columns) if costs[i, j] < 1e6}
        with self.db:
            for index, observation in enumerate(observations):
                track = matches.get(index)
                if track is None:
                    track = Track(str(uuid4()), observation, stamp, stamp)
                    self.tracks[track.object_id] = track
                else:
                    position = .5 * track.observation.position + .5 * observation.position
                    track.observation = replace(observation, position=position)
                    track.hits += 1
                    track.last_ns = stamp
                if observation.crop and observation.confidence > track.crop_score:
                    track.crop, track.crop_score = observation.crop, observation.confidence
                    track.crop_stamp_ns, track.crop_frame = stamp, observation.camera_frame
                just_confirmed = not track.confirmed and track.hits >= self.min_observations
                if not track.confirmed:
                    track.pending.append((stamp, observation))
                track.confirmed |= just_confirmed
                if not track.confirmed:
                    continue
                self._save_track(track)
                if just_confirmed:
                    for observed_stamp, observed in track.pending:
                        self._save_observation(track.object_id, observed_stamp, observed)
                    track.pending.clear()
                    track.history_ns = stamp
                elif stamp - track.history_ns >= self.history_period_ns:
                    self._save_observation(track.object_id, stamp, observation)
                    track.history_ns = stamp
        return True

    def _save_track(self, track):
        observation = track.observation
        self.db.execute("""
            INSERT INTO objects VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            ON CONFLICT(object_id) DO UPDATE SET
                x=excluded.x, y=excluded.y, z=excluded.z, confidence=excluded.confidence,
                last_seen_ns=excluded.last_seen_ns, observation_count=excluded.observation_count,
                metadata=excluded.metadata, crop_jpeg=excluded.crop_jpeg,
                crop_stamp_ns=excluded.crop_stamp_ns, crop_frame=excluded.crop_frame
        """, (track.object_id, self.session_id, observation.label,
              *map(float, observation.position), observation.confidence, track.first_ns, track.last_ns,
              track.hits, json.dumps(details(observation)), track.crop,
              track.crop_stamp_ns, track.crop_frame))

    def _save_observation(self, object_id, stamp, observation):
        self.db.execute("""INSERT OR IGNORE INTO observations
            (object_id, stamp_ns, x, y, z, confidence, metadata) VALUES (?, ?, ?, ?, ?, ?, ?)""",
                        (object_id, stamp, *map(float, observation.position), observation.confidence,
                         json.dumps(details(observation))))

    def query(self, now, *, label="", object_id="", session_id="", include_stale=False,
              include_images=False, limit=100):
        conditions, values = [], []
        session = session_id or self.session_id
        if session != "*":
            conditions.append("o.session_id=?")
            values.append(session)
        if label:
            conditions.append("o.label=?")
            values.append(" ".join(label.split()).casefold())
        if object_id:
            conditions.append("o.object_id=?")
            values.append(object_id)
        if not include_stale:
            conditions.extend(("o.session_id=?", "o.last_seen_ns>=?"))
            values.extend((self.session_id, now - self.stale_ns))
        columns = "o.*" if include_images else (
            "o.object_id,o.session_id,o.label,o.x,o.y,o.z,o.confidence,o.first_seen_ns,"
            "o.last_seen_ns,o.observation_count,o.metadata,o.crop_stamp_ns,o.crop_frame")
        query = f"SELECT {columns},s.frame_id FROM objects o JOIN sessions s USING(session_id)"
        if conditions:
            query += " WHERE " + " AND ".join(conditions)
        query += " ORDER BY o.first_seen_ns,o.object_id"
        if limit is not None:
            query += " LIMIT ?"
            values.append(max(1, min(limit or 100, 200)))
        result = []
        for row in self.db.execute(query, values):
            record = dict(row)
            record["metadata"] = json.loads(record["metadata"])
            record["stale"] = record["session_id"] != self.session_id or now - record["last_seen_ns"] > self.stale_ns
            result.append(record)
        return result

    def close(self):
        self.db.close()
