import sqlite3

import numpy as np

from xlerobot_perception.object_geometry import Observation
from xlerobot_perception.object_memory import ObjectMemory


def observation(x, label="cube", score=.9, crop=b"jpeg"):
    return Observation(label, np.array([x, 0., .8]), np.array([.05, .05, .01]),
                       np.array([.02, .02, 0.]), score, .9, 40, crop, "optical")


def test_eight_identical_cubes_confirm_separately_and_keep_identity_when_reordered(tmp_path):
    memory = ObjectMemory(tmp_path / "objects.sqlite3")
    memory.new_session(0, "test")
    for frame in range(3):
        observations = [observation(i * .09 + frame * .001) for i in range(8)]
        memory.process(1_000_000_000 + frame * 200_000_000, observations[::-1] if frame == 2 else observations)
        if frame < 2:
            assert not memory.query(1_400_000_000)
    records = memory.query(1_400_000_000)
    assert len(records) == 8 and len({r["object_id"] for r in records}) == 8
    assert all(r["observation_count"] == 3 for r in records)
    identities = {round(r["x"], 2): r["object_id"] for r in records}
    memory.process(1_600_000_000, [observation(i * .09 + .002) for i in reversed(range(8))])
    assert {round(r["x"], 2): r["object_id"] for r in memory.query(1_600_000_000)} == identities
    assert not memory.process(1_600_000_000, [observation(0)])
    assert all(r["observation_count"] == 4 for r in memory.query(1_600_000_000))
    memory.close()


def test_confirmation_timeout_staleness_history_and_representative_crop(tmp_path):
    memory = ObjectMemory(tmp_path / "objects.sqlite3", stale_after=2)
    memory.new_session(0, "test")
    memory.process(1_000_000_000, [observation(0, crop=b"old")])
    # An isolated false positive expires instead of accumulating over minutes.
    for stamp, score, crop in ((4_000_000_000, .8, b"first"), (4_200_000_000, .95, b"best"),
                               (4_400_000_000, .85, b"last"), (4_600_000_000, .8, b"other"),
                               (5_500_000_000, .9, b"later")):
        memory.process(stamp, [observation(0, score=score, crop=crop)])
    records = memory.query(5_500_000_000, label=" CuBe ", include_images=True)
    assert len(records) == 1 and records[0]["crop_jpeg"] == b"best"
    assert records[0]["first_seen_ns"] == 4_000_000_000
    assert records[0]["observation_count"] == 5
    assert memory.db.execute("SELECT COUNT(*) FROM observations").fetchone()[0] == 4
    assert not memory.query(8_000_000_000)
    assert memory.query(8_000_000_000, include_stale=True)[0]["stale"]
    assert not memory.query(5_500_000_000, label="mug")
    memory.close()


def test_database_survives_restarts_but_coordinates_never_cross_sessions(tmp_path):
    path = tmp_path / "objects.sqlite3"
    memory = ObjectMemory(path, min_observations=1)
    old_session = memory.new_session(0, "first map")
    memory.process(10, [observation(1, label="mug")])
    memory.close()
    memory = ObjectMemory(path, min_observations=1)
    assert memory.new_session(0, "new map") != old_session
    assert not memory.query(20, include_stale=True)
    history = memory.query(20, session_id=old_session, include_stale=True, include_images=True)
    assert len(history) == 1 and history[0]["stale"] and history[0]["crop_jpeg"] == b"jpeg"
    memory.process(10, [observation(1, label="mug")])
    current = memory.query(20)
    assert current[0]["object_id"] != history[0]["object_id"]
    assert len(memory.query(20, session_id="*", include_stale=True)) == 2
    assert not memory.query(20, label="' OR 1=1 --")
    memory.close()
    with sqlite3.connect(path) as db:
        assert db.execute("SELECT COUNT(*) FROM sessions").fetchone()[0] == 2
