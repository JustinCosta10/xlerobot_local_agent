from xlerobot_navigation.guard import fresh


def test_clock_pause_and_old_or_future_data_cannot_authorize_motion():
    assert fresh((10, 100), 10.1, 100.1, 0.5)
    assert not fresh((10, 100), 11, 100, 0.5)  # paused simulation
    assert not fresh((10, 99), 10.1, 100, 0.5)  # recently received old data
    assert not fresh((10, 101), 10.1, 100, 0.5)  # wrong time domain
    assert not fresh(None, 10, 100, 0.5)
