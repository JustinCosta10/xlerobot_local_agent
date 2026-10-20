import math

from geometry_msgs.msg import Pose, Transform

from xlerobot_navigation.guard import fresh
from xlerobot_navigation.sim_localization import correction


def test_localization_compensates_wheel_drift_and_spawn_heading():
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = -1.15, 0.2, 0.0635
    pose.orientation.z = 1.0  # Actual heading pi, odometry heading zero.
    pose.orientation.w = 0.0
    odom = Transform()
    odom.translation.x = 0.9
    odom.rotation.w = 1.0
    x, y, z, angle = correction(pose, odom)
    assert math.isclose(x, -0.25)
    assert math.isclose(y, 0.2)
    assert math.isclose(z, 0.0635)
    assert math.isclose(angle, math.pi)


def test_clock_pause_and_old_or_future_data_cannot_authorize_motion():
    assert fresh((10, 100), 10.1, 100.1, 0.5)
    assert not fresh((10, 100), 11, 100, 0.5)  # paused simulation
    assert not fresh((10, 99), 10.1, 100, 0.5)  # recently received old data
    assert not fresh((10, 101), 10.1, 100, 0.5)  # wrong time domain
    assert not fresh(None, 10, 100, 0.5)
