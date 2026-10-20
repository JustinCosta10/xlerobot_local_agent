import math
from collections import deque
from types import SimpleNamespace

from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped
import numpy as np
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer

from xlerobot_navigation.guard import fresh
from xlerobot_navigation.sim_localization import SimLocalization, correction


def test_localization_compensates_wheel_drift_and_spawn_heading():
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = -1.15, 0.2, 0.0635
    pose.orientation.z = 1.0  # Actual heading pi, odometry heading zero.
    pose.orientation.w = 0.0
    odom = Transform()
    odom.translation.x = 0.9
    odom.rotation.w = 1.0
    result = correction(pose, odom)
    assert math.isclose(result.translation.x, -0.25)
    assert math.isclose(result.translation.y, 0.2)
    assert math.isclose(result.translation.z, 0.0635)
    np.testing.assert_allclose(matrix(result.translation, result.rotation)[:3, :3],
                               Rotation.from_euler('z', math.pi).as_matrix(), atol=1e-9)


def test_clock_pause_and_old_or_future_data_cannot_authorize_motion():
    assert fresh((10, 100), 10.1, 100.1, 0.5)
    assert not fresh((10, 100), 11, 100, 0.5)  # paused simulation
    assert not fresh((10, 99), 10.1, 100, 0.5)  # recently received old data
    assert not fresh((10, 101), 10.1, 100, 0.5)  # wrong time domain
    assert not fresh(None, 10, 100, 0.5)


def matrix(translation, quaternion):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([
        quaternion.x, quaternion.y, quaternion.z, quaternion.w]).as_matrix()
    result[:3, 3] = [translation.x, translation.y, translation.z]
    return result


def sample(stamp, rpy, xyz):
    message = PoseStamped()
    message.header.frame_id = 'sim_world'
    message.header.stamp = Time(seconds=stamp).to_msg()
    message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, xyz)
    (message.pose.orientation.x, message.pose.orientation.y,
     message.pose.orientation.z, message.pose.orientation.w) = map(float, Rotation.from_euler('xyz', rpy).as_quat())
    return message


def publish_corrections(buffer, messages):
    sent = []
    # Exercise the node's publication path without DDS or a running simulator.
    state = SimpleNamespace(buffer=buffer, pending=deque(messages),
                            broadcaster=SimpleNamespace(sendTransform=sent.append),
                            pose_publisher=SimpleNamespace(publish=lambda _: None))
    SimLocalization.update(state)
    return sent


def test_camera_registration_preserves_body_roll_and_pitch():
    buffer = Buffer()
    odom = TransformStamped()
    odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
    odom.header.stamp = Time(seconds=10).to_msg()
    odom.transform.translation.x, odom.transform.translation.y = 0.8, -0.2
    odom.transform.rotation.z, odom.transform.rotation.w = math.sin(.3), math.cos(.3)
    buffer.set_transform(odom, 'wheel_odometry')
    actual = sample(10, [.06, -.09, 1.2], [-1.0, .4, .07])
    tf = publish_corrections(buffer, [actual])[0]
    reconstructed = matrix(tf.transform.translation, tf.transform.rotation) @ matrix(
        odom.transform.translation, odom.transform.rotation)
    expected = matrix(actual.pose.position, actual.pose.orientation)
    # A distant observed point amplifies the projection error from discarded tilt.
    point = np.array([3.0, .2, 1.0, 1.0])
    np.testing.assert_allclose(reconstructed @ point, expected @ point, atol=1e-9)


def test_correction_is_available_at_measurement_time_without_future_shift():
    buffer = Buffer()
    messages = []
    for stamp, yaw, x in [(10.0, .2, .1), (10.2, .6, .3)]:
        odom = TransformStamped()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.header.stamp = Time(seconds=stamp).to_msg()
        odom.transform.rotation.w = 1.0
        buffer.set_transform(odom, 'wheel_odometry')
        messages.append(sample(stamp, [0, 0, yaw], [x, 0, .0635]))
    for tf in publish_corrections(buffer, messages):
        buffer.set_transform(tf, 'simulation_localization')
    for message in messages:
        tf = buffer.lookup_transform('map', 'base_link', Time.from_msg(message.header.stamp))
        np.testing.assert_allclose(matrix(tf.transform.translation, tf.transform.rotation),
                                   matrix(message.pose.position, message.pose.orientation), atol=1e-9)
