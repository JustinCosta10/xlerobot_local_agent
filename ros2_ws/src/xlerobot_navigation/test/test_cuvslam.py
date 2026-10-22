import struct
from collections import deque
from types import SimpleNamespace

import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.clock import ClockType
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from tf2_ros import Buffer

from xlerobot_navigation.depth_conversion import millimeter_depth
from xlerobot_navigation.guard import MappingStatus
from xlerobot_navigation.vslam_localization import VslamLocalization, initial_alignment, matrix, set_pose


def transform(xyz, rpy):
    result = np.eye(4)
    result[:3, 3] = xyz
    result[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    return result


def test_depth_units_invalid_pixels_padding_and_byte_order():
    message = Image(height=2, width=3, encoding="32FC1", is_bigendian=1, step=16)
    message.header.stamp = Time(seconds=12).to_msg()
    message.header.frame_id = "camera_color_optical_frame"
    message.data = struct.pack(">8f", 1.25, float("nan"), float("inf"), 42, -1, 0, 70, 42)
    converted = millimeter_depth(message)
    assert converted.header == message.header
    assert converted.encoding == "mono16" and converted.step == 6
    assert np.frombuffer(converted.data, dtype="<u2").tolist() == [1250, 0, 0, 0, 0, 0]


def test_visual_pose_removes_head_motion_and_retains_base_tilt():
    mount0 = transform([.1, 0, 1.2], [0, -.15, .2])
    sensor0 = transform([0, 0, 0], [0, 0, 0])
    alignment = initial_alignment(sensor0, mount0, .0635)
    actual_base = transform([.9, -.2, .07], [.03, -.04, .5])
    mount1 = transform([.1, .02, 1.18], [0, .3, -.4])
    # Simulate exactly the sensor pose a visual estimator should observe.
    observed_sensor = np.linalg.inv(alignment) @ actual_base @ mount1
    buffer = Buffer()
    odom_base = transform([.8, -.1, 0], [0, 0, .4])
    for parent, child, value in [("base_link", "head_camera_link", mount1),
                                 ("odom", "base_link", odom_base)]:
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = parent, child
        tf.header.stamp = Time(seconds=10).to_msg()
        set_pose(tf.transform.translation, tf.transform.rotation, value)
        buffer.set_transform(tf, "test")
    observation = Odometry()
    observation.header.frame_id, observation.child_frame_id = "vslam_map", "head_camera_link"
    observation.header.stamp = Time(seconds=10).to_msg()
    set_pose(observation.pose.pose.position, observation.pose.pose.orientation, observed_sensor)
    sent, poses = [], []
    state = SimpleNamespace(
        buffer=buffer, pending=deque([observation]), alignment=alignment,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=10.1, clock_type=ClockType.ROS_TIME)),
        broadcaster=SimpleNamespace(sendTransform=sent.append),
        output=SimpleNamespace(publish=poses.append))
    state.transform = lambda *args: VslamLocalization.transform(state, *args)
    VslamLocalization.update(state)
    assert sent[0].header.stamp == observation.header.stamp
    assert sent[0].header.frame_id == "map" and sent[0].child_frame_id == "odom"
    correction = sent[0].transform
    np.testing.assert_allclose(matrix(correction.translation, correction.rotation) @ odom_base,
                               actual_base, atol=1e-12)
    np.testing.assert_allclose(matrix(poses[0].pose.position, poses[0].pose.orientation),
                               actual_base, atol=1e-12)


def test_visual_tracking_loss_blocks_even_with_fresh_pose_and_map(monkeypatch):
    monkeypatch.setattr("xlerobot_navigation.guard.time.monotonic", lambda: 100.)
    state = SimpleNamespace(
        tracking_ok=True,
        samples={key: (100., 10.) for key in ("depth", "map", "pose", "tracking")},
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=10)))
    assert MappingStatus.ready(state)
    state.tracking_ok = False
    assert not MappingStatus.ready(state)
    state.tracking_ok = True
    state.samples["tracking"] = (97., 7.)
    assert not MappingStatus.ready(state)
