import numpy as np
import pytest
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

from xlerobot_perception.object_geometry import camera_parameters, depth_in_meters, project_mask
from xlerobot_perception.object_sync import StampCache
from xlerobot_perception.processing import instance_masks_from_arrays


def test_same_class_instances_and_overlaps_survive_cropped_mask_publication():
    header = Header(frame_id="optical")
    header.stamp.sec = 10
    masks = np.zeros((2, 12, 20))
    masks[0, 2:8, 3:10] = 1
    masks[1, 5:11, 8:15] = 1
    result = instance_masks_from_arrays(header, (12, 20), [.9, .8], [0, 0], masks, ["cube"], 7)
    assert result.header == header and result.vocabulary_revision == 7
    assert (result.image_height, result.image_width) == (12, 20)
    assert [item.instance_id for item in result.instances] == [1, 2]
    assert [item.class_name for item in result.instances] == ["cube", "cube"]
    for index, item in enumerate(result.instances):
        restored = np.zeros((12, 20), np.uint8)
        h, w = item.mask.height, item.mask.width
        restored[item.y_offset:item.y_offset + h, item.x_offset:item.x_offset + w] = (
            CvBridge().imgmsg_to_cv2(item.mask))
        np.testing.assert_array_equal(restored != 0, masks[index] != 0)
        assert item.mask.header == header
    assert not instance_masks_from_arrays(header, (12, 20), [], [], None, ["cube"], 7).instances


def test_projection_uses_crop_offsets_original_intrinsics_and_capture_transform():
    depth = np.full((40, 60), 2.0, np.float32)
    k = np.array([[100, 0, 30], [0, 100, 20], [0, 0, 1]], float)
    transform = np.eye(4)
    # A camera rotated 90 degrees about map Z, then translated.
    transform[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    transform[:3, 3] = [1, 2, 3]
    result = project_mask(np.ones((11, 11)), (35, 15), depth, k, np.zeros(5), transform)
    position, extent, spread, fraction, count = result
    np.testing.assert_allclose(position, [1, 2.2, 5], atol=1e-7)
    assert count == 81 and fraction == 1.0
    assert np.all(extent >= 0) and np.all(spread >= 0)


def test_invalid_depth_and_background_outliers_do_not_become_object_positions():
    k = np.array([[100, 0, 5], [0, 100, 5], [0, 0, 1]], float)
    depth = np.ones((10, 10), np.float32)
    depth[0, :] = np.nan
    depth[1, :] = 0
    depth[2, :] = 3  # Table/background bleeding into the mask.
    result = project_mask(np.ones((10, 10)), (0, 0), depth, k, np.zeros(5), np.eye(4), erosion=0)
    np.testing.assert_allclose(result[0][2], 1.0)
    assert result[3] == .7 and result[4] == 70
    assert project_mask(np.ones((10, 10)), (0, 0), depth * 0, k, np.zeros(5), np.eye(4)) is None
    with pytest.raises(ValueError, match="outside"):
        project_mask(np.ones((10, 10)), (1, 0), depth, k, np.zeros(5), np.eye(4))
    np.testing.assert_allclose(depth_in_meters(np.array([[1000]], np.uint16), "16UC1"), [[1]])
    with pytest.raises(ValueError, match="encoding"):
        depth_in_meters(depth, "rgb8")


def test_camera_calibration_and_depth_timestamp_matching_reject_wrong_inputs():
    info = CameraInfo(width=60, height=40, k=[100., 0., 30., 0., 100., 20., 0., 0., 1.])
    camera_parameters(info, (40, 60))
    with pytest.raises(ValueError, match="resolution"):
        camera_parameters(info, (20, 30))
    cache = StampCache(2, 10)
    for sec in (10, 11, 12):
        message = CameraInfo()
        message.header.stamp.sec = sec
        cache.put(message)
    assert cache.get(10 * 10**9) is None  # Bounded memory.
    assert cache.nearest(11 * 10**9 + 4_000_000, 5_000_000).header.stamp.sec == 11
    assert cache.nearest(11 * 10**9 + 6_000_000, 5_000_000) is None
    assert cache.calibration(10 * 10**9, 0) is None  # Never take a future calibration.
    assert cache.calibration(13 * 10**9, 0).header.stamp.sec == 12
