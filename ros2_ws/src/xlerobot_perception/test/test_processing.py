from threading import Thread

import numpy as np
import pytest
from std_msgs.msg import Header

from xlerobot_perception.processing import LatestFrame, messages_from_arrays, render_segmentation


def test_semantic_overlap_unknown_and_detection_coordinates():
    header = Header(frame_id="camera_color_optical_frame")
    header.stamp.sec = 42
    masks = np.zeros((3, 3, 5), dtype=np.float32)
    masks[0, :2, :3] = 1
    masks[1, 1:, 1:3] = 1
    masks[2, :2, 3] = 1
    detections, labels = messages_from_arrays(
        header, (3, 5), np.array([[0, 0, 3, 2], [1, 1, 3, 3], [3, 0, 4, 2]]),
        np.array([.9, .6, .8]), np.array([300, 4, 300]), masks)
    assert labels.tolist() == [[301, 301, 301, 301, 0], [301, 301, 301, 301, 0], [0, 5, 5, 0, 0]]
    assert labels.dtype == np.uint16
    assert detections.header == header
    detection = detections.detections[0]
    assert detection.header == header
    assert detection.results[0].hypothesis.class_id == "301"
    assert detection.results[0].hypothesis.score == .9
    assert (detection.bbox.center.position.x, detection.bbox.center.position.y) == (1.5, 1.)
    assert (detection.bbox.size_x, detection.bbox.size_y) == (3., 2.)
    assert detection.id == ""  # No invented tracking identity.


def test_empty_frame_clears_masks_and_detections():
    detections, labels = messages_from_arrays(Header(), (4, 7), [], [], [], None)
    assert not detections.detections
    assert labels.shape == (4, 7) and not labels.any()


def test_mask_resolution_must_match_original_image():
    with pytest.raises(ValueError, match="full-resolution"):
        messages_from_arrays(Header(), (4, 8), [[0, 0, 2, 2]], [.8], [0], np.ones((1, 2, 2)))


def test_overlay_preserves_unknown_pixels_and_colors_by_class():
    image = np.full((10, 12, 3), 100, dtype=np.uint8)
    labels = np.zeros((10, 12), dtype=np.uint16)
    labels[1:3, 1:3] = labels[5:7, 7:9] = 1234
    detections, _ = messages_from_arrays(Header(), (10, 12), [], [], [], None)
    output = render_segmentation(image, labels, detections, {}, .5)
    np.testing.assert_array_equal(output[labels == 0], image[labels == 0])
    np.testing.assert_array_equal(output[1, 1], output[5, 7])
    assert (output[1, 1] != image[1, 1]).any()
    assert (image == 100).all()


def test_single_pending_frame_and_close_wakes_waiter():
    pending = LatestFrame()
    for frame in range(100):
        pending.put(frame)
    assert pending.take() == 99
    assert pending.take(timeout=0) is None
    received = []
    worker = Thread(target=lambda: received.append(pending.take(timeout=10)))
    worker.start()
    pending.close()
    worker.join(timeout=1)
    assert not worker.is_alive() and received == [None]
