"""Message conversion and bounded handoff, independent of the inference runtime."""

from threading import Condition

import cv2
import numpy as np
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from sensor_msgs.msg import Image
from xlerobot_perception_interfaces.msg import InstanceMask, InstanceMaskArray


class LatestFrame:
    """One replaceable pending frame, in addition to the frame being processed."""

    def __init__(self):
        self.condition = Condition()
        self.frame = None
        self.closed = False

    def put(self, frame):
        with self.condition:
            if not self.closed:
                self.frame = frame
                self.condition.notify()

    def take(self, timeout=0.5):
        with self.condition:
            self.condition.wait_for(lambda: self.frame is not None or self.closed, timeout)
            frame, self.frame = self.frame, None
            return frame

    def close(self):
        with self.condition:
            self.closed = True
            self.frame = None
            self.condition.notify_all()


def messages_from_arrays(header, shape, boxes, scores, classes, masks, class_ids=None):
    """Merge instance masks into class IDs; highest-confidence mask wins overlaps.

    Zero means unknown; class_ids maps local model indices to stable semantic
    IDs shared with Detection2D and the latched `yoloe/classes` metadata.
    No tracking or 3D pose is inferred.
    """
    labels = np.zeros(shape, dtype=np.uint16)
    detections = Detection2DArray(header=header)
    if len(boxes) != len(scores) or len(classes) != len(scores):
        raise ValueError("Inconsistent detection arrays")
    if len(boxes) and (masks is None or masks.shape != (len(boxes), *shape)):
        raise ValueError("YOLOE must return one full-resolution mask per detection")
    if any(int(cls) < 0 or int(cls) >= 65535 for cls in classes):
        raise ValueError("Model class IDs exceed the 16-bit semantic label range")
    semantic_ids = [int(cls) + 1 if class_ids is None else class_ids[int(cls)] for cls in classes]
    for index in np.argsort(scores, kind="stable"):
        labels[masks[index] > 0.5] = semantic_ids[index]
    for box, score, label in zip(boxes, scores, semantic_ids):
        x1, y1, x2, y2 = map(float, box)
        detection = Detection2D(header=header)
        detection.bbox.center.position.x = (x1 + x2) / 2
        detection.bbox.center.position.y = (y1 + y2) / 2
        detection.bbox.size_x = x2 - x1
        detection.bbox.size_y = y2 - y1
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(label)
        hypothesis.hypothesis.score = float(score)
        detection.results.append(hypothesis)
        detections.detections.append(detection)
    return detections, labels


def class_color(label):
    """Deterministic BGR color shared by all instances of a class."""
    hue = (int(label) * 47) % 180
    return tuple(int(v) for v in cv2.cvtColor(
        np.array([[[hue, 190, 245]]], dtype=np.uint8), cv2.COLOR_HSV2BGR)[0, 0])


def instance_masks_from_arrays(header, shape, scores, classes, masks, names, revision):
    """Preserve separate masks, including overlap, with explicit RGB pixel offsets."""
    result = InstanceMaskArray(header=header, image_height=shape[0], image_width=shape[1],
                               vocabulary_revision=revision)
    if len(classes) != len(scores) or (len(scores) and (
            masks is None or masks.shape != (len(scores), *shape))):
        raise ValueError("Instance masks must match detection count and original image dimensions")
    for index, (score, cls) in enumerate(zip(scores, classes)):
        mask = masks[index] > .5
        ys, xs = np.nonzero(mask)
        if not len(xs):
            continue
        x, y, right, bottom = int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1
        crop = np.ascontiguousarray(mask[y:bottom, x:right], dtype=np.uint8) * 255
        message = Image(header=header, height=bottom - y, width=right - x,
                        encoding="mono8", step=right - x, data=crop.tobytes())
        result.instances.append(InstanceMask(
            instance_id=index + 1, class_name=names[int(cls)], confidence=float(score),
            x_offset=x, y_offset=y, mask=message))
    return result


def render_segmentation(image, labels, detections, names, alpha):
    if image.shape[:2] != labels.shape:
        raise ValueError("Source and semantic mask dimensions differ")
    output = image.copy()
    for label in np.unique(labels):
        if label == 0:
            continue
        selected = labels == label
        output[selected] = np.rint(
            (1 - alpha) * image[selected] + alpha * np.array(class_color(label)))
    for detection in detections.detections:
        hypothesis = detection.results[0].hypothesis
        label = hypothesis.class_id
        box = detection.bbox
        x = max(0, min(image.shape[1] - 1, int(box.center.position.x - box.size_x / 2)))
        y = max(16, min(image.shape[0] - 1, int(box.center.position.y - box.size_y / 2)))
        text = f"{names.get(label, label)} {hypothesis.score:.0%}"
        cv2.putText(output, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 3)
        cv2.putText(output, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .5, class_color(label), 1)
    return output
