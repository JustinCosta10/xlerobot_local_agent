"""Masked RGB-D projection. Positions describe visible surfaces, not grasp poses."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Observation:
    label: str
    position: np.ndarray
    extent: np.ndarray
    spread: np.ndarray
    confidence: float
    valid_fraction: float
    point_count: int
    crop: bytes = b""
    camera_frame: str = ""


def depth_in_meters(array, encoding):
    if array.ndim != 2:
        raise ValueError("Depth must have one channel")
    if encoding == "32FC1":
        return array.astype(np.float32, copy=False)
    if encoding in ("16UC1", "mono16"):
        return array.astype(np.float32) * .001
    raise ValueError(f"Unsupported depth encoding: {encoding}; use 32FC1 meters or 16UC1 millimeters")


def camera_parameters(info, shape):
    if (info.height, info.width) != shape or info.binning_x > 1 or info.binning_y > 1:
        raise ValueError("CameraInfo must match the RGB/depth resolution without binning")
    if info.roi.x_offset or info.roi.y_offset:
        raise ValueError("Cropped CameraInfo requires adjusted intrinsics")
    k = np.asarray(info.k, dtype=float).reshape(3, 3)
    d = np.asarray(info.d, dtype=float)
    if not np.isfinite(k).all() or not np.isfinite(d).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError("CameraInfo has invalid or uncalibrated intrinsics")
    if np.any(d) and info.distortion_model not in ("plumb_bob", "rational_polynomial"):
        raise ValueError(f"Unsupported camera distortion model: {info.distortion_model}")
    return k, d


def project_mask(mask, offset, depth, intrinsics, distortion, transform, *,
                 erosion=1, min_pixels=20, min_fraction=.5, min_depth=.1, max_depth=8.0):
    """Return a robust map-frame surface position and quality, or reject the mask."""
    x, y = offset
    height, width = mask.shape
    if min(x, y) < 0 or x + width > depth.shape[1] or y + height > depth.shape[0]:
        raise ValueError("Instance mask lies outside the original image")
    selected = np.asarray(mask != 0, dtype=np.uint8)
    if erosion:
        selected = cv2.erode(selected, np.ones((3, 3), np.uint8), iterations=erosion,
                             borderType=cv2.BORDER_CONSTANT, borderValue=0)
    count = int(selected.sum())
    if count < min_pixels:
        return None
    region = depth[y:y + height, x:x + width]
    valid = selected.astype(bool) & np.isfinite(region) & (region >= min_depth) & (region <= max_depth)
    z = region[valid]
    if len(z) < min_pixels or len(z) / count < min_fraction:
        return None
    # Reject depth bleeding across object/table edges using a robust depth band.
    median = np.median(z)
    band = max(.03, 3 * 1.4826 * float(np.median(np.abs(z - median))))
    keep = np.abs(z - median) <= band
    if int(keep.sum()) < min_pixels or keep.sum() / count < min_fraction:
        return None
    v, u = np.nonzero(valid)
    u, v, z = u[keep] + x, v[keep] + y, z[keep]
    if np.any(distortion):
        pixels = np.stack((u, v), axis=1).astype(np.float64).reshape(-1, 1, 2)
        rays = cv2.undistortPoints(pixels, intrinsics, distortion).reshape(-1, 2)
    else:
        rays = np.stack(((u - intrinsics[0, 2]) / intrinsics[0, 0],
                         (v - intrinsics[1, 2]) / intrinsics[1, 1]), axis=1)
    points = np.column_stack((rays * z[:, None], z))
    points = points @ transform[:3, :3].T + transform[:3, 3]
    if not np.isfinite(points).all():
        raise ValueError("Projection produced nonfinite points")
    position = np.median(points, axis=0)
    extent = np.percentile(points, 95, axis=0) - np.percentile(points, 5, axis=0)
    spread = 1.4826 * np.median(np.abs(points - position), axis=0)
    return position, extent, spread, float(len(points) / count), len(points)


def encode_crop(image, offset, shape, max_size=256):
    x, y = offset
    height, width = shape
    crop = image[y:y + height, x:x + width]
    if max(crop.shape[:2]) > max_size:
        scale = max_size / max(crop.shape[:2])
        crop = cv2.resize(crop, (max(1, round(width * scale)), max(1, round(height * scale))),
                          interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        raise ValueError("Could not encode object crop")
    return encoded.tobytes()
