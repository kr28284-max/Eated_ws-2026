"""Pure calculation helpers for synchronized RGB-D object localization."""

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np


SUPPORTED_DEPTH_ENCODINGS = ('16UC1', '32FC1')


@dataclass(frozen=True)
class Detection:
    """Normalized object detection independent of a model implementation."""

    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def center(self) -> tuple[int, int]:
        """Return the rounded bounding-box center pixel."""
        return (
            int(round((self.x1 + self.x2) / 2.0)),
            int(round((self.y1 + self.y2) / 2.0)),
        )


def select_target_detection(
    detections: Iterable[Detection],
    target_class: str,
    confidence_threshold: float,
) -> Detection | None:
    """Select the highest-confidence detection for the requested class."""
    target = target_class.strip().lower()
    candidates = [
        detection
        for detection in detections
        if detection.class_name.strip().lower() == target
        and detection.confidence >= confidence_threshold
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda detection: detection.confidence)


def get_median_depth(
    depth_image: np.ndarray,
    u: int,
    v: int,
    encoding: str,
    depth_scale: float,
    window_size: int,
) -> float | None:
    """Return median depth in meters around a pixel, ignoring invalid data."""
    if depth_image.ndim != 2:
        raise ValueError('depth image must have exactly two dimensions')
    if window_size < 1 or window_size % 2 == 0:
        raise ValueError('depth_window_size must be a positive odd integer')
    if depth_scale <= 0.0:
        raise ValueError('depth_scale must be greater than zero')

    normalized_encoding = encoding.upper()
    if normalized_encoding not in SUPPORTED_DEPTH_ENCODINGS:
        raise ValueError(f'unsupported depth encoding: {encoding}')

    height, width = depth_image.shape
    if not (0 <= u < width and 0 <= v < height):
        return None

    radius = window_size // 2
    x_start = max(0, u - radius)
    x_end = min(width, u + radius + 1)
    y_start = max(0, v - radius)
    y_end = min(height, v + radius + 1)

    values = np.asarray(
        depth_image[y_start:y_end, x_start:x_end],
        dtype=np.float64,
    )
    if normalized_encoding == '16UC1':
        values *= depth_scale

    valid_values = values[np.isfinite(values) & (values > 0.0)]
    if valid_values.size == 0:
        return None
    return float(np.median(valid_values))


def get_detection_depth(
    depth_image: np.ndarray,
    detection: Detection,
    encoding: str,
    depth_scale: float,
    window_size: int,
) -> float | None:
    """Read center depth, then search the inner half of the detected object."""
    u, v = detection.center
    depth = get_median_depth(
        depth_image, u, v, encoding, depth_scale, window_size
    )
    if depth is not None:
        return depth

    height, width = depth_image.shape
    half_width = max(1, int((detection.x2 - detection.x1) / 4))
    half_height = max(1, int((detection.y2 - detection.y1) / 4))
    x_start = max(0, u - half_width)
    x_end = min(width, u + half_width + 1)
    y_start = max(0, v - half_height)
    y_end = min(height, v + half_height + 1)
    if x_start >= x_end or y_start >= y_end:
        return None

    patch = depth_image[y_start:y_end, x_start:x_end]
    values = np.asarray(patch, dtype=np.float64)
    if encoding.upper() == '16UC1':
        values *= depth_scale
    valid = values[np.isfinite(values) & (values > 0.0)]
    if valid.size < 10 or valid.size < 0.1 * values.size:
        return None
    return float(np.median(valid))


def deproject_pixel(
    u: int,
    v: int,
    depth_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> tuple[float, float, float]:
    """Deproject one color pixel into its optical-frame point in meters."""
    values = np.asarray([depth_m, fx, fy, cx, cy], dtype=np.float64)
    if not np.all(np.isfinite(values)):
        raise ValueError('depth and camera intrinsics must be finite')
    if depth_m <= 0.0:
        raise ValueError('depth must be greater than zero')
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError('camera focal lengths must be greater than zero')

    x = (float(u) - cx) * depth_m / fx
    y = (float(v) - cy) * depth_m / fy
    return x, y, depth_m


def median_localization(
    samples: Sequence[tuple[float, float, float, float]],
) -> tuple[float, float, float, float]:
    """Return coordinate and confidence medians for localization samples."""
    if not samples:
        raise ValueError('at least one localization sample is required')
    sample_array = np.asarray(samples, dtype=np.float64)
    if sample_array.ndim != 2 or sample_array.shape[1] != 4:
        raise ValueError('samples must contain x, y, z, confidence values')
    if not np.all(np.isfinite(sample_array)):
        raise ValueError('localization samples must be finite')
    medians = np.median(sample_array, axis=0)
    return tuple(float(value) for value in medians)
