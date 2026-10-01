import numpy as np
import pytest

from vision_pkg.vision_utils import Detection
from vision_pkg.vision_utils import deproject_pixel
from vision_pkg.vision_utils import get_detection_depth
from vision_pkg.vision_utils import get_median_depth
from vision_pkg.vision_utils import median_localization
from vision_pkg.vision_utils import select_target_detection


def test_selects_highest_confidence_matching_detection():
    detections = [
        Detection('coke', 0.75, 0.0, 0.0, 10.0, 10.0),
        Detection('tissue', 0.99, 0.0, 0.0, 10.0, 10.0),
        Detection('coke', 0.92, 10.0, 20.0, 30.0, 40.0),
    ]

    selected = select_target_detection(detections, 'COKE', 0.5)

    assert selected == detections[2]
    assert selected.center == (20, 30)


def test_returns_none_when_confidence_is_below_threshold():
    detections = [Detection('coke', 0.49, 0.0, 0.0, 2.0, 2.0)]

    assert select_target_detection(detections, 'coke', 0.5) is None


def test_16uc1_depth_uses_scale_and_ignores_zero():
    image = np.array(
        [
            [0, 1000, 1000],
            [1000, 5000, 1000],
            [1000, 1000, 0],
        ],
        dtype=np.uint16,
    )

    depth = get_median_depth(image, 1, 1, '16UC1', 0.001, 3)

    assert depth == pytest.approx(1.0)


def test_32fc1_depth_is_already_in_meters():
    image = np.array(
        [[np.nan, 0.4, 0.6], [0.0, 0.5, np.inf]],
        dtype=np.float32,
    )

    depth = get_median_depth(image, 1, 0, '32FC1', 0.001, 3)

    assert depth == pytest.approx(0.5)


def test_depth_uses_box_interior_when_center_has_no_depth():
    image = np.full((30, 30), 3000, dtype=np.uint16)
    image[10:21, 10:21] = 300
    image[12:19, 12:19] = 0
    detection = Detection('airpod', 0.9, 5, 5, 25, 25)

    depth = get_detection_depth(image, detection, '16UC1', 0.001, 7)

    assert depth == pytest.approx(0.3)


def test_depth_rejects_sparse_values_in_detection():
    image = np.zeros((30, 30), dtype=np.uint16)
    image[10, 10] = 300
    detection = Detection('airpod', 0.9, 5, 5, 25, 25)

    assert get_detection_depth(image, detection, '16UC1', 0.001, 7) is None


def test_deprojection_uses_optical_frame_convention():
    point = deproject_pixel(420, 290, 2.0, 500.0, 500.0, 320.0, 240.0)

    assert point == pytest.approx((0.4, 0.2, 2.0))


def test_localization_result_is_per_column_median():
    samples = [
        (0.1, -0.2, 0.3, 0.8),
        (0.2, -0.1, 0.4, 0.9),
        (8.0, 7.0, 9.0, 0.7),
    ]

    assert median_localization(samples) == pytest.approx(
        (0.2, -0.1, 0.4, 0.8)
    )
