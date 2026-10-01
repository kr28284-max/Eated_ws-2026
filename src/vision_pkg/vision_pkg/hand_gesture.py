"""Geometry checks for an open hand detected by MediaPipe Hands."""

import math


def _distance(first, second) -> float:
    return math.hypot(first.x - second.x, first.y - second.y)


def _joint_angle(base, joint, tip) -> float:
    first = (base.x - joint.x, base.y - joint.y)
    second = (tip.x - joint.x, tip.y - joint.y)
    length = math.hypot(*first) * math.hypot(*second)
    if length == 0.0:
        return 0.0
    cosine = sum(a * b for a, b in zip(first, second)) / length
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def is_open_palm(landmarks) -> bool:
    """Require all four fingers to extend away from the wrist."""
    if len(landmarks) < 21:
        return False
    wrist = landmarks[0]
    for mcp, pip, tip in ((5, 6, 8), (9, 10, 12), (13, 14, 16),
                          (17, 18, 20)):
        if _joint_angle(
            landmarks[mcp], landmarks[pip], landmarks[tip]
        ) < 150.0:
            return False
        if _distance(landmarks[tip], wrist) < (
            1.1 * _distance(landmarks[pip], wrist)
        ):
            return False
    return True
