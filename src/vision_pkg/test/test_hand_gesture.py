"""Check the open-palm classifier with straight and folded fingers."""

from types import SimpleNamespace

from vision_pkg.hand_gesture import is_open_palm


def _hand():
    landmarks = [SimpleNamespace(x=0.5, y=0.5) for _ in range(21)]
    landmarks[0] = SimpleNamespace(x=0.5, y=1.0)
    for x, (mcp, pip, tip) in zip(
        (0.2, 0.4, 0.6, 0.8),
        ((5, 6, 8), (9, 10, 12), (13, 14, 16), (17, 18, 20)),
    ):
        landmarks[mcp] = SimpleNamespace(x=x, y=0.6)
        landmarks[pip] = SimpleNamespace(x=x, y=0.4)
        landmarks[tip] = SimpleNamespace(x=x, y=0.1)
    return landmarks


def test_straight_fingers_are_open():
    assert is_open_palm(_hand())


def test_one_folded_finger_is_not_open():
    landmarks = _hand()
    landmarks[8] = SimpleNamespace(x=0.2, y=0.7)
    assert not is_open_palm(landmarks)
