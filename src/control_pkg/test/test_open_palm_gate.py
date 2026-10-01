"""Check release timing around closed hands and camera dropouts."""

from control_pkg.open_palm_gate import OpenPalmGate


def test_open_palm_requires_two_continuous_seconds():
    gate = OpenPalmGate(2.0, 0.4)
    assert not gate.observe(True, 0.0)
    assert gate.remaining_seconds(0.0) == 2.0
    assert not gate.observe(True, 0.3)
    assert not gate.observe(True, 0.6)
    assert not gate.observe(True, 0.9)
    assert not gate.observe(True, 1.2)
    assert not gate.observe(True, 1.5)
    assert not gate.observe(True, 1.8)
    assert gate.observe(True, 2.0)
    assert gate.remaining_seconds(2.0) == 0.0


def test_closed_hand_and_frame_gap_restart_hold_time():
    gate = OpenPalmGate(2.0, 0.4)
    assert not gate.observe(True, 0.0)
    assert not gate.observe(False, 0.3)
    assert not gate.observe(True, 0.4)
    assert not gate.observe(True, 0.7)
    assert not gate.observe(True, 2.0)
    assert not gate.observe(True, 2.3)
    assert not gate.observe(True, 2.6)
    assert not gate.observe(True, 2.9)
    assert not gate.observe(True, 3.0)
    assert gate.remaining_seconds(3.0) == 1.0
    assert not gate.observe(False, 3.1)
    assert gate.remaining_seconds(3.1) == 2.0
