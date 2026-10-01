"""Check automatic HOME recovery after ordinary task failures."""

import threading
from types import SimpleNamespace

from control_pkg.control_node import ControlNode
from control_pkg.task_state import TaskState


def make_node(home_fails=False):
    """Replace robot actions with recorded calls."""
    events = []
    node = SimpleNamespace(
        state=TaskState.PICKING,
        home_pose=[0.0] * 4,
        _ensure_active=lambda operation_id: None,
        _cancel_active_goals=lambda: events.append('cancel'),
        _publish_zero_velocity=lambda: events.append('stop_base'),
        _set_manual_drive=lambda enabled, force: events.append(
            ('manual_drive', enabled)
        ),
        _finish_task=lambda operation_id: events.append('finish'),
        get_logger=lambda: SimpleNamespace(
            error=lambda message: events.append(('error', message)),
            warn=lambda message: events.append(('warn', message)),
        ),
    )

    def set_state(state, operation_id):
        node.state = state
        events.append(('state', state))

    def send_joint_pose(positions, operation_id, label):
        events.append(('move', label))
        if home_fails:
            raise RuntimeError('controller unavailable')

    node._set_state = set_state
    node._send_joint_pose = send_joint_pose
    return node, events


def test_task_error_returns_home_before_finishing():
    node, events = make_node()

    ControlNode._fail_operation(node, 1, 'vision failed')

    assert ('state', TaskState.ERROR) in events
    assert ('state', TaskState.RETURN_HOME) in events
    assert ('move', 'HOME') in events
    assert events.index(('move', 'HOME')) < events.index('finish')
    assert ('manual_drive', False) in events


def test_home_failure_stays_in_error_for_x_retry():
    node, events = make_node(home_fails=True)

    ControlNode._fail_operation(node, 1, 'vision failed')

    assert node.state == TaskState.ERROR
    assert events.count(('move', 'HOME')) == 1
    assert ('manual_drive', True) in events
    assert 'finish' not in events


def test_x_can_retry_home_after_an_ordinary_error():
    """Manual HOME recovery works even when no emergency stop occurred."""
    events = []
    node = SimpleNamespace(
        _lock=threading.RLock(),
        state=TaskState.ERROR,
        _emergency_active=False,
        _stop_event=SimpleNamespace(clear=lambda: events.append('clear_stop')),
        _set_manual_drive=lambda enabled, force: events.append(
            ('manual_drive', enabled)
        ),
        _publish_zero_velocity=lambda: events.append('stop_base'),
        _set_state=lambda state: events.append(('state', state)),
        _home_recovery_sequence=object(),
        _start_operation=lambda worker: events.append(('start', worker)),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None,
            warn=lambda message: None,
        ),
    )

    ControlNode._handle_home_recovery(node)

    assert ('state', TaskState.RETURN_HOME) in events
    assert ('start', node._home_recovery_sequence) in events
