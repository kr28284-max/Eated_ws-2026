"""Check that a fetch command starts recognition without an A press."""

import threading
from types import SimpleNamespace

from eated_interfaces.msg import RobotCommand
from sensor_msgs.msg import Joy

from control_pkg.control_node import ControlNode
from control_pkg.task_state import TaskState


def test_fetch_starts_immediately_and_busy_command_is_rejected():
    """A valid fetch claims control and starts one recognition worker."""
    events = []
    node = SimpleNamespace(
        _lock=threading.RLock(),
        state=TaskState.IDLE,
        supported_targets=('airpod',),
        current_target_class='',
        last_requested_target_class='',
        last_object_point=None,
        _recognition_pick_sequence=object(),
        _publish_target=lambda: events.append('target'),
        _set_manual_drive=lambda enabled, force: events.append(
            ('manual_drive', enabled)
        ),
        _publish_zero_velocity=lambda: events.append('stop_base'),
        _start_operation=lambda worker: events.append(('start', worker)),
        _publish_command_status=lambda *args: events.append(('status', args)),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None,
            warn=lambda message: None,
        ),
    )

    def set_state(state):
        node.state = state
        events.append(('state', state))

    node._set_state = set_state
    node._begin_recognition = lambda target: ControlNode._begin_recognition(
        node, target
    )
    command = RobotCommand()
    command.action = 'fetch'
    command.target_class = 'airpod'

    ControlNode.command_callback(node, command)

    assert node.state == TaskState.MOVE_TO_RECOGNITION
    assert node.current_target_class == 'airpod'
    assert node.last_requested_target_class == 'airpod'
    assert ('manual_drive', False) in events
    assert 'stop_base' in events
    assert ('start', node._recognition_pick_sequence) in events
    assert events[-1][0] == 'status'
    assert events[-1][1][0] is True

    ControlNode.command_callback(node, command)

    assert sum(
        event[0] == 'start'
        for event in events
        if isinstance(event, tuple)
    ) == 1
    assert events[-1][1][0] is False


def test_a_button_repeats_last_target_when_idle():
    """The A button remains available as a manual retry."""
    events = []
    node = SimpleNamespace(
        _previous_buttons=[],
        recognition_button_index=0,
        home_button_index=2,
        emergency_button_index=3,
        _lock=threading.RLock(),
        state=TaskState.IDLE,
        last_requested_target_class='airpod',
        _rising_edge=lambda buttons, index: ControlNode._rising_edge(
            node, buttons, index
        ),
        _handle_home_recovery=lambda: events.append('home'),
        _handle_emergency_stop=lambda: events.append('stop'),
        _handle_recognition_request=lambda: events.append('recognition'),
    )
    joy = Joy()
    joy.buttons = [1, 0, 0, 0]

    ControlNode.joy_callback(node, joy)

    assert events == ['recognition']


def test_manual_recognition_uses_last_target_only_when_idle():
    """A manual retry cannot interrupt an active task."""
    targets = []
    node = SimpleNamespace(
        _lock=threading.RLock(),
        state=TaskState.IDLE,
        last_requested_target_class='airpod',
        _begin_recognition=lambda target: targets.append(target),
        _restore_state_authority=lambda: None,
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None,
            warn=lambda message: None,
        ),
    )

    ControlNode._handle_recognition_request(node)
    node.state = TaskState.PICKING
    ControlNode._handle_recognition_request(node)

    assert targets == ['airpod']
