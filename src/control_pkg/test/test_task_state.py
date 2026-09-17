from control_pkg.task_state import TaskState, manual_drive_allowed


def test_manual_drive_is_only_allowed_in_safe_states():
    assert manual_drive_allowed(TaskState.IDLE)
    assert manual_drive_allowed(TaskState.DRIVE_TO_PICK)
    assert manual_drive_allowed(TaskState.DRIVE_TO_PLACE)
    assert manual_drive_allowed(TaskState.ERROR)

    assert not manual_drive_allowed(TaskState.MOVE_TO_RECOGNITION)
    assert not manual_drive_allowed(TaskState.VISION_SEARCH)
    assert not manual_drive_allowed(TaskState.PICKING)
    assert not manual_drive_allowed(TaskState.MOVE_TO_CARGO)
    assert not manual_drive_allowed(TaskState.PLACING)
    assert not manual_drive_allowed(TaskState.RETURN_HOME)
