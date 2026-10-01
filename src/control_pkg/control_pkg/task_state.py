"""Task states and base-driving policy for the robot controller."""

from enum import Enum


class TaskState(str, Enum):
    """Top-level pick-and-place state."""

    IDLE = 'IDLE'
    MOVE_TO_RECOGNITION = 'MOVE_TO_RECOGNITION'
    VISION_SEARCH = 'VISION_SEARCH'
    PICKING = 'PICKING'
    MOVE_TO_CARGO = 'MOVE_TO_CARGO'
    DRIVE_TO_PLACE = 'DRIVE_TO_PLACE'
    PLACING = 'PLACING'
    RETURN_HOME = 'RETURN_HOME'
    ERROR = 'ERROR'


MANUAL_DRIVE_STATES = {
    TaskState.IDLE,
    TaskState.DRIVE_TO_PLACE,
    TaskState.ERROR,
}


def manual_drive_allowed(state: TaskState) -> bool:
    """Return whether joystick base driving is permitted in a state."""
    return state in MANUAL_DRIVE_STATES
