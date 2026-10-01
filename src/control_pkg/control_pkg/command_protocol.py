"""Validation rules for commands received from the voice pipeline."""

from dataclasses import dataclass
from typing import Collection


FETCH_ACTION = 'fetch'
STOP_ACTION = 'stop'
UNKNOWN_ACTION = 'unknown'
DEFAULT_SUPPORTED_TARGETS = ('airpod', 'coke', 'tissue', 'vaseline')


@dataclass(frozen=True)
class CommandDecision:
    """Normalized command and the result of validating it."""

    accepted: bool
    action: str
    target_class: str
    message: str


def validate_voice_command(
    action: str,
    target_class: str,
    supported_targets: Collection[str] = DEFAULT_SUPPORTED_TARGETS,
) -> CommandDecision:
    """Normalize and validate a voice command without changing control state."""
    normalized_action = str(action).strip().lower()
    normalized_target = str(target_class).strip().lower()
    allowed_targets = {str(target).strip().lower() for target in supported_targets}

    if normalized_action == UNKNOWN_ACTION:
        return CommandDecision(
            False,
            normalized_action,
            normalized_target,
            'LLM classified the request as unknown',
        )

    if normalized_action == STOP_ACTION:
        if normalized_target:
            return CommandDecision(
                False,
                normalized_action,
                normalized_target,
                'stop command must have an empty target_class',
            )
        return CommandDecision(
            True,
            normalized_action,
            normalized_target,
            'stop requested',
        )

    if normalized_action != FETCH_ACTION:
        return CommandDecision(
            False,
            normalized_action,
            normalized_target,
            f'unsupported action: {normalized_action or "<empty>"}',
        )

    if normalized_target not in allowed_targets:
        return CommandDecision(
            False,
            normalized_action,
            normalized_target,
            f'unsupported target_class: {normalized_target or "<empty>"}',
        )

    return CommandDecision(
        True,
        normalized_action,
        normalized_target,
        'target stored by control node',
    )
