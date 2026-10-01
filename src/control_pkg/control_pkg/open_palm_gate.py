"""Require an uninterrupted open-palm signal before releasing an object."""


class OpenPalmGate:
    """Track the duration of fresh consecutive positive observations."""

    def __init__(self, hold_seconds: float, max_gap_seconds: float) -> None:
        if hold_seconds <= 0.0 or max_gap_seconds <= 0.0:
            raise ValueError('open-palm timing values must be positive')
        self.hold_seconds = hold_seconds
        self.max_gap_seconds = max_gap_seconds
        self.reset()

    def reset(self) -> None:
        """Discard any partial gesture."""
        self.started_at: float | None = None
        self.last_seen_at: float | None = None

    def observe(self, is_open: bool, now: float) -> bool:
        """Return true once an open hand has been seen for the hold time."""
        if not is_open:
            self.reset()
            return False
        if (
            self.last_seen_at is None
            or now - self.last_seen_at > self.max_gap_seconds
        ):
            self.started_at = now
        self.last_seen_at = now
        return now - self.started_at >= self.hold_seconds

    def remaining_seconds(self, now: float) -> float:
        """Return the time still needed for the current hand gesture."""
        if self.started_at is None:
            return self.hold_seconds
        return max(0.0, self.hold_seconds - (now - self.started_at))
