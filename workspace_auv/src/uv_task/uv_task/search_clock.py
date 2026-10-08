"""Accumulate BLINE time while excluding stopped visual calibration."""
import time


class BlineSearchClock:
    def __init__(self, total_seconds, near_seconds, now=time.monotonic):
        self.total_seconds = float(total_seconds)
        self.near_seconds = float(near_seconds)
        self.now = now
        self.reset()

    def reset(self):
        self.spent = 0.0
        self.started = None
        self.near_entered_at = None

    @property
    def active(self):
        return self.started is not None

    def elapsed(self):
        return self.spent + (max(0.0, self.now()-self.started) if self.active else 0.0)

    def resume(self):
        if not self.active:
            self.started = self.now()

    def pause(self):
        if self.active:
            self.spent = self.elapsed()
            self.started = None

    def enter_near(self):
        if self.near_entered_at is None:
            self.near_entered_at = self.elapsed()

    def remaining(self):
        elapsed = self.elapsed()
        remaining = self.total_seconds-elapsed
        if self.near_entered_at is not None:
            remaining = min(remaining, self.near_seconds-(elapsed-self.near_entered_at))
        return max(0.0, remaining)
