"""Small, dependency-free control-loop timing statistics.

The simulator owns the real-time scheduling thread, so this module deliberately
does not depend on ROS.  Keeping the arithmetic here makes it testable without
starting Stonefish and gives the bridge a stable performance contract.
"""

from __future__ import annotations

import math
import threading
import time


class ControlLoopStats:
    """Collect a rolling window of loop frequency and deadline statistics."""

    def __init__(self, target_hz: float = 100.0) -> None:
        if not math.isfinite(float(target_hz)) or target_hz <= 0.0:
            raise ValueError("target_hz must be a positive finite number")
        self.target_hz = float(target_hz)
        self.target_period_s = 1.0 / self.target_hz
        self._lock = threading.Lock()
        self._window_started = None
        self._last_tick = None
        self._ticks = 0
        self._period_count = 0
        self._period_sum = 0.0
        self._period_sum_sq = 0.0
        self._max_jitter_s = 0.0
        self._deadline_misses = 0

    def record_tick(self, timestamp: float | None = None,
                    late_by_s: float = 0.0) -> None:
        """Record one tick and the lateness of its scheduled deadline."""
        now = time.monotonic() if timestamp is None else float(timestamp)
        late = max(0.0, float(late_by_s))
        with self._lock:
            if self._window_started is None:
                self._window_started = now
            if self._last_tick is not None:
                period = max(0.0, now - self._last_tick)
                self._period_count += 1
                self._period_sum += period
                self._period_sum_sq += period * period
                self._max_jitter_s = max(
                    self._max_jitter_s,
                    abs(period - self.target_period_s),
                )
            self._last_tick = now
            self._ticks += 1
            if late > 0.001:
                self._deadline_misses += 1

    def snapshot(self, timestamp: float | None = None,
                 *, reset: bool = False) -> dict:
        """Return JSON-safe statistics for the current rolling window."""
        now = time.monotonic() if timestamp is None else float(timestamp)
        with self._lock:
            elapsed = (max(0.0, now - self._window_started)
                       if self._window_started is not None else 0.0)
            periods = self._period_count
            mean = self._period_sum / periods if periods else 0.0
            variance = (
                self._period_sum_sq / periods - mean * mean
                if periods else 0.0
            )
            stddev = math.sqrt(max(0.0, variance))
            result = {
                'target_hz': self.target_hz,
                'window_s': elapsed,
                'ticks': self._ticks,
                'period_samples': periods,
                'frequency_hz': (
                    self._ticks / elapsed if elapsed > 0.0 else 0.0),
                'period_mean_ms': mean * 1000.0,
                'period_stddev_ms': stddev * 1000.0,
                'max_jitter_ms': self._max_jitter_s * 1000.0,
                'deadline_misses': self._deadline_misses,
            }
            if reset:
                self._window_started = None
                self._last_tick = None
                self._ticks = 0
                self._period_count = 0
                self._period_sum = 0.0
                self._period_sum_sq = 0.0
                self._max_jitter_s = 0.0
                self._deadline_misses = 0
            return result
