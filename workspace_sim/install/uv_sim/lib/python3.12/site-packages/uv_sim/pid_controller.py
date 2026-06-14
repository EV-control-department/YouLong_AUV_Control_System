"""Reusable PID controller with anti-windup and output clamping."""

import math
from dataclasses import dataclass, field


@dataclass
class PidGains:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    i_limit: float = 0.5
    output_limit: float = 1000.0
    feedforward: float = 0.0


class PidController:
    """PID controller with integral clamping and output limiting."""

    def __init__(self, gains: PidGains = None):
        self.gains = gains or PidGains()
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True

    def step(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        g = self.gains

        # Proportional
        p = g.kp * error

        # Integral with clamping
        self._integral += error * dt
        self._integral = max(-g.i_limit, min(g.i_limit, self._integral))
        i = g.ki * self._integral

        # Derivative
        if self._first:
            d = 0.0
            self._first = False
        else:
            d = g.kd * (error - self._prev_error) / dt
        self._prev_error = error

        # Sum with output clamping
        output = p + i + d + g.feedforward
        output = max(-g.output_limit, min(g.output_limit, output))

        return output

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True

    def set_gains(self, gains: PidGains):
        self.gains = gains

    def get_state(self) -> dict:
        return {
            'kp': self.gains.kp,
            'ki': self.gains.ki,
            'kd': self.gains.kd,
            'integral': self._integral,
            'prev_error': self._prev_error,
        }
