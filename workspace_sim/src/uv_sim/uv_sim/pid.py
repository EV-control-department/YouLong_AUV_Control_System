"""One PID channel, with integral clamping and an optional vertical feed-forward.

Moved verbatim out of ``sim_bridge._Pid`` (T5) so the same object can be used by
the ROS-free controller core, the parameter service and the unit tests without
importing ``rclpy``.

``z_thrust_ff`` is a constant bias added to the output, used by the depth loops to
carry the vehicle's buoyancy offset.  It is added even when ``dt <= 0`` -- the
original code does that, and the first tick after a mode switch can arrive with a
zero (or negative) delta, so dropping the bias there would change behaviour.
"""

from __future__ import annotations

from dataclasses import dataclass

from uv_sim.mathutil import clamp


@dataclass
class Pid:
    kp: float
    ki: float
    kd: float
    i_limit: float = 0.5
    z_thrust_ff: float = 0.0

    def __post_init__(self) -> None:
        self.integral = 0.0
        self.prev_err = 0.0

    def reset(self) -> None:
        """Clear the integrator and the derivative history."""
        self.integral = 0.0
        self.prev_err = 0.0

    def step(self, err: float, dt: float) -> float:
        """Return the controller output for ``err`` over ``dt`` seconds."""
        if dt <= 0.0:
            return self.kp * err + self.z_thrust_ff
        self.integral = clamp(self.integral + err * dt, -self.i_limit, self.i_limit)
        derivative = (err - self.prev_err) / dt
        self.prev_err = err
        return self.kp * err + self.ki * self.integral + self.kd * derivative + self.z_thrust_ff
