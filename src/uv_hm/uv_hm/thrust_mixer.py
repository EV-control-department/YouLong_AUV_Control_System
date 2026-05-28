"""4-DOF force to 6-thruster mixing for the xunyun robot geometry."""


class ThrustMixer:
    """Converts 4-DOF body forces [Fx, Fy, Fz, Mz] to 6 normalized thruster commands.

    Thruster layout (xunyun):
      T0 (aft-stbd):   diagonal ~45deg   -> contributes to +Fx, +Fy, +Mz
      T1 (aft-port):   diagonal ~-45deg  -> contributes to +Fx, -Fy, -Mz
      T2 (heave port): vertical           -> contributes to +Fz
      T3 (heave stbd): vertical           -> contributes to +Fz
      T4 (fwd-stbd):   diagonal ~135deg  -> contributes to -Fx, -Fy, +Mz (negated)
      T5 (fwd-port):   diagonal ~-135deg -> contributes to -Fx, +Fy, -Mz (negated)
    """

    def __init__(self, max_thrust: float = 1000.0):
        self.max_thrust = max_thrust

    def mix(self, fx: float, fy: float, fz: float, mz: float) -> list[float]:
        """Mix 4-DOF forces into 6 thruster commands in [-1, 1]."""
        # Normalize by max thrust
        x = fx / self.max_thrust
        y = fy / self.max_thrust
        z = fz / self.max_thrust
        rz = mz / self.max_thrust

        # Horizontal thrusters (diagonal mounting at ~45 degrees)
        h0 = x + y + rz * 0.5    # T0: aft-stbd
        h1 = x - y - rz * 0.5    # T1: aft-port
        h4 = -(x - y + rz * 0.5) # T4: fwd-stbd (reversed mounting)
        h5 = -(x + y - rz * 0.5) # T5: fwd-port (reversed mounting)

        # Vertical thrusters
        h2 = z  # T2: heave port
        h3 = z  # T3: heave stbd

        # Clamp to [-1, 1]
        return [
            max(-1.0, min(1.0, h0)),
            max(-1.0, min(1.0, h1)),
            max(-1.0, min(1.0, h2)),
            max(-1.0, min(1.0, h3)),
            max(-1.0, min(1.0, h4)),
            max(-1.0, min(1.0, h5)),
        ]
