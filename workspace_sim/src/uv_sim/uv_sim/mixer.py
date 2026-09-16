"""4-DOF body force to 6-thruster mixing for the xunyun vehicle.

Thruster layout (``xunyun_fixed.scn``)::

    T0 aft-starboard diagonal   T1 aft-port diagonal
    T2 heave bow                T3 heave stern
    T4 fwd-starboard diagonal   T5 fwd-port diagonal

The maths is **moved verbatim** from ``sim_bridge._publish_thrust_from_4dof``
(T5), including the two scale factors that the never-imported
``thrust_mixer.ThrustMixer`` had drifted away from:

* yaw enters at **×1.0** (the dead class used ×0.5)
* heave enters at **×0.8** (the dead class used ×1.0)

The golden fixture ``test/data/thrust_golden.json`` pins both (see
``test_mcu_control.py``), because "reuse the existing ThrusterMixer" was the
obvious-looking thing to do and it would have silently changed the vehicle's
yaw authority and depth authority at the same time.
"""

from __future__ import annotations

from uv_sim.mathutil import clamp

#: SIL: the PID emits forces in the same units as the tuned gains (thousands).
MAX_THRUST_SIL = 1000.0

#: HIL: the real MCU hands over forces already normalised to [-1, 1].
MAX_THRUST_HIL = 1.0


def mix_thruster_forces(
    fx: float,
    fy: float,
    fz: float,
    mz: float,
    max_thrust: float = MAX_THRUST_SIL,
) -> list[float]:
    """Mix 4-DOF body force ``[Fx, Fy, Fz, Mz]`` into 6 commands in ``[-1, 1]``.

    ``max_thrust`` is the value the forces are divided by before clamping; pass
    :data:`MAX_THRUST_HIL` when the forces arrive from the real MCU.
    """
    h0 = (fx + fy - mz)     # T0: aft-stbd diagonal
    h1 = (fx - fy + mz)     # T1: aft-port diagonal
    h4 = -(fx - fy - mz)    # T4: fwd-stbd diagonal
    h5 = -(fx + fy + mz)    # T5: fwd-port diagonal
    h2 = fz * 0.8           # T2: heave bow
    h3 = fz * 0.8           # T3: heave stern

    raw = [h0, h1, h2, h3, h4, h5]
    for index in range(len(raw)):
        raw[index] = clamp(raw[index] / max_thrust, -1.0, 1.0)
    return raw
