"""6-DOF force to six-thruster mixing for the YouLong geometry.

与固件控制核配合:ZIT6 原生核输出归一化 [-1,1] 的 6-DOF 机体分力
[Fx, Fy, Fz, Mroll, Mpitch, Myaw],此处把水平/垂向分力混合成 6 路推进器指令,
发给 Stonefish(/auv/sim/actuators/thruster_command)。

The signs below follow the measured force directions of the six actuator
origins in ``youlong.scn``.  Stonefish's ``inverted_setpoint`` changes the
rotor setpoint/rotation convention; the fluid thrust direction remains the
direction of the installed actuator origin.
"""

from __future__ import annotations


class ThrustMixer:
    """Converts 6-DOF body forces [Fx, Fy, Fz, Mroll, Mpitch, Myaw] to 6
    normalized thruster commands (YouLong geometry, input already [-1,1])."""

    def __init__(self, heave_factor: float = 0.8):
        self.heave_factor = heave_factor

    def mix6(self, fx: float, fy: float, fz: float, mroll: float,
             mpitch: float, myaw: float) -> list[float]:
        """Mix 6-DOF forces (normalized) into 6 thruster commands in [-1, 1].

        水平对角对:从 Fx/Fy/Myaw 合成 4 个水平推进器(T0,T1,T4,T5)。
        垂向:T2/T3 只受 Fz 作用(heave_factor 缩放)。
        Mroll/Mpitch 在真机上由电机板处理,仿真几何里记 0(firmware 里
        roll/pitch 增益通常为 0,量级忽略)。
        """
        x = fx
        y = fy
        z = fz
        rz = myaw

        # Horizontal thrusters (diagonal mounting).  These signs were checked
        # with one-thruster-at-a-time Stonefish steps in an empty pool:
        # T0/T1 point forward, T4/T5 point aft, and the port/stbd members have
        # opposite lateral components.
        h0 = x + y - rz              # T0: aft-stbd
        h1 = x - y + rz              # T1: aft-port
        h4 = -(x - y - rz)           # T4: fwd-stbd
        h5 = -(x + y + rz)           # T5: fwd-port

        # Vertical thrusters
        h2 = z * self.heave_factor   # T2: heave port
        h3 = z * self.heave_factor   # T3: heave stbd

        # Clamp to [-1, 1]
        return [
            max(-1.0, min(1.0, h0)),
            max(-1.0, min(1.0, h1)),
            max(-1.0, min(1.0, h2)),
            max(-1.0, min(1.0, h3)),
            max(-1.0, min(1.0, h4)),
            max(-1.0, min(1.0, h5)),
        ]
