"""NED coordinate frame with 3D homogeneous transform support.

A Coordinate represents a 6-DOF pose (x, y, z, rx, ry, rz) in the NED frame
and supports transform_to / transform_from via 4×4 homogeneous matrices.

Units: internally degrees for angles; math functions use radians internally.
Design based on AUV2026 CoordinateSystem.py.
"""

import math
from dataclasses import dataclass

PI = math.pi
DEG2RAD = PI / 180.0
RAD2DEG = 180.0 / PI


def _wrap_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def _wrap_rad(angle: float) -> float:
    while angle > PI:
        angle -= 2.0 * PI
    while angle < -PI:
        angle += 2.0 * PI
    return angle


def _mat4_mul(a: list, b: list) -> list:
    """Multiply two 4×4 matrices laid out as flat 16-element lists."""
    r = [0.0] * 16
    for i in range(4):
        for j in range(4):
            r[i * 4 + j] = sum(a[i * 4 + k] * b[k * 4 + j] for k in range(4))
    return r


@dataclass
class Coordinate:
    """NED 6-DOF pose (degrees internally)."""

    x: float = 0.0       # North  (m)
    y: float = 0.0       # East   (m)
    z: float = 0.0       # Down   (m)
    rx: float = 0.0      # Roll   (deg)
    ry: float = 0.0      # Pitch  (deg)
    rz: float = 0.0      # Yaw    (deg, 0=N, CW+)

    # ── matrix cache ────────────────────────────────────────────────
    _h: list = None      # 4×4 homogeneous matrix (lazy)

    # ── constructors ────────────────────────────────────────────────

    @classmethod
    def from_zit6_pos(cls, data: list):
        """From /zit6/state/pos Float32MultiArray [x, y, z, yaw_rad]."""
        if len(data) >= 4:
            return cls(x=data[0], y=data[1], z=data[2],
                       rz=math.degrees(data[3]))
        return cls()

    @classmethod
    def from_dict(cls, d: dict):
        """From dict with keys x, y, z, rx, ry, rz (degrees)."""
        return cls(x=d.get('x', 0.0), y=d.get('y', 0.0),
                   z=d.get('z', 0.0), rz=d.get('rz', 0.0))

    # ── serialization ───────────────────────────────────────────────

    def to_dict(self) -> dict:
        return {'x': self.x, 'y': self.y, 'z': self.z,
                'rx': self.rx, 'ry': self.ry, 'rz': self.rz}

    def to_zit6_pos(self) -> list:
        """Float32MultiArray format [x, y, z, yaw_rad]."""
        return [self.x, self.y, self.z, math.radians(self.rz)]

    def copy(self):
        return Coordinate(self.x, self.y, self.z, self.rx, self.ry, self.rz)

    # ── matrix computation ──────────────────────────────────────────

    def extract(self):
        """Build 4×4 homogeneous transform matrix (NED ZYX convention)."""
        sx, cx = math.sin(self.rx * DEG2RAD), math.cos(self.rx * DEG2RAD)
        sy, cy = math.sin(self.ry * DEG2RAD), math.cos(self.ry * DEG2RAD)
        sz, cz = math.sin(self.rz * DEG2RAD), math.cos(self.rz * DEG2RAD)

        h = [0.0] * 16
        # R = Rz(yaw) * Ry(pitch) * Rx(roll)  (ZYX extrinsic = Rz·Ry·Rx)
        h[0] = cy * cz
        h[1] = sx * sy * cz - cx * sz
        h[2] = cx * sy * cz + sx * sz
        h[3] = self.x

        h[4] = cy * sz
        h[5] = sx * sy * sz + cx * cz
        h[6] = cx * sy * sz - sx * cz
        h[7] = self.y

        h[8] = -sy
        h[9] = sx * cy
        h[10] = cx * cy
        h[11] = self.z

        h[12] = 0.0
        h[13] = 0.0
        h[14] = 0.0
        h[15] = 1.0

        self._h = h
        return h

    def r_extract(self):
        """Decompose internal h_matrix back to x, y, z, rx, ry, rz."""
        if self._h is None:
            return
        h = self._h
        self.x = h[3]
        self.y = h[7]
        self.z = h[11]

        if abs(h[8]) < 0.999999:
            ry = -math.asin(h[8])
            self.ry = ry * RAD2DEG
            self.rx = math.atan2(h[9] / math.cos(ry), h[10] / math.cos(ry)) * RAD2DEG
            self.rz = math.atan2(h[4] / math.cos(ry), h[0] / math.cos(ry)) * RAD2DEG
        else:
            # Gimbal lock — give up on roll/pitch, yaw still valid from h[0],h[4]
            self.rz = math.atan2(h[4], h[0]) * RAD2DEG

    # ── transforms ──────────────────────────────────────────────────

    def transform_to(self, target: 'Coordinate') -> 'Coordinate':
        """Return target expressed in THIS frame (world → self coords)."""
        self.extract()
        target.extract()
        inv = self._invert_matrix(self._h)
        result = Coordinate()
        result._h = _mat4_mul(inv, target._h)
        result.r_extract()
        return result

    def transform_from(self, local: 'Coordinate') -> 'Coordinate':
        """Return local (in THIS frame) expressed in world coords (self → world)."""
        self.extract()
        local.extract()
        result = Coordinate()
        result._h = _mat4_mul(self._h, local._h)
        result.r_extract()
        return result

    # ── 2D convenience (roll=pitch=0) ───────────────────────────────

    def body_to_world(self, dx_b: float, dy_b: float,
                       dz: float = 0.0) -> 'Coordinate':
        """Body-frame displacement → world displacement (2D yaw only)."""
        yaw_rad = self.rz * DEG2RAD
        cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
        dx_w = cy * dx_b - sy * dy_b
        dy_w = sy * dx_b + cy * dy_b
        return Coordinate(x=dx_w, y=dy_w, z=dz)

    def world_to_body(self, dx_w: float, dy_w: float,
                       dz: float = 0.0) -> 'Coordinate':
        """World displacement → body-frame displacement (2D yaw only)."""
        yaw_rad = self.rz * DEG2RAD
        cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
        dx_b = cy * dx_w + sy * dy_w
        dy_b = -sy * dx_w + cy * dy_w
        return Coordinate(x=dx_b, y=dy_b, z=dz)

    def offset(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
               drx: float = 0.0, dry: float = 0.0, drz: float = 0.0) -> 'Coordinate':
        """Return a new Coordinate offset from this one (world-frame)."""
        return Coordinate(x=self.x + dx, y=self.y + dy, z=self.z + dz,
                          rx=self.rx + drx, ry=self.ry + dry, rz=self.rz + drz)

    # ── utilities ───────────────────────────────────────────────────

    @staticmethod
    def wrap_deg(angle: float) -> float:
        return _wrap_deg(angle)

    @staticmethod
    def wrap_rad(angle: float) -> float:
        return _wrap_rad(angle)

    @staticmethod
    def _invert_matrix(m: list) -> list:
        """Gauss-Jordan inverse of 4×4 matrix (flat 16)."""
        aug = [0.0] * 32
        for i in range(4):
            for j in range(4):
                aug[i * 8 + j] = m[i * 4 + j]
                aug[i * 8 + j + 4] = 1.0 if i == j else 0.0

        for i in range(4):
            if aug[i * 8 + i] == 0.0:
                row = i + 1
                while row < 4 and aug[row * 8 + i] == 0.0:
                    row += 1
                if row < 4:
                    for j in range(8):
                        aug[i * 8 + j], aug[row * 8 + j] = aug[row * 8 + j], aug[i * 8 + j]
            scale = aug[i * 8 + i]
            for j in range(8):
                aug[i * 8 + j] /= scale
            for j in range(4):
                if j != i:
                    factor = aug[j * 8 + i]
                    for k in range(8):
                        aug[j * 8 + k] -= factor * aug[i * 8 + k]

        inv = [0.0] * 16
        for i in range(4):
            for j in range(4):
                inv[i * 4 + j] = aug[i * 8 + j + 4]
        return inv

    @property
    def yaw_rad(self) -> float:
        return self.rz * DEG2RAD

    @yaw_rad.setter
    def yaw_rad(self, value: float):
        self.rz = value * RAD2DEG

    @property
    def yaw_deg(self) -> float:
        return self.rz

    @yaw_deg.setter
    def yaw_deg(self, value: float):
        self.rz = value

    def __repr__(self):
        return (f"Coordinate(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, "
                f"rz={self.rz:.1f}°)")
