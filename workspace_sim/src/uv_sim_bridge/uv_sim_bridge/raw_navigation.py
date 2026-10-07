"""Continuous simulated INS navigation, independent of the ZIT6 odom origin."""

from dataclasses import dataclass
import math
import threading


DVL_POSITION_BODY = (-0.375, 0.0, 0.2)


def remove_sensor_lever_arm(velocity, angular_velocity,
                            sensor_position=DVL_POSITION_BODY):
    vx, vy, vz = velocity
    wx, wy, wz = angular_velocity
    rx, ry, rz = sensor_position
    return (vx - (wy * rz - wz * ry),
            vy - (wz * rx - wx * rz),
            vz - (wx * ry - wy * rx))


def body_to_world(vx, vy, yaw):
    cy, sy = math.cos(yaw), math.sin(yaw)
    return cy * vx - sy * vy, sy * vx + cy * vy


@dataclass(frozen=True)
class RawNavSample:
    position: tuple
    velocity: tuple
    timestamp_ms: int
    valid: bool


class RawNavigation:
    """Integrate DVL/gyro samples without ever rebasing position or attitude."""

    def __init__(self, timeout_s=2.0, timestamp_origin_s=0.0):
        self.timeout_s = timeout_s
        self.timestamp_origin_s = timestamp_origin_s
        self._lock = threading.Lock()
        self._position = [0.0] * 6
        self._velocity_sensor = [0.0] * 3
        self._angular_velocity = [0.0] * 3
        self._velocity_time = None
        self._imu_time = None
        self._update_time = None
        self._timestamp_ms = 0

    def update_velocity(self, velocity, now_s):
        if len(velocity) != 3 or not all(math.isfinite(v) for v in velocity):
            return
        with self._lock:
            self._velocity_sensor[:] = velocity
            self._velocity_time = now_s

    def update_imu(self, angular_velocity, now_s):
        if len(angular_velocity) != 3 or not all(
                math.isfinite(v) for v in angular_velocity):
            return
        with self._lock:
            self._angular_velocity[:] = angular_velocity
            self._imu_time = now_s

    def snapshot(self, now_s):
        with self._lock:
            dt = (0.0 if self._update_time is None
                  else max(0.0, min(0.2, now_s - self._update_time)))
            self._update_time = now_s
            velocity_fresh = (self._velocity_time is not None
                              and 0 <= now_s - self._velocity_time <= self.timeout_s)
            imu_fresh = (self._imu_time is not None
                         and 0 <= now_s - self._imu_time <= self.timeout_s)
            angular = (tuple(self._angular_velocity) if imu_fresh
                       else (0.0, 0.0, 0.0))
            linear = (remove_sensor_lever_arm(self._velocity_sensor, angular)
                      if velocity_fresh else (0.0, 0.0, 0.0))
            # Bootstrap integration mirrors the former localization backend.
            dx, dy = body_to_world(linear[0], linear[1], self._position[5])
            self._position[0] += dx * dt
            self._position[1] += dy * dt
            self._position[2] += linear[2] * dt
            # The bootstrap algorithm integrates yaw only; IMU roll/pitch
            # orientation remains unfused, as in the original backend.
            self._position[5] = math.remainder(
                self._position[5] + angular[2] * dt, 2 * math.pi)
            valid = velocity_fresh and imu_fresh
            if valid:
                self._timestamp_ms = int((now_s - self.timestamp_origin_s) * 1000) & 0xFFFFFFFF
            return RawNavSample(tuple(self._position), linear + angular,
                                self._timestamp_ms, valid)
