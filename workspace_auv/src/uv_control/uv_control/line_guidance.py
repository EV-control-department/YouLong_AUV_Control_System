"""Finite-segment guidance in odom/NED, independent of ROS and actuators."""

from dataclasses import dataclass
import math


def norm(vector):
    return math.sqrt(sum(value * value for value in vector))


def dot(left, right):
    return sum(a * b for a, b in zip(left, right))


def saturate(vector, limit):
    length = norm(vector)
    scale = min(1.0, limit / length) if length else 1.0
    return tuple(value * scale for value in vector)


def rotate_body_to_world(vector, roll_deg, pitch_deg, yaw_deg):
    roll, pitch, yaw = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    return tuple(dot(row, vector) for row in rotation)


def rotate_world_to_body(vector, roll_deg, pitch_deg, yaw_deg):
    # Columns of R are body axes expressed in world coordinates.
    axes = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    return tuple(dot(rotate_body_to_world(axis, roll_deg, pitch_deg, yaw_deg),
                     vector) for axis in axes)


@dataclass(frozen=True)
class LineConfig:
    default_speed: float = 0.15
    max_speed: float = 0.25
    acceleration: float = 0.05
    deceleration: float = 0.05
    vector_acceleration: float = 0.05
    brake_acceleration: float = 0.03
    cross_gain: float = 0.5
    cross_speed: float = 0.10
    capture_enter: float = 0.30
    capture_exit: float = 0.15
    capture_speed: float = 0.05
    reverse_speed: float = 0.05
    terminal_speed: float = 0.08
    terminal_radius: float = 0.20
    position_tolerance: float = 0.10
    stop_speed: float = 0.03
    yaw_tolerance: float = 5.0
    stable_seconds: float = 1.0
    align_timeout: float = 30.0
    yaw_gain: float = 0.8
    yaw_rate_limit: float = 15.0
    brake_delay: float = 0.25

    def validate(self):
        if not all(math.isfinite(value) and value > 0.0
                   for value in self.__dict__.values()):
            raise ValueError('BLINE parameters must be finite and positive')
        if not 0.0 < self.default_speed < 0.18:
            raise ValueError('BLINE default speed must be strictly below 0.18m/s')
        if self.max_speed < 0.18 or self.capture_exit >= self.capture_enter:
            raise ValueError('invalid BLINE total speed or capture hysteresis')
        if self.brake_acceleration > min(self.deceleration,
                                         self.vector_acceleration):
            raise ValueError('BLINE brake acceleration exceeds command limits')


def validate_goal(target, axes, cruise_speed, timeout, config):
    if len(target) != 4 or not all(math.isfinite(float(v)) for v in target):
        raise ValueError('BLINE target must be four finite values [dx,dy,dz,0]')
    if axes not in ('', 'xyz') or float(target[3]) != 0.0:
        raise ValueError('BLINE requires axes="" or "xyz" and target[3]=0')
    if norm(target[:3]) <= 1e-6:
        raise ValueError('BLINE requires a nonzero displacement')
    speed = float(cruise_speed)
    if not math.isfinite(speed) or speed < 0.0 or speed >= 0.18:
        raise ValueError('BLINE cruise_speed must be 0 (default) or 0<speed<0.18')
    if not math.isfinite(float(timeout)):
        raise ValueError('BLINE timeout must be finite')
    return config.default_speed if speed == 0.0 else speed


@dataclass(frozen=True)
class LineSample:
    progress: float
    cross_error: float
    remaining: float
    measured_speed: float
    endpoint_error: tuple


class LineGuidance:
    def __init__(self, position, yaw_deg, displacement, speed, config):
        self.config = config
        self.speed = speed
        self.start = tuple(position)
        # Keep the existing 4DOF displacement convention; freeze before ALIGN.
        offset = rotate_body_to_world(displacement, 0.0, 0.0, yaw_deg)
        self.end = tuple(a + b for a, b in zip(self.start, offset))
        self.length = norm(offset)
        self.direction = tuple(value / self.length for value in offset)
        self.yaw = (math.degrees(math.atan2(offset[1], offset[0]))
                    if math.hypot(offset[0], offset[1]) > 1e-6 else yaw_deg)
        self.yaw = (self.yaw + 180.0) % 360.0 - 180.0
        self.velocity = (0.0, 0.0, 0.0)
        self.capture = False
        self.terminal = False

    def sample(self, position, world_velocity):
        offset = tuple(p - a for p, a in zip(position, self.start))
        progress = dot(offset, self.direction)
        perpendicular = tuple(p - progress * d
                              for p, d in zip(offset, self.direction))
        endpoint_error = tuple(b - p for b, p in zip(self.end, position))
        return LineSample(progress, norm(perpendicular), self.length - progress,
                          dot(world_velocity, self.direction), endpoint_error)

    def step(self, position, world_velocity, dt):
        cfg = self.config
        sample = self.sample(position, world_velocity)
        if sample.cross_error > cfg.capture_enter:
            self.capture = True
        elif sample.cross_error <= cfg.capture_exit:
            self.capture = False
        if (sample.remaining <= 0.0
                or norm(sample.endpoint_error) <= cfg.terminal_radius):
            self.terminal = True
        current_along = dot(self.velocity, self.direction)
        stopping_speed = max(0.0, sample.measured_speed, current_along)
        brake_distance = (stopping_speed ** 2 / (2.0 * cfg.brake_acceleration)
                          + stopping_speed * cfg.brake_delay)
        if self.terminal:
            # A proportional endpoint vector has the same along/cross split.
            along_star = cfg.cross_gain * sample.remaining
            along_star = max(-min(cfg.reverse_speed, self.speed),
                             min(self.speed, along_star))
            phase = 'TERMINAL'
        else:
            available = max(0.0, sample.remaining
                            - stopping_speed * cfg.brake_delay)
            along_star = min(self.speed,
                             math.sqrt(2.0 * cfg.brake_acceleration * available))
            if self.capture:
                along_star = min(along_star, cfg.capture_speed)
                phase = 'CAPTURE'
            else:
                phase = ('BRAKE' if sample.remaining <= max(
                    brake_distance, self.speed ** 2 / (2.0 * cfg.brake_acceleration))
                         else 'CRUISE')
        change = along_star - current_along
        along = current_along + max(-cfg.deceleration * dt,
                                    min(cfg.acceleration * dt, change))
        offset = tuple(p - a for p, a in zip(position, self.start))
        perpendicular = tuple(p - sample.progress * d
                              for p, d in zip(offset, self.direction))
        cross_limit = min(cfg.cross_speed,
                          math.sqrt(max(0.0, cfg.max_speed ** 2 - along ** 2)))
        cross = saturate(tuple(-cfg.cross_gain * p for p in perpendicular),
                         cross_limit)
        desired = tuple(along * d + c for d, c in zip(self.direction, cross))
        desired = saturate(desired, cfg.terminal_speed if self.terminal
                           else cfg.max_speed)
        delta = saturate(tuple(new - old for new, old in zip(desired, self.velocity)),
                         cfg.vector_acceleration * dt)
        self.velocity = tuple(old + dv for old, dv in zip(self.velocity, delta))
        return self.velocity, phase, sample

    def yaw_rate(self, yaw_deg):
        error = (self.yaw - yaw_deg + 180.0) % 360.0 - 180.0
        return max(-self.config.yaw_rate_limit,
                   min(self.config.yaw_rate_limit, self.config.yaw_gain * error))

    def reached(self, position, world_velocity, yaw_deg):
        yaw_error = abs((self.yaw - yaw_deg + 180.0) % 360.0 - 180.0)
        return (all(abs(b - p) <= self.config.position_tolerance
                    for b, p in zip(self.end, position))
                and norm(world_velocity) <= self.config.stop_speed
                and yaw_error <= self.config.yaw_tolerance)
