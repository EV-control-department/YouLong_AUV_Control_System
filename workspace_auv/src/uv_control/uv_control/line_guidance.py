"""Finite-segment guidance in odom/NED, independent of ROS and actuators."""

from dataclasses import dataclass
import math


# Leave a small margin so float32 transport rounding stays strictly below the
# unified 0.80m/s LINE speed ceiling.
CRUISE_SPEED_LIMIT = 0.80 - 1e-6
TOTAL_SPEED_LIMIT = 0.80 - 1e-6


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
    default_speed: float = 0.30
    max_speed: float = 0.80
    acceleration: float = 0.30
    deceleration: float = 0.30
    vector_acceleration: float = 0.40
    brake_acceleration: float = 0.20
    cross_gain: float = 0.60
    cross_speed: float = 0.18
    z_gain: float = 1.20
    z_max_speed: float = 0.20
    capture_enter: float = 0.30
    capture_exit: float = 0.10
    capture_speed: float = 0.15
    reverse_speed: float = 0.08
    terminal_speed: float = 0.10
    terminal_radius: float = 0.35
    position_tolerance: float = 0.30
    z_tolerance: float = 0.15
    stop_speed: float = 0.03
    yaw_tolerance: float = 3.0
    stable_seconds: float = 1.2
    align_timeout: float = 30.0
    yaw_gain: float = 0.8
    yaw_rate_limit: float = 15.0
    brake_delay: float = 0.30

    def validate(self):
        if not all(math.isfinite(value) and value > 0.0
                   for value in self.__dict__.values()):
            raise ValueError('BLINE parameters must be finite and positive')
        if self.capture_exit >= self.capture_enter:
            raise ValueError('invalid BLINE capture hysteresis')


def validate_goal(target, axes, cruise_speed, timeout, config, world=False):
    if len(target) != 4 or not all(math.isfinite(float(v)) for v in target):
        raise ValueError('BLINE target must be four finite values [x,y,z,yaw_deg]')
    if axes not in ('', 'xyz'):
        raise ValueError('LINE requires axes="" or "xyz"')
    if not world and norm(target[:3]) <= 1e-6 and abs(float(target[3])) <= 1e-6:
        raise ValueError('BLINE requires a nonzero displacement or final rotation')
    speed = float(cruise_speed)
    if not math.isfinite(speed) or speed < 0.0:
        raise ValueError('BLINE cruise_speed must be finite and nonnegative')
    if not math.isfinite(float(timeout)):
        raise ValueError('BLINE timeout must be finite')
    requested = config.default_speed if speed == 0.0 else speed
    return min(requested, CRUISE_SPEED_LIMIT, config.max_speed, TOTAL_SPEED_LIMIT)


@dataclass(frozen=True)
class LineSample:
    progress: float
    cross_error: float
    remaining: float
    measured_speed: float
    endpoint_error: tuple


class LineGuidance:
    def __init__(self, position, yaw_deg, displacement, speed, config,
                 world=False, final_yaw=None):
        self.config = config
        self.speed = min(speed, CRUISE_SPEED_LIMIT, config.max_speed, TOTAL_SPEED_LIMIT)
        self.start = tuple(position)
        # Keep the existing 4DOF displacement convention; freeze before ALIGN.
        offset = (tuple(float(p) - a for p, a in zip(displacement, self.start))
                  if world else rotate_body_to_world(displacement, 0.0, 0.0, yaw_deg))
        self.end = tuple(a + b for a, b in zip(self.start, offset))
        self.length = norm(offset)
        self.direction = (tuple(value / self.length for value in offset)
                          if self.length > 1e-6 else (1.0, 0.0, 0.0))
        self.yaw = (math.degrees(math.atan2(offset[1], offset[0]))
                    if math.hypot(offset[0], offset[1]) > 1e-6 else yaw_deg)
        self.yaw = (self.yaw + 180.0) % 360.0 - 180.0
        self.final_yaw = (self.yaw if final_yaw is None else
                          (float(final_yaw) + 180.0) % 360.0 - 180.0)
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
        max_speed = min(cfg.max_speed, TOTAL_SPEED_LIMIT)
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
        offset = tuple(p - a for p, a in zip(position, self.start))
        perpendicular = tuple(p - sample.progress * d
                              for p, d in zip(offset, self.direction))
        target_limit = min(cfg.terminal_speed, max_speed) if self.terminal else max_speed
        # Follow the segment depth with along-track feed-forward; near the end
        # servo directly to endpoint Z. The XY terminal cap must not weaken Z.
        vz = (cfg.z_gain * sample.endpoint_error[2] if self.terminal else
              along_star * self.direction[2] - cfg.z_gain * perpendicular[2])
        z_limit = min(cfg.z_max_speed, max_speed)
        vz = max(-z_limit, min(z_limit, vz))
        horizontal_limit = min(target_limit,
                               math.sqrt(max(0.0, max_speed ** 2 - vz ** 2)))
        # XY correction and terminal speed leave a separate budget for depth.
        cross_xy = saturate(tuple(-cfg.cross_gain * p for p in perpendicular[:2]),
                            min(cfg.cross_speed, horizontal_limit))
        cross_star = (*cross_xy, 0.0)
        # Reserve the requested horizontal correction before along-track speed.
        # Otherwise cruising at max_speed leaves no authority to recover drift.
        along_limit = math.sqrt(max(0.0, horizontal_limit ** 2 - dot(cross_star, cross_star)))
        along_star = max(-along_limit, min(along_limit, along_star))
        change = along_star - current_along
        along = current_along + max(-cfg.deceleration * dt,
                                    min(cfg.acceleration * dt, change))
        # As along-track speed ramps down, progressively release its budget to
        # correction. Keep the hard total limit throughout the transition.
        cross_limit = math.sqrt(max(0.0, max_speed ** 2 - along ** 2))
        cross = saturate(cross_star, cross_limit)
        desired = tuple(along * d + c for d, c in zip(self.direction, cross))
        # Reach the horizontal target under the existing deceleration limit;
        # reserve only the hard total-speed budget during that transition.
        horizontal = saturate(desired[:2], math.sqrt(max(0.0, max_speed ** 2 - vz ** 2)))
        desired = (*horizontal, vz)
        # Terminal speed is a target reached under the same slew limits; an
        # immediate terminal clamp would bypass along-track deceleration.
        desired = saturate(desired, max_speed)
        delta = saturate(tuple(new - old for new, old in zip(desired, self.velocity)),
                         cfg.vector_acceleration * dt)
        self.velocity = tuple(old + dv for old, dv in zip(self.velocity, delta))
        return self.velocity, phase, sample

    def yaw_rate(self, yaw_deg):
        error = (self.yaw - yaw_deg + 180.0) % 360.0 - 180.0
        return max(-self.config.yaw_rate_limit,
                   min(self.config.yaw_rate_limit, self.config.yaw_gain * error))

    def reached(self, position, world_velocity, yaw_deg, final=False,
                position_tolerance=None):
        target_yaw = self.final_yaw if final else self.yaw
        yaw_error = abs((target_yaw - yaw_deg + 180.0) % 360.0 - 180.0)
        return (self.position_reached(position, position_tolerance=position_tolerance)
                and norm(world_velocity) <= self.config.stop_speed
                and yaw_error <= self.config.yaw_tolerance)

    def position_reached(self, position, *, target=None, position_tolerance=None):
        """Use an independent depth tolerance for alignment and arrival.

        The per-goal position override controls XY; Z uses ``z_tolerance``.
        """
        target = self.end if target is None else target
        tolerance = (self.config.position_tolerance if position_tolerance is None
                     else float(position_tolerance))
        return (all(abs(b - p) <= tolerance
                    for b, p in zip(target[:2], position[:2]))
                and abs(target[2] - position[2]) <= self.config.z_tolerance)
