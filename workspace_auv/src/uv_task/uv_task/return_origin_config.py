"""Configuration for the final horizontal and upward open-loop thrust pulses."""
import math


DEFAULTS = {
    'horizontal_yaw_deg': -135.0,  # odom -X/-Y, independent of body heading.
    'horizontal_thrust_fraction': 1.0,
    'horizontal_duration_seconds': 3.0,
    'ascent_thrust_fraction': 1.0,
    'ascent_duration_seconds': 10.0,
}
SCHEMA = {key: float for key in DEFAULTS}
ALIASES = {
    'horizontal.yaw_deg': 'horizontal_yaw_deg',
    'horizontal.thrust_fraction': 'horizontal_thrust_fraction',
    'horizontal.duration_seconds': 'horizontal_duration_seconds',
    'ascent.thrust_fraction': 'ascent_thrust_fraction',
    'ascent.duration_seconds': 'ascent_duration_seconds',
}


def validate_return_origin_params(params):
    values = {**DEFAULTS, **params}
    for key in DEFAULTS:
        value = values[key]
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value)):
            raise ValueError(f'return_origin.{key} 必须是有限数值')
        values[key] = float(value)
    for key in ('horizontal_duration_seconds', 'ascent_duration_seconds'):
        if values[key] <= 0.0:
            raise ValueError(f'return_origin.{key} 必须大于0秒')
    for key in ('horizontal_thrust_fraction', 'ascent_thrust_fraction'):
        if not 0.0 < values[key] <= 1.0:
            raise ValueError(f'return_origin.{key} 必须在(0,1]范围内')
    return values
