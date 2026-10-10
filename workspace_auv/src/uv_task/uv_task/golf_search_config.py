"""Validated settings for the collection-frame approach before golf pickup."""
import math

SEARCH_DEFAULTS = {
    'collection_frame_position': [5.8, -0.3, 0.5],
    'search_cruise_depth_m': 0.2,
    'depth_timeout': 15.0,
    'search_timeout': 40.0,
    'search_near_radius_m': 0.5,
    'search_near_timeout': 10.0,
    'search_front_observe_seconds': 2.0,
    'search_front_align_seconds': 3.0,
    'search_yaw_tolerance_deg': 2.0,
    'search_yaw_stable_seconds': 0.2,
    'search_yaw_gain': 1.2,
    'search_max_yaw_rate_deg_s': 10.0,
    'search_depth_gain': 0.8,
    'search_max_vertical_speed_mps': 0.08,
    'search_speed_mps': 0.15,
    'search_detection_timeout': 0.8,
    'search_min_confidence': 0.02,
    'search_detection_stop_delay': 0.5,
    'search_fallback_observe_seconds': 2.0,
    'search_fallback_rotate_speed_deg_s': 30.0,
    'search_fallback_rotate_degrees': 360.0,
    'search_fallback_rotate_timeout': 15.0,
    'search_move_timeout': 60.0,
    'search_period': 0.05,
    'search_stereo_pair_slop': 0.15,
    'search_stereo_min_angle_deg': 0.1,
    'search_stereo_max_ray_gap_m': 0.05,
    'search_stereo_max_distance_m': 20.0,
    'search_cancel_timeout': 2.0,
}
SEARCH_SCHEMA = {key: (list, float) if isinstance(value, list) else float
                 for key, value in SEARCH_DEFAULTS.items()}


def validate_search_params(params, *, defaults=None, position_key='collection_frame_position'):
    defaults = SEARCH_DEFAULTS if defaults is None else defaults
    values = {key: params.get(key, default) for key, default in defaults.items()}
    position = values[position_key]
    if (not isinstance(position, (list, tuple)) or len(position) != 3
            or any(isinstance(x, bool) or not isinstance(x, (int, float))
                   or not math.isfinite(x) for x in position)):
        raise ValueError(f'{position_key} 必须为有限数值 [x,y,z]，单位米')
    values[position_key] = tuple(float(x) for x in position)
    nonnegative = {'search_cruise_depth_m', 'search_min_confidence',
                   'search_yaw_stable_seconds', 'search_detection_stop_delay'}
    for key, value in values.items():
        if key == position_key:
            continue
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0 or (key not in nonnegative and value == 0)):
            raise ValueError(f'{key} 必须为有限的正数（深度、置信度、保持和延时可为零）')
        values[key] = float(value)
    if not 0 < values['search_speed_mps'] < 0.18:
        raise ValueError('search.speed_mps 必须满足 0 < speed < 0.18（BLINE 接口）')
    if values['search_min_confidence'] > 1:
        raise ValueError('search.min_confidence 必须在 [0,1] 内')
    if values['search_yaw_stable_seconds'] > values['search_front_align_seconds']:
        raise ValueError('search.yaw_stable_seconds 不能超过 front_align_seconds')
    return values
