"""Target-rack approach settings before the existing drop-ball procedure."""
import math
from uv_task.golf_search_config import SEARCH_DEFAULTS, SEARCH_SCHEMA, validate_search_params

DROP_SEARCH_DEFAULTS = dict(SEARCH_DEFAULTS)
DROP_SEARCH_DEFAULTS.pop('collection_frame_position')
DROP_SEARCH_DEFAULTS.update(rack_position=[5.0, 0.0, -1.0],
                            search_timeout=20.0, search_near_radius_m=0.3)
DROP_SEARCH_SCHEMA = {
    key: value for key, value in SEARCH_SCHEMA.items()
    if key not in {'collection_frame_position', 'search_cruise_depth_m',
                   'search_depth_gain', 'search_max_vertical_speed_mps'}
}
DROP_SEARCH_SCHEMA.update(rack_position=(list, float),
                          depth_task_depth_m=float, depth_timeout=float)


def validate_drop_search_params(params):
    depth = params.get('depth_task_depth_m', 0.2)
    timeout = params.get('depth_timeout', 30.0)
    for key, value, allow_zero in (('depth.task_depth_m', depth, True),
                                    ('depth.timeout', timeout, False)):
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value < 0
                or (value == 0 and not allow_zero)):
            raise ValueError(f'{key} 必须为有限数值；深度非负，超时为正')
    values = validate_search_params(
        {**params, 'search_cruise_depth_m': float(depth)},
        defaults=DROP_SEARCH_DEFAULTS, position_key='rack_position')
    values['depth_timeout'] = float(timeout)
    return values
