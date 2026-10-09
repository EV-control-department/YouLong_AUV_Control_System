"""Gate task defaults, flat schema and shared semantic validation."""
import math

GROUPS = {
    'depth': dict(front=[0.1]*4, set_timeout=10.0, kp=0.8,
                  max_speed_mps=0.12, tolerance_m=0.03),
    'search': dict(observe_seconds=2.0, start_offset_deg=-30.0,
                   sweep_degrees=[60.0, 120.0, 180.0], yaw_rate_deg_s=10.0,
                   timeout=60.0, detection_timeout=1.5, min_confidence=0.02,
                   min_bbox_pixels=12.0, lock_center_delta_fraction=0.35,
                   lock_bearing_tolerance_deg=20.0, priority_release_seconds=2.0,
                   reacquire_timeout=10.0, align_observe_seconds=1.0,
                   align_timeout=20.0, yaw_kp=1.2, max_yaw_rate_deg_s=12.0,
                   velocity_period=0.05, light_pulse_seconds=0.3,
                   light_gap_seconds=0.3),
    'lateral': dict(front=[0.0]*4, speed_mps=0.05, tolerance_m=0.02,
                    timeout=40.0, path_kp=0.5, max_path_speed_mps=0.03),
    'fore_aft': dict(target_area_percent=80.0, area_tolerance_percent=1.0,
                     stable_seconds=0.5, speed_mps=0.10, reverse_speed_mps=0.08,
                     area_kp=0.4, line_kp=0.5, max_cross_speed_mps=0.03,
                     line_tolerance_m=0.03, max_travel_m=4.0, timeout=90.0,
                     observe_seconds=1.0),
    'pass': dict(yaw_servo_min_seconds=1.0, yaw_servo_max_seconds=3.0,
                 yaw_tolerance_deg=2.0, yaw_stable_seconds=0.3, yaw_kp=1.2,
                 max_yaw_rate_deg_s=12.0, stereo_pair_slop=0.25,
                 stereo_max_ray_gap_m=0.15, stereo_min_angle_deg=0.1,
                 stereo_max_distance_m=30.0, distance_m=1.6,
                 speed_mps=0.15, timeout=60.0, pause_seconds=0.5),
}
DEFAULTS = {'gate_count': 4, 'timeout': 720.0}
ALIASES = {}
for group, values in GROUPS.items():
    for key, value in values.items():
        flat = f'{group}_{key}'
        DEFAULTS[flat] = value
        ALIASES[f'{group}.{key}'] = flat
SCHEMA = {key: ((list, float) if isinstance(value, list) else type(value))
          for key, value in DEFAULTS.items()}


def validate_gate_params(params):
    """Validate also at runtime, so direct task calls obey the YAML contract."""
    unknown = set(params) - set(DEFAULTS)
    if unknown:
        raise ValueError(f'未知过门参数: {sorted(unknown)}')
    values = {key: value.copy() if isinstance(value, list) else value
              for key, value in {**DEFAULTS, **params}.items()}
    for key, value in values.items():
        expected = SCHEMA[key]
        items = value if isinstance(expected, tuple) else [value]
        if isinstance(expected, tuple) and not isinstance(value, list):
            raise ValueError(f'{key} 必须是数值数组')
        for item in items:
            if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item):
                raise ValueError(f'{key} 必须是有限数值')
        if expected is int and not isinstance(value, int):
            raise ValueError(f'{key} 必须为整数')
    if not 1 <= values['gate_count'] <= 4:
        raise ValueError('gate_count 必须为 1–4')
    for key in ('depth_front', 'lateral_front'):
        if len(values[key]) != 4:
            raise ValueError(f'{key} 必须有四位数值')
    if any(x < 0 for x in values['depth_front']):
        raise ValueError('depth.front 水深不能为负')
    nonnegative = {'lateral_front', 'depth_front', 'search_start_offset_deg'}
    for key, value in values.items():
        if key in nonnegative or isinstance(value, list):
            continue
        if value <= 0:
            raise ValueError(f'{key} 必须大于零')
    sweeps = values['search_sweep_degrees']
    if not sweeps or any(x <= 0 or x > 360 for x in sweeps):
        raise ValueError('search.sweep_degrees 必须包含 (0,360] 度的扫描范围')
    if not 0 < values['search_min_confidence'] <= 1:
        raise ValueError('search.min_confidence 必须在 (0,1] 内')
    if not 0 < values['fore_aft_target_area_percent'] <= 100:
        raise ValueError('fore_aft.target_area_percent 必须在 (0,100] 内')
    if values['fore_aft_area_tolerance_percent'] >= values['fore_aft_target_area_percent']:
        raise ValueError('面积容限必须小于目标面积百分比')
    if values['pass_yaw_servo_min_seconds'] > values['pass_yaw_servo_max_seconds']:
        raise ValueError('最终 yaw 最短时限不能大于最长时限')
    if not 0 < values['pass_speed_mps'] < 0.18:
        raise ValueError('pass.speed_mps 必须在 (0,0.18) 内，与 BLINE 接口一致')
    if not 0 < values['search_lock_center_delta_fraction'] <= 1:
        raise ValueError('search.lock_center_delta_fraction 必须在 (0,1] 内')
    return values
