"""Gate task defaults, flat schema and shared semantic validation."""
import math
from copy import deepcopy

GROUPS = {
    'observation': dict(poses=[[1.5, -2.0, 0.2, 0.0]], xy_radius_m=[0.6]*5,
                        travel_timeout=60.0, set_timeout=20.0),
    'fixed_pass': dict(code='0000'),
    # Legacy tracking/association keys remain accepted for existing mission overrides.
    # The gate task no longer uses them for selection, alignment or loss handling.
    'tracking': dict(confirm_frames=3, max_center_distance_fraction=0.12,
                     max_height_ratio=1.5, ambiguity_margin=0.15),
    'depth': dict(front=[0.1]*4, set_timeout=10.0, kp=0.8,
                  max_speed_mps=0.12, tolerance_m=0.03),
    'search': dict(observe_seconds=2.0, scan_angle_deg=90.0, start_offset_deg=-30.0,
                   sweep_degrees=[60.0, 120.0, 180.0], yaw_rate_deg_s=10.0,
                   timeout=60.0, detection_timeout=1.5, min_confidence=0.02,
                   min_bbox_pixels=12.0, lock_center_delta_fraction=0.35,
                   lock_bearing_tolerance_deg=20.0, priority_release_seconds=2.0,
                   reacquire_timeout=10.0, align_observe_seconds=1.0,
                   align_timeout=20.0, yaw_kp=1.2, max_yaw_rate_deg_s=12.0,
                   position_hold_kp=0.8,
                   position_hold_max_speed_mps=0.05,
                   position_hold_tolerance_m=0.01,
                   velocity_period=0.05, light_pulse_seconds=0.3,
                   light_gap_seconds=0.3),
    'lateral': dict(front=[0.0]*4, speed_mps=0.05, tolerance_m=0.02,
                    timeout=40.0, path_kp=0.5, max_path_speed_mps=0.03),
    'fore_aft': dict(height_limit_percent=80.0, speed_mps=0.15,
                     max_distance_m=0.30,
                     position_tolerance_m=0.01, timeout=30.0,
                     observe_seconds=1.0),
    'pass': dict(yaw_servo_min_seconds=1.0, yaw_servo_max_seconds=3.0,
                 yaw_tolerance_deg=2.0, yaw_stable_seconds=0.3, yaw_kp=1.2,
                 max_yaw_rate_deg_s=12.0, stereo_pair_slop=0.25,
                 stereo_max_ray_gap_m=0.15, stereo_min_angle_deg=0.1,
                 stereo_max_distance_m=30.0, distance_m=1.0,
                 speed_mps=0.15, timeout=60.0, pause_seconds=0.5,
                 next_observe_seconds=2.0),
}
DEFAULTS = {'gate_count': 1, 'timeout': 720.0, 'max_failures': 1}
ALIASES = {}
for group, values in GROUPS.items():
    for key, value in values.items():
        flat = f'{group}_{key}'
        DEFAULTS[flat] = value
        ALIASES[f'{group}.{key}'] = flat
SCHEMA = {key: ((list, float) if isinstance(value, list) else type(value))
          for key, value in DEFAULTS.items()}

SCHEMA['observation_poses'] = (list, (list, float))


def validate_gate_params(params, complete=False):
    """Validate also at runtime, so direct task calls obey the YAML contract."""
    unknown = set(params) - set(DEFAULTS)
    if unknown:
        raise ValueError(f'未知过门参数: {sorted(unknown)}')
    values = deepcopy({**DEFAULTS, **params})

    def finite_values(value):
        if isinstance(value, list):
            return all(finite_values(item) for item in value)
        return (not isinstance(value, bool) and isinstance(value, (int, float))
                and math.isfinite(value))

    for key, value in values.items():
        expected = SCHEMA[key]
        if expected is str:
            if not isinstance(value, str):
                raise ValueError(f'{key} 必须为字符串')
            continue
        if isinstance(expected, tuple) and not isinstance(value, list):
            raise ValueError(f'{key} 必须是数值数组')
        if not finite_values(value):
            raise ValueError(f'{key} 必须是有限数值')
        if expected is int and not isinstance(value, int):
            raise ValueError(f'{key} 必须为整数')
    if not 1 <= values['gate_count'] <= 4:
        raise ValueError('gate_count 必须为 1–4')
    code = values['fixed_pass_code']
    if len(code) != 4 or any(bit not in '01' for bit in code):
        raise ValueError('fixed_pass.code 必须为四位二进制字符串，例如 "0000" 或 "1010"')
    poses = values['observation_poses']
    # Partial mission overrides are validated before merging. Only compare an
    # explicitly supplied pair here; runtime validates the merged contract.
    # Extra poses may remain configured; gate_count selects the active prefix.
    if ((complete or ('gate_count' in params and 'observation_poses' in params))
            and len(poses) < values['gate_count']):
        raise ValueError('observation.poses 数量不能少于 gate_count')
    if (not poses or len(poses) > 5
            or any(not isinstance(pose, list) or len(pose) != 4
                   or any(not isinstance(value, (int, float)) for value in pose) for pose in poses)):
        raise ValueError('observation.poses 必须为 1–5 条数值 [x,y,z,yaw]')
    radii = values['observation_xy_radius_m']
    if (not 1 <= len(radii) <= 5
            or any(not isinstance(radius, (int, float)) or radius <= 0 for radius in radii)):
        raise ValueError('observation.xy_radius_m 必须包含 1–5 个有限正数')
    if ((complete or {'observation_poses', 'observation_xy_radius_m'} <= set(params))
            and len(radii) < len(poses)):
        raise ValueError('observation.xy_radius_m 必须覆盖每条预设位姿')
    if complete or {'gate_count', 'fixed_pass_code', 'observation_poses'} <= set(params):
        for index, bit in enumerate(code[:values['gate_count']]):
            if bit == '1' and index+1 >= len(poses):
                raise ValueError(f'第 {index+1} 门定点通过缺少第 {index+2} 条终点位姿；'
                                 '第 4 门定点通过需配置第 5 条出口位姿')
    if any(pose[2] < 0 for pose in poses):
        raise ValueError('observation.poses 水深不能为负')
    if 'depth_front' in params and (complete or 'observation_poses' in params) and len(values['depth_front']) == 4:
        if any(abs(pose[2]-values['depth_front'][i]) > 1e-9
               for i, pose in enumerate(poses[:4])):
            raise ValueError('depth.front 与 observation.poses 的 z 冲突')
    for key in ('depth_front', 'lateral_front'):
        if len(values[key]) != 4:
            raise ValueError(f'{key} 必须有四位数值')
    if any(x < 0 for x in values['depth_front']):
        raise ValueError('depth.front 水深不能为负')
    nonnegative = {'lateral_front', 'depth_front', 'search_start_offset_deg', 'max_failures'}
    if values['max_failures'] < 0:
        raise ValueError('max_failures 必须为非负整数')
    for key, value in values.items():
        if key in nonnegative or isinstance(value, (list, str)):
            continue
        if value <= 0:
            raise ValueError(f'{key} 必须大于零')
    sweeps = values['search_sweep_degrees']
    if not sweeps or any(x <= 0 or x > 360 for x in sweeps):
        raise ValueError('search.sweep_degrees 必须包含 (0,360] 度的扫描范围')
    if not 0 < values['search_min_confidence'] <= 1:
        raise ValueError('search.min_confidence 必须在 (0,1] 内')
    if not 0 < values['search_scan_angle_deg'] <= 180:
        raise ValueError('search.scan_angle_deg 必须在 (0,180] 内')
    if not 0 < values['tracking_max_center_distance_fraction'] <= 1:
        raise ValueError('tracking.max_center_distance_fraction 必须在 (0,1] 内')
    if values['tracking_max_height_ratio'] <= 1:
        raise ValueError('tracking.max_height_ratio 必须大于 1')
    if not 0 < values['tracking_ambiguity_margin'] < 1:
        raise ValueError('tracking.ambiguity_margin 必须在 (0,1) 内')
    if not 0 < values['fore_aft_height_limit_percent'] <= 100:
        raise ValueError('fore_aft.height_limit_percent 必须在 (0,100] 内')
    if values['fore_aft_max_distance_m'] <= values['fore_aft_position_tolerance_m']:
        raise ValueError('fore_aft.max_distance_m 必须大于 BLINE 到点位置容差')
    if values['fore_aft_speed_mps'] >= 0.80:
        raise ValueError('fore_aft.speed_mps 必须小于 BLINE 速度上限 0.80m/s')
    if values['pass_yaw_servo_min_seconds'] > values['pass_yaw_servo_max_seconds']:
        raise ValueError('最终 yaw 最短时限不能大于最长时限')
    if not 0 < values['search_lock_center_delta_fraction'] <= 1:
        raise ValueError('search.lock_center_delta_fraction 必须在 (0,1] 内')
    return values


def gate_uses_vision(params):
    """Only active visual stages need the gate task's camera dependencies."""
    values = validate_gate_params(params, complete=True)
    return '0' in values['fixed_pass_code'][:values['gate_count']]
