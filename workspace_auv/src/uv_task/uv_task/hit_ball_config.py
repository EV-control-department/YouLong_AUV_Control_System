"""ROS-independent parameters for the preset-position impact-ball workflow."""
import math

BALL_NAMES = {'impact_ball_blue', 'impact_ball_red'}
DEFAULTS = {
    'order': ['impact_ball_blue'],
    'targets_blue_position': [2.0, 0.0, 0.2],
    'targets_red_position': [2.0, 0.0, 0.2],
    'depth_task_depth_m': 0.2,
    'depth_timeout': 15.0,
    'search_initial_observe_seconds': 2.0,
    'search_align_seconds': 3.0,
    'search_detection_timeout': 5.0,
    'search_min_confidence': 0.02,
    'search_yaw_tolerance_deg': 2.0,
    'search_yaw_stable_seconds': 0.2,
    'search_yaw_gain': 1.2,
    'search_max_yaw_rate_deg_s': 10.0,
    'search_rotate_timeout': 8.0,
    'search_period': 0.05,
    'search_stereo_pair_slop': 0.15,
    'search_stereo_min_angle_deg': 0.1,
    'search_stereo_max_ray_gap_m': 0.05,
    'search_stereo_max_distance_m': 20.0,
    'light_pulse_seconds': 0.35,
    'light_gap_seconds': 0.25,
    'cruise_radius_m': 1.5,
    'cruise_speed_mps': 0.24,
    'cruise_timeout': 20.0,
    'observe_timeout': 5.0,
    'observe_yaw_step_deg': 15.0,
    'observe_direction_dwell_seconds': 5.0,
    'observe_rotate_timeout': 5.0,
    'approach_distance_m': 0.5,
    'approach_speed_mps': 0.18,
    'approach_timeout': 15.0,
    'charge_alignment_seconds': 2.0,
    'charge_duration': 8.0,
    'charge_speed_mps': 0.24,
    'charge_timeout': 20.0,
    'fallback_duration': 15.0,
    'fallback_speed_mps': 0.15,
    'fallback_timeout': 25.0,
    'motion_cancel_timeout': 2.0,
    'motion_start_timeout': 30.0,
    'motion_accept_timeout': 5.0,
    'between_balls_pause': 0.5,
}
SCHEMA = {
    key: (list, str) if key == 'order' else (list, float)
    if key.startswith('targets_') else float for key in DEFAULTS
}


def validate_hit_params(params):
    p = {key: params.get(key, value) for key, value in DEFAULTS.items()}
    if (not isinstance(p['order'], list) or not p['order']
            or any(not isinstance(name, str) or name not in BALL_NAMES for name in p['order'])
            or len(p['order']) != len(set(p['order']))):
        raise ValueError('撞球 order 必须为不重复的 impact_ball_blue / impact_ball_red 列表')
    p['order'] = tuple(p['order'])
    for key in ('targets_blue_position', 'targets_red_position'):
        value = p[key]
        if not isinstance(value, (list, tuple)) or len(value) not in (0, 3):
            raise ValueError(f'{key} 必须为 [x,y,z]；未配置时使用空列表 []')
        if any(isinstance(x, bool) or not isinstance(x, (int, float))
               or not math.isfinite(x) for x in value):
            raise ValueError(f'{key} 坐标必须为有限数值，单位米')
        p[key] = tuple(float(x) for x in value)
    nonnegative = {'depth_task_depth_m', 'search_min_confidence',
                   'search_yaw_stable_seconds', 'light_gap_seconds',
                   'between_balls_pause'}
    for key, value in p.items():
        if key == 'order' or key.startswith('targets_'):
            continue
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value < 0
                or (value == 0 and key not in nonnegative)):
            raise ValueError(f'{key} 必须为有限正数（深度、置信度、保持和间隔可为零）')
        p[key] = float(value)
    for key in ('cruise_speed_mps', 'approach_speed_mps',
                'charge_speed_mps', 'fallback_speed_mps'):
        if not 0 < p[key] < 0.28:
            raise ValueError(f'{key} 必须满足 BLINE 接口要求 0 < speed < 0.28')
    if p['search_min_confidence'] > 1:
        raise ValueError('search.min_confidence 必须在 [0,1] 内')
    if p['observe_yaw_step_deg'] > 180:
        raise ValueError('observe.yaw_step_deg 不能超过 180°')
    if p['search_yaw_stable_seconds'] > min(p['search_align_seconds'], p['charge_alignment_seconds']):
        raise ValueError('yaw_stable_seconds 不能大于任一 yaw 对准阶段时长')
    for phase in ('charge', 'fallback'):
        if p[f'{phase}_timeout'] <= p[f'{phase}_duration']:
            raise ValueError(f'{phase}.timeout 必须大于 duration，留出 BLINE 起步时间')
    return p
