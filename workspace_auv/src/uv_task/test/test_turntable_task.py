"""转盘几何与禁动参数测试；不连接推进器或仿真。"""

from pathlib import Path
import json
import threading
from types import SimpleNamespace

import pytest
from std_msgs.msg import String

from uv_task.config_loader import load_task
from uv_task.turntable_task import (
    DISK_DIAMETER_M, STROKE_COUNT, TurntableTask, _hole_world, _robot_for_tip, _tip_world,
)


def test_rod_tip_inverse_kinematics():
    tip_body = (0.42, -0.08, 0.02)
    desired_tip = (2.1, 3.2, 0.7)
    robot = _robot_for_tip(desired_tip, 35.0, tip_body)
    actual_tip = _tip_world(robot, 35.0, tip_body)
    assert actual_tip == pytest.approx(desired_tip)


def test_vertical_hole():
    # z 向下；90° 孔位应在盘心上方。
    assert _hole_world((1, 2, 1), 0, 0.2, 90, -0.1) == pytest.approx(
        (0.9, 2.0, 0.8))


def test_unfilled_geometry_is_rejected():
    task = object.__new__(TurntableTask)
    task.params = {'allow_contact_motion': False}
    with pytest.raises(ValueError, match='尚未标定'):
        task._calibration()


def test_turntable_template_loads_new_disk_dimensions_without_enabling_contact():
    config = Path(__file__).resolve().parents[1] / 'config/tasks/turntable.yaml'
    params = load_task(config)[0]['params']
    assert DISK_DIAMETER_M == pytest.approx(0.230)
    assert STROKE_COUNT == 3
    assert params['stroke_yaw_deg'] == pytest.approx(6.0)
    assert params['allow_contact_motion'] is False
    assert params['force_limited_control_confirmed'] is False
    assert 'min_progress_deg' not in params


def test_calibrated_but_contact_disabled_never_commands_motion():
    class Logger:
        def __init__(self):
            self.errors = []

        def info(self, message):
            pass

        def error(self, message):
            self.errors.append(message)

    task = object.__new__(TurntableTask)
    task.params = {
        'rod_radius_m': 0.005,
        'approach_standoff_m': 0.1, 'insert_depth_m': 0.02,
        'stroke_yaw_deg': 6.0,
        'allow_contact_motion': False,
    }
    task.log = Logger()
    task._wait_new_observation = lambda *_args, **_kwargs: {
        'angle_deg': 90.0, 'phase_valid': True, 'capture_stamp_ns': 1,
        'disk_center_world': [1.0, 0.0, 0.1], 'disk_axis_yaw_deg': 0.0,
        'axis_ratio': 0.95, 'plane_residual_m': 0.005,
    }
    task._measured_pose = lambda: (0.0, 0.0, 0.0, 0.0)
    task._motion = lambda *_args, **_kwargs: pytest.fail('禁动模式下发送了运动命令')
    assert not task.execute()
    assert 'allow_contact_motion=false' in task.log.errors[-1]


def test_camera_to_rod_tip_geometry():
    task = object.__new__(TurntableTask)
    task.params = {
        'rod_radius_m': 0.005,
        'approach_standoff_m': 0.1, 'insert_depth_m': 0.02,
    }
    tip, radius = task._calibration()
    assert tip == pytest.approx((0.39, -0.09, 0.076))
    assert radius == pytest.approx(0.05875)


def test_spoke_clearance_rejects_oversized_rod():
    task = object.__new__(TurntableTask)
    task.params = {
        'rod_radius_m': 0.024,
        'approach_standoff_m': 0.1, 'insert_depth_m': 0.02,
    }
    with pytest.raises(ValueError, match='净空不足'):
        task._calibration()


def test_initial_visual_timeout_reports_camera_rejection_reason():
    task = object.__new__(TurntableTask)
    task.node = SimpleNamespace(stopped=False)
    task._lock = threading.RLock()
    task._observation = None
    task._last_observation_error = None
    task._on_observation(String(data=json.dumps({
        'capture_stamp_ns': 1,
        'valid': False,
        'reason': 'missing_disk_mask',
    })))
    with pytest.raises(RuntimeError, match='初始等待超时.*missing_disk_mask'):
        task._wait_new_observation(0, timeout=0.01)


def test_three_strokes_do_not_require_yellow_progress():
    class Logger:
        def info(self, _message):
            pass

        def error(self, message):
            pytest.fail(message)

    class Node:
        stopped = False

    task = object.__new__(TurntableTask)
    task.node = Node()
    task.log = Logger()
    task.params = {
        'allow_contact_motion': True,
        'force_limited_control_confirmed': True,
        'rod_radius_m': 0.005,
        'approach_standoff_m': 0.05,
        'insert_depth_m': 0.01,
        'stroke_yaw_deg': 4.0,
    }
    observations = iter({
        'capture_stamp_ns': index,
        'disk_center_world': [1.0, 0.0, 0.1],
        'disk_axis_yaw_deg': 0.0,
        'axis_ratio': 0.95,
        'plane_residual_m': 0.005,
        'phase_valid': True,
        'angle_deg': 45.0,  # 故意不变：不应据此判定失败
    } for index in range(1, 8))
    task._wait_new_observation = lambda *_args, **_kwargs: next(observations)
    task._after_motion_observation = lambda *_args: next(observations)
    task._measured_pose = lambda: (0.0, 0.0, 0.0, 0.0)
    task._check_hole_alignment = lambda *_args: None
    commands = []
    task._motion = lambda command, target, axes, context, timeout=30.0: commands.append(
        (command, tuple(target), axes, context))
    assert task.execute()
    assert sum('次推盘' in context for _, _, _, context in commands) == 6
    assert sum('次插入' in context for _, _, _, context in commands) == 9
    assert sum('盘外复位' == context for _, _, _, context in commands) == 2
