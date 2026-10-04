"""转盘几何与禁动参数测试；不连接推进器或仿真。"""

from pathlib import Path

import pytest

from uv_camera.turntable_vision import EXPECTED_DISK_DIAMETER_M
from uv_task.config_loader import load_task
from uv_task.turntable_task import (
    TurntableTask, _hole_world, _robot_for_tip, _rotation_delta, _tip_world,
)


def test_rod_tip_inverse_kinematics():
    tip_body = (0.42, -0.08, 0.02)
    desired_tip = (2.1, 3.2, 0.7)
    robot = _robot_for_tip(desired_tip, 35.0, tip_body)
    actual_tip = _tip_world(robot, 35.0, tip_body)
    assert actual_tip == pytest.approx(desired_tip)


def test_vertical_hole_and_wrapped_progress():
    # z 向下；90° 孔位应在盘心上方。
    assert _hole_world((1, 2, 1), 0, 0.2, 90, -0.1) == pytest.approx(
        (0.9, 2.0, 0.8))
    assert _rotation_delta(358, 3) == pytest.approx(5)
    assert _rotation_delta(3, 358) == pytest.approx(-5)


def test_unfilled_geometry_is_rejected():
    task = object.__new__(TurntableTask)
    task.params = {'allow_contact_motion': False}
    with pytest.raises(ValueError, match='尚未标定'):
        task._calibration()


def test_turntable_template_loads_new_disk_dimensions_without_enabling_contact():
    config = Path(__file__).resolve().parents[1] / 'config/tasks/turntable.yaml'
    params = load_task(config)[0]['params']
    assert params['disk_diameter_m'] == pytest.approx(0.230)
    assert params['disk_diameter_m'] == EXPECTED_DISK_DIAMETER_M
    assert params['inner_radius_m'] == pytest.approx(0.0175)
    assert params['outer_radius_m'] == pytest.approx(0.100)
    assert params['spoke_width_m'] == pytest.approx(0.020)
    assert params['contact_radius_m'] == pytest.approx(0.05875)
    assert params['allow_contact_motion'] is False


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
        'front_camera_center_x': 0.23, 'front_camera_center_y': 0.0,
        'front_camera_center_z': 0.076, 'disk_diameter_m': 0.230,
        'rod_radius_m': 0.005, 'inner_radius_m': 0.0175,
        'outer_radius_m': 0.100, 'spoke_width_m': 0.020,
        'contact_radius_m': 0.05875,
        'label_to_hole_deg': 0.0, 'image_angle_to_disk_sign': 1,
        'approach_standoff_m': 0.1, 'insert_depth_m': 0.02,
        'stroke_count': 3, 'stroke_yaw_deg': 6.0,
        'yaw_step_deg': 2.0, 'drive_yaw_sign': 1,
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
        'front_camera_center_x': 0.23, 'front_camera_center_y': 0.0,
        'front_camera_center_z': 0.076, 'disk_diameter_m': 0.230,
        'rod_radius_m': 0.005, 'inner_radius_m': 0.0175,
        'outer_radius_m': 0.100, 'spoke_width_m': 0.020,
        'contact_radius_m': 0.05875,
        'label_to_hole_deg': 0.0, 'image_angle_to_disk_sign': 1,
        'approach_standoff_m': 0.1, 'insert_depth_m': 0.02,
    }
    tip, radius = task._calibration()
    assert tip == pytest.approx((0.39, -0.09, 0.076))
    assert radius == pytest.approx(0.05875)


def test_spoke_clearance_rejects_oversized_rod():
    task = object.__new__(TurntableTask)
    task.params = {
        'front_camera_center_x': 0.23, 'front_camera_center_y': 0.0,
        'front_camera_center_z': 0.076, 'disk_diameter_m': 0.230,
        'rod_radius_m': 0.024, 'inner_radius_m': 0.0175,
        'outer_radius_m': 0.100, 'spoke_width_m': 0.020,
        'contact_radius_m': 0.05875,
    }
    with pytest.raises(ValueError, match='净空不足'):
        task._calibration()
