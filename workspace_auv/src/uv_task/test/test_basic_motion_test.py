"""Offline checks: refusal, fail-fast, cancellation, and watchdog evidence."""

import ast
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import pytest

from uv_task.basic_motion_test import (
    MotionTestError, RosMotionTest, Settings, build_steps, expected_target, run_test,
)
from uv_task.config_loader import load_mission_or_task


class FakePort:
    def __init__(self, fail_at=None, stale=False, bad_watchdog=False):
        self.clock = 0.0
        self.sent = []
        self.fail_at = fail_at
        self.stale = stale
        self.bad_watchdog = bad_watchdog
        self.neutrals = 0

    def configure(self, settings):
        self.settings = settings

    def ready(self):
        if self.stale:
            raise MotionTestError('feedback', '定位输入不健康')

    def check(self):
        pass

    def pose(self):
        return [0.0] * 4

    def set_reference(self, pose):
        pass

    def now(self):
        return self.clock

    def send(self, command, target, timeout):
        self.sent.append((command, list(target)))
        if len(self.sent) == self.fail_at:
            raise MotionTestError('motion', 'Action 失败')

    def pause(self, seconds):
        self.clock += seconds

    def wait_reset(self, sent_at):
        pass

    def verify_pose(self, target):
        pass

    def verify_watchdog(self, sent_at):
        if self.bad_watchdog:
            raise MotionTestError('watchdog', '没有租约到期零速度输出')

    def neutral(self):
        self.neutrals += 1

    def log(self, text):
        pass


def test_stale_preflight_dispatches_no_action_including_zero():
    port = FakePort(stale=True)
    outcome = run_test(port, {'stage': 'all', 'reset_origin': True})
    assert not outcome
    assert outcome.failure_code == 'basic_motion_test.feedback'
    assert port.sent == [] and port.neutrals == 0


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -0.2, 3, True])
def test_invalid_distance_dispatches_nothing(value):
    port = FakePort()
    assert not run_test(port, {'distance_m': value})
    assert port.sent == [] and port.neutrals == 0


def test_first_failed_excursion_ends_sequence_and_sends_neutral():
    port = FakePort(fail_at=2)
    outcome = run_test(port, {'stage': 'all'})
    assert outcome.failure_code == 'basic_motion_test.motion'
    assert len(port.sent) == 2
    assert port.neutrals == 1


def test_missing_watchdog_zero_is_a_failure_and_neutral_is_attempted():
    port = FakePort(bad_watchdog=True)
    outcome = run_test(port, {'stage': 'velocity'})
    assert outcome.failure_code == 'basic_motion_test.watchdog'
    assert port.sent[-1][0] == 7
    assert port.neutrals == 1


def test_neutral_failure_overrides_a_successful_motion_result():
    port = FakePort()
    def failed_neutral():
        raise RuntimeError('no ack')
    port.neutral = failed_neutral
    outcome = run_test(port, {'stage': 'hold'})
    assert outcome.failure_code == 'basic_motion_test.stop'


def test_body_offset_and_yaw_are_checked_in_world_frame():
    target = expected_target(2, [0.25, 0.0, 0.2, 10.0], [3.0, 4.0, 1.0, 90.0])
    assert target == pytest.approx([3.0, 4.25, 1.2, 100.0])


def test_world_motion_uses_absolute_targets_away_from_zero_origin():
    reference = [3.0, 4.0, 0.4, 90.0]
    steps = build_steps(Settings(stage='wmove'), reference)
    targets = [step[2] for step in steps if step[1] == 1]
    assert targets == [[3.0, 4.0, 0.4, 100.0], [3.25, 4.0, 0.4, 90.0], [3.0, 4.25, 0.4, 90.0]]
    assert expected_target(1, targets[1], reference) == targets[1]


def test_world_travel_preserves_entry_depth_and_uses_absolute_xy():
    reference = [3.0, 4.0, 0.4, 90.0]
    steps = build_steps(Settings(stage='travel'), reference)
    target = next(step[2] for step in steps if step[1] == 4)
    assert target == pytest.approx([3.0, 4.25, 0.4, 90.0])
    assert expected_target(4, target, reference) == pytest.approx(target)


def driver_with_feedback():
    driver = object.__new__(RosMotionTest)
    driver.now = lambda: 10.0
    driver.node = NS(stopped=False)
    driver.settings = Settings()
    driver.reference = [0.0] * 4
    driver.lock = threading.Lock()
    driver.feedback = {
        'pose': (10.0, NS(robot_x=0.0, robot_y=0.0, robot_z=0.0,
                         robot_yaw=0.0, robot_roll=0.0, robot_pitch=0.0)),
        'twist': (10.0, NS(twist=NS(twist=NS(linear=NS(x=0., y=0., z=0.), angular=NS(x=0., y=0., z=0.))))),
        'health': (10.0, NS(available=True)),
        'status': (10.0, NS(is_armed=True, navigation_ready=True,
                           error_flags=0, battery_voltage=25.0)),
    }
    return driver


def test_republishing_odom_does_not_hide_unhealthy_raw_position():
    driver = driver_with_feedback()
    driver.feedback['health'][1].available = False
    with pytest.raises(MotionTestError, match='原始定位反馈不健康'):
        driver.check()


@pytest.mark.parametrize('field,value', [('is_armed', False), ('navigation_ready', False), ('error_flags', 2), ('battery_voltage', 0.0)])
def test_mcu_readiness_and_voltage_are_required(field, value):
    driver = driver_with_feedback()
    setattr(driver.feedback['status'][1], field, value)
    with pytest.raises(MotionTestError):
        driver.check()


def test_missing_battery_telemetry_requires_current_external_measurement():
    driver = driver_with_feedback()
    driver.feedback['status'][1].battery_voltage = 0.0
    with pytest.raises(MotionTestError, match='本轮必须填写'):
        driver.check()
    driver.settings.external_battery_voltage = 24.0
    driver.check()


@pytest.mark.parametrize('voltage', [20.0, -1.0, float('nan')])
def test_external_measurement_cannot_override_bad_nonzero_telemetry(voltage):
    driver = driver_with_feedback()
    driver.feedback['status'][1].battery_voltage = voltage
    driver.settings.external_battery_voltage = 24.0
    with pytest.raises(MotionTestError, match='电压未达到'):
        driver.check()


@pytest.mark.parametrize('voltage', [20.0, float('nan'), float('inf')])
def test_bad_external_voltage_is_rejected(voltage):
    driver = driver_with_feedback()
    driver.feedback['status'][1].battery_voltage = 0.0
    driver.settings.external_battery_voltage = voltage
    with pytest.raises(MotionTestError, match='电压未达到'):
        driver.check()


def test_excursion_limit_aborts_even_if_action_has_not_completed():
    driver = driver_with_feedback()
    driver.feedback['pose'][1].robot_x = 1.2
    with pytest.raises(MotionTestError, match='位姿超出'):
        driver.check()


def test_velocity_watchdog_uses_actual_zit6_velocity_key_and_all_axes():
    driver = driver_with_feedback()
    driver.feedback['setpoint'] = (10.0, NS(control_key=0x11, x=0., y=0., z=0., yaw=0.))
    driver.verify_watchdog(9.0)
    driver.feedback['setpoint'][1].y = 0.1
    with pytest.raises(MotionTestError):
        driver.verify_watchdog(9.0)


def test_stop_before_pending_goal_ack_cancels_late_accepted_goal():
    calls = []
    handle = NS(accepted=True, cancel_goal_async=lambda: calls.append('cancel'))
    RosMotionTest.cancel_late_goal(NS(result=lambda: handle))
    assert calls == ['cancel']


def runner_method(name):
    path = Path(__file__).parents[1] / 'uv_task/task_runner.py'
    tree = ast.parse(path.read_text())
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == 'TaskRunnerNode')
    method = next(item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name == name)
    namespace = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


def test_runner_stop_uses_public_goal_handle_cancellation():
    calls = []
    logger = NS(warning=lambda text: None)
    handle = NS(cancel_goal_async=lambda: calls.append('cancel'))
    node = NS(_motion_stop_sent=False, stopped=False, _active_goal_handle=handle,
              get_logger=lambda: logger)
    runner_method('_stop_active_motion')(node)
    assert calls == ['cancel'] and node.stopped


def test_run_service_rejects_an_overlapping_mission():
    node = NS(running=True, _debug_executing=False)
    response = runner_method('_run_task_cb')(node, NS(start=True), NS())
    assert not response.success


def test_shipped_task_is_staged_and_initializes_only_on_first_run():
    path = Path(__file__).parents[1] / 'config/tasks/basic_motion_test.yaml'
    tasks = load_mission_or_task(path)
    assert tasks[0]['name'] == 'basic_motion_test'
    settings = Settings.load(tasks[0]['params'])
    assert settings.stage == 'hold' and settings.reset_origin


def test_shallow_water_default_does_not_command_vertical_motion():
    settings = Settings(stage='all')
    reference = [0.0, 0.0, 0.0, 0.0]
    assert all(step[2][2] == 0.0 for step in build_steps(settings, reference))
    settings.test_depth = True
    assert any(step[2][2] > 0.0 for step in build_steps(settings, reference))


def test_manual_stages_keep_loaded_battery_limit_without_repeating_start(monkeypatch):
    from uv_task import basic_motion_test
    captured = []
    monkeypatch.setattr(basic_motion_test, 'run_test', lambda port, params: captured.append(params))
    driver = driver_with_feedback()
    driver.settings.min_battery_voltage = 22.0
    driver.settings.reset_origin = True
    driver.settings.external_battery_voltage = 24.0
    driver.run({'stage': 'bmove'})
    assert captured[0]['min_battery_voltage'] == 22.0
    assert not captured[0]['reset_origin']
    assert captured[0]['external_battery_voltage'] == 0.0


def test_completed_test_failure_is_exposed_in_status():
    from uv_task.task_outcome import TaskOutcome
    messages = []
    class Status:
        STATUS_RUNNING = 1
        STATUS_DONE = 3
        STATUS_ERROR = 4
        STATUS_PAUSED = 2
        STATUS_IDLE = 0
    path = Path(__file__).parents[1] / 'uv_task/task_runner.py'
    tree = ast.parse(path.read_text())
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == 'TaskRunnerNode')
    method = next(item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name == '_publish_status')
    namespace = {'TaskStatus': Status}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
    node = NS(_debug_executing=False, running=False, stopped=True,
              _last_basic_motion_test_outcome=TaskOutcome.failed('basic_motion_test.feedback', 'stale'),
              pub_status=NS(publish=messages.append), pub_status_legacy=NS(publish=lambda message: None))
    namespace['_publish_status'](node)
    assert messages[0].status == 4
    assert messages[0].current_task_name == 'basic_motion_test'
    assert 'stale' in messages[0].error_message
