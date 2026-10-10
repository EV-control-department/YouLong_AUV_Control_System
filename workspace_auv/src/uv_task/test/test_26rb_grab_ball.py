"""抓球流程的偏置、回位复检和重试测试。"""

from importlib import import_module
import threading
import time
import math
import ast
from pathlib import Path
import pytest
from types import SimpleNamespace


_grab = import_module('uv_task.26rb_grab_ball')
GrabBallTask = _grab.RB26GrabBallTask
GrabSeaCucumberTask = import_module('uv_task.grab_sea_cucumber').GrabSeaCucumberTask


class _Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


def test_sea_horizontal_hold_activates_position_even_when_already_centered(monkeypatch):
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    calls = []
    task._logger, task._color = _Logger(), '收集框'
    task._servo_timeout, task._command_timeout = 30., 2.
    task._measured_pose = lambda: (1., 2., .4, 0., 0., -90.)
    task._node = SimpleNamespace(stopped=False,
        _send_action_goal=lambda *args, **kwargs: calls.append(args) or (True, ''),
        _format_motion_context=lambda s: s)
    task._wait_visual_motion = lambda target, deadline: True
    def servo(deadline):
        assert task._horizontal_hold_z_yaw == (.4, -90.)
        return [1., 2., .4, -90.]
    task._locked_visual_loop = servo
    assert task._servo_horizontally() == [1., 2., .4, -90.]
    assert calls[0][1:] == ([1., 2., .4, -90.], 'xyzrz')
    assert (task._node._cmd_z, task._node._cmd_yaw) == (.4, -90.)
    assert task._servo_timeout == 30.


def test_sea_gripper_offset_uses_fresh_xy_but_preserves_hold_z_yaw():
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    calls = []
    task._logger, task._color = _Logger(), '收集框'
    task._command_timeout = 2.
    task._horizontal_hold_z_yaw = (.4, -90.)
    task._gripper_offset_x, task._gripper_offset_y = .25, 0.
    task._measured_pose = lambda: (3., 4., .48, 0., 0., -90.)
    task._node = SimpleNamespace(
        _send_action_goal=lambda *args, **kwargs: calls.append(args) or (True, ''),
        _format_motion_context=lambda s: s)
    assert task._apply_gripper_offset()
    assert calls[0][2] == 'xyzrz'
    assert calls[0][1] == pytest.approx([3., 3.75, .4, -90.])


def test_down_camera_vertical_pixel_error_is_horizontal_not_depth():
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._node = SimpleNamespace(_latest_robot_pose=lambda: (0, 0, .4, 0, 0, 0))
    task._CX, task._CY, task._FX, task._FY = 320., 240., 500., 500.
    task._projection_depth, task._servo_gain, task._max_xy_step = .8, .5, .08
    task._camera_mount_yaw = 180.
    step = task._horizontal_step(SimpleNamespace(pixel_x=320., pixel_y=340.))
    assert step[1:5] == pytest.approx((.08, 0., .08, 0.))
    assert step[0][2] == .4


def test_horizontal_corrections_do_not_absorb_depth_heading_drift(monkeypatch):
    task = GrabBallTask.__new__(GrabBallTask)
    calls, clock = [], [0.]
    monkeypatch.setattr(_grab.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(_grab.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    task._logger, task._class_id, task._color = _Logger(), 4, '收集框'
    task._servo_timeout, task._servo_period, task._hold_seconds = 5., .1, .1
    task._log_period, task._pixel_tolerance = .5, .04
    task._projection_depth, task._command_timeout = .8, 1.
    task._node = SimpleNamespace(stopped=False, _cmd_z=.6,
        _latest_robot_pose=lambda: (1., 2., .6, 0., 0., -70.),
        _send_action_goal=lambda *args, **kwargs: calls.append(args) or (True, ''),
        _format_motion_context=lambda s: s)
    task._best_left_detection = lambda: SimpleNamespace(pixel_x=350., pixel_y=260.)
    # 两次修正反馈都有z/yaw漂移，最后居中；保持目标不得跟随漂移。
    steps = iter([(task._node._latest_robot_pose(), .02, .02, .02, .02, .1, .1),
                  (task._node._latest_robot_pose(), .01, .01, .01, .01, .08, .08)])
    centered = (task._node._latest_robot_pose(), 0., 0., 0., 0., 0., 0.)
    task._horizontal_step = lambda d: next(steps, centered)
    assert task._servo_horizontally(hold_z_yaw=(.4, -90.)) is not None
    assert len(calls) == 2
    for call in calls:
        assert call[2] == 'xyzrz'
        assert call[1][2:] == [.4, -90.]


def test_failed_start_prevents_grab_and_movement():
    source = Path(__file__).parents[1] / 'uv_task' / 'task_runner.py'
    cls = next(n for n in ast.parse(source.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'TaskRunnerNode')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'run_task_list')
    scope = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), scope)
    calls = []
    logger = SimpleNamespace(info=lambda msg: None, warn=lambda msg: None, error=lambda msg: None)
    node = SimpleNamespace(tasks=[{'name': 'start'}, {'name': 'grab_sea_cucumber'}],
                           _cmd_x=0., _cmd_y=0., _cmd_z=0., _cmd_yaw=0.,
                           get_logger=lambda: logger,
                           _execute_task=lambda name, params: calls.append(name) or False)
    scope['run_task_list'](node)
    assert calls == ['start']
    assert node.stopped and not node.running


def test_negative_sea_descent_rejected_before_initialization():
    params = dict(sea_cucumber_class_id=2, image_width=640, image_height=480,
                  gripper_offset_x_m=.3, gripper_offset_y_m=0.,
                  descent_speed_mps=-.5, descent_duration_seconds=4.,
                  max_press_distance_m=1.3, drop_pose=[0,0,.3,0], search_pose=[1,1,.3,0])
    with pytest.raises(ValueError, match='有限正数'):
        GrabSeaCucumberTask(None, params)


def _locked_detection_task():
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._class_id, task._min_confidence, task._detection_timeout = 2, .35, 1.
    task._association_active, task._locked_pixel, task._locked_pose = True, None, None
    task._target_match_px = 70.
    task._measured_pose = lambda: (0., 0., .4, 0., 0., 0.)
    task._CX, task._CY, task._FX, task._FY = 320., 240., 500., 500.
    task._projection_depth, task._camera_mount_yaw = .8, 180.
    task._node = SimpleNamespace(_perception_lock=threading.Lock(), _down_detections={})
    return task


def _set_detection_frame(task, candidates, stamp=10):
    task._node._down_detections['down_left'] = (time.monotonic(), SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=stamp)),
        detections=[SimpleNamespace(class_id=2, confidence=confidence,
            mask_x=[x-1, x, x+1], mask_y=[239., 240., 241.])
                    for x, confidence in candidates]))


def test_sea_lock_does_not_follow_alternating_confidence_and_waits_if_missing():
    task = _locked_detection_task()
    _set_detection_frame(task, [(260., .9), (400., .8)])
    assert task._best_left_detection().pixel_x == 260.
    _set_detection_frame(task, [(265., .7), (400., .99)], 11)
    assert task._best_left_detection().pixel_x == 265.
    _set_detection_frame(task, [(400., .99)], 12)
    assert task._best_left_detection() is None
    assert task._locked_pixel == (265., 240.)
    _set_detection_frame(task, [(270., .8), (400., .99)], 13)
    assert task._best_left_detection().pixel_x == 270.


def test_sea_lock_predicts_pixel_from_actual_body_motion():
    task = _locked_detection_task()
    _set_detection_frame(task, [(260., .9), (400., .8)])
    assert task._best_left_detection().pixel_x == 260.
    # 倒装相机：艇体+y移动0.16m，同一目标像素右移100；超过旧像素70px门限。
    task._measured_pose = lambda: (0., .16, .4, 0., 0., 0.)
    _set_detection_frame(task, [(360., .7), (260., .99)], 11)
    assert task._best_left_detection().pixel_x == pytest.approx(360.)


def test_sea_rejects_frame_captured_before_motion_settled():
    task = _locked_detection_task()
    task._scan_capture_after_ns = 20
    _set_detection_frame(task, [(260., .9)], 19)
    assert task._best_left_detection() is None
    _set_detection_frame(task, [(260., .9)], 21)
    assert task._best_left_detection() is not None


@pytest.mark.parametrize('actually_moved', [False, True])
def test_visual_arrival_is_independent_of_basic_motion_tolerance(monkeypatch, actually_moved):
    clock = [0.]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0]+seconds))
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._logger = _Logger()
    task._servo_xy_tolerance, task._servo_settle_seconds = .01, .3
    task._command_timeout, task._log_period = 1., .5
    task._node = SimpleNamespace(stopped=False,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock[0]*1e9))))
    task._measured_pose = lambda: (.03 if actually_moved else 0., 0., .4, 0., 0., 0.)
    assert task._wait_visual_motion([.03, 0., .4, 0.], 2.) is actually_moved
    if actually_moved:
        assert task._scan_capture_after_ns >= 300_000_000
    else:
        assert not hasattr(task, '_scan_capture_after_ns')


def test_locked_loop_does_not_command_twice_from_same_frame(monkeypatch):
    clock, calls = [0.], []
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0]+seconds))
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._logger, task._color = _Logger(), '海参'
    task._locked_pixel = (400., 240.)
    task._servo_period, task._hold_seconds, task._log_period = .1, .1, .5
    task._pixel_tolerance, task._servo_xy_tolerance = .04, .01
    task._command_timeout, task._horizontal_hold_z_yaw = 1., (.4, 0.)
    task._node = SimpleNamespace(stopped=False,
        _send_action_goal=lambda *args, **kwargs: calls.append(args) or (True, ''),
        _format_motion_context=lambda s: s)
    frames = iter([1, 1, 2, 3, 4])
    task._best_left_detection = lambda: SimpleNamespace(stamp_ns=next(frames, 5), pixel_x=400., pixel_y=240.)
    pose = (0., 0., .4, 0., 0., 0.)
    task._horizontal_step = lambda d: (pose, .03, 0., .03, 0., .1 if d.stamp_ns == 1 else 0., 0.)
    task._measured_pose = lambda: pose
    task._wait_visual_motion = lambda *args: True
    assert task._locked_visual_loop(2.) is not None
    assert len(calls) == 1


def test_locked_target_loss_waits_without_switching_or_commanding(monkeypatch):
    clock = [0.]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0]+seconds))
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._logger, task._locked_pixel = _Logger(), (260., 240.)
    task._target_lost_seconds, task._servo_period, task._log_period = 2., .1, .5
    task._node = SimpleNamespace(stopped=False)
    task._best_left_detection = lambda: None
    assert task._locked_visual_loop(10.) is None
    assert 2. <= clock[0] < 2.2
    assert task._locked_pixel == (260., 240.)


def test_visual_tracking_parameters_load_from_nested_servo_config():
    import yaml
    from uv_task.config_loader import load_task
    path = Path(__file__).parents[1] / 'config/tasks/grab_sea_cucumber.yaml'
    servo = yaml.safe_load(path.read_text())['params']['servo']
    params = load_task(path)[0]['params']
    for nested, flat in [('target_match_radius_px', 'target_match_radius_px'),
                         ('target_lost_wait_seconds', 'target_lost_wait_seconds'),
                         ('position_tolerance_m', 'visual_position_tolerance_m'),
                         ('motion_settle_seconds', 'visual_motion_settle_seconds')]:
        assert params[flat] == servo[nested]


def test_sea_degree_config_preserves_internal_radians_without_old_descent_cap():
    from uv_task.config_loader import load_task
    params = load_task(Path(__file__).parents[1] / 'config/tasks/grab_sea_cucumber.yaml')[0]['params']
    params.update(descent_speed_mps=.5, descent_duration_seconds=3.,
                  ascent_speed_mps=.01,
                  max_press_distance_m=1.3, pickup_servo_angle_deg=0., release_servo_angle_deg=90.)
    node = SimpleNamespace(get_logger=lambda: _Logger())
    task = GrabSeaCucumberTask(node, params)
    assert task._pickup_angle == 0.
    assert task._release_angle == pytest.approx(math.pi/2)
    assert task._descent_speed == .5 and task._descent_duration == 3.
    assert task._ascent_speed == .01


def test_sea_descent_exception_still_sends_zero_velocity():
    task = _sea_flow_fake()
    del task._descend
    task._descent_speed, task._descent_duration, task._descent_period = .5, 3., .05
    packets = []
    def publish(vertical_mps=0.):
        packets.append(vertical_mps)
        if vertical_mps:
            raise RuntimeError('publish failed')
    task._node._publish_body_velocity = publish
    with pytest.raises(RuntimeError, match='publish failed'):
        task._descend()
    assert packets == [.5,0.]


def test_near_floor_press_is_latched_force_and_neutral_on_exception():
    task = _sea_flow_fake()
    del task._descend
    task._near_floor_enabled = True
    task._floor_z, task._bottom_offset, task._force_clearance = 1., 0., .2
    task._press_thrust, task._press_seconds = .15, .02
    task._descent_duration, task._descent_period = .2, .001
    task._measured_pose = lambda: (1,2,.81,0,0,0)
    packets = []
    def force(value):
        packets.append(value)
        if value:
            raise RuntimeError('test force failure')
    task._node._publish_body_thrust = force
    task._node._publish_body_velocity = lambda **kw: pytest.fail('near floor used velocity')
    with pytest.raises(RuntimeError, match='test force failure'):
        task._descend()
    assert packets == [.15, 0.]
    assert task._pre_press_pose[:2] == (1,2)


def test_press_switches_velocity_to_force_once():
    task = _sea_flow_fake()
    del task._descend
    task._near_floor_enabled = True
    task._floor_z, task._bottom_offset, task._force_clearance = 1., 0., .2
    task._press_thrust, task._press_seconds = .15, .005
    task._descent_speed, task._descent_duration, task._descent_period = .1, .1, .001
    poses = iter([(0,0,.5,0,0,0), (0,0,.6,0,0,0), (0,0,.81,0,0,0)])
    task._measured_pose = lambda: next(poses)
    packets = []
    task._node._publish_body_velocity = lambda **kw: packets.append(('velocity', kw))
    task._node._publish_body_thrust = lambda value: packets.append(('force', value))
    assert task._descend()
    assert packets[0][0] == 'velocity'
    assert all(p[0] == 'force' for p in packets[1:])
    assert packets[-1] == ('force', 0.)


def test_collection_align_before_release_and_fallback_on_failure():
    task = _sea_flow_fake()
    task._collection_align = True
    task._drop_pose, task._drop_timeout = (0,0,.5,0), 1
    task._release_angle, task._release_wait = math.pi/2, 0
    events = []
    task._travel = lambda *a: events.append('travel') or True
    task._align_collection = lambda deadline: events.append('align') or False
    task._node.set_servo = lambda *a: events.append('release')
    assert task._deliver(task._search_pose, time.monotonic()+2, False)
    assert events == ['travel', 'align', 'travel', 'release']
    events.clear()
    task._align_collection = lambda deadline: events.append('align') or True
    assert task._deliver(task._search_pose, time.monotonic()+2, False)
    assert events == ['travel', 'align', 'release']


def test_collection_class_is_restored_after_alignment_failure():
    task = _sea_flow_fake()
    task._pixel_tolerance, task._hold_seconds = .04, .5
    task._class_id, task._color, task._projection_depth = 2, '海参', .8
    task._collection_class, task._collection_depth = 4, .5
    def servo():
        assert task._class_id == 4 and task._projection_depth == .5
        assert task._pixel_tolerance == .08 and task._hold_seconds == 0.
        return None
    task._servo_horizontally = servo
    assert not task._align_collection(time.monotonic()+1)
    assert (task._pixel_tolerance, task._hold_seconds) == (.04, .5)
    assert (task._class_id, task._color, task._projection_depth) == (2, '海参', .8)


def test_duplicate_empty_frame_is_not_repeated_target_loss():
    task = _locked_detection_task()
    _set_detection_frame(task, [(260., .9)])
    assert task._best_left_detection() is not None
    _set_detection_frame(task, [], 11)
    assert task._best_left_detection() is None
    assert task._detection_reject_reason == '新帧无匹配目标'
    assert task._best_left_detection() is None
    assert task._detection_reject_reason == '重复采集帧'


def test_collection_accepts_fresh_centered_frame_without_extra_hold(monkeypatch):
    clock = [0.]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0]+seconds))
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._logger, task._color = _Logger(), '收集框'
    task._node = SimpleNamespace(stopped=False)
    task._pixel_tolerance, task._hold_seconds, task._log_period = .08, 0., .5
    task._best_left_detection = lambda: SimpleNamespace(stamp_ns=1, pixel_x=316.5, pixel_y=212.)
    task._horizontal_step = lambda d: ((0.,0.,.4,0.,0.,0.), -.03,.016,-.03,.016,-.0398,-.0749)
    task._measured_pose = lambda: (0.,0.,.4,0.,0.,0.)
    assert task._locked_visual_loop(.01) == [0.,0.,.4,0.]
    assert clock[0] == 0.


def test_open_loop_requires_calibration_not_velocity_as_force():
    from uv_task.config_loader import load_task
    params = load_task(Path(__file__).parents[1] / 'config/tasks/grab_sea_cucumber.yaml')[0]['params']
    params['near_floor_open_loop'] = True
    params['collection_correct_odom_xy'] = False
    params['press_thrust'] = 0.0  # 显式未标定，不依赖现场YAML当前值。
    with pytest.raises(ValueError, match='先标定'):
        GrabSeaCucumberTask(SimpleNamespace(get_logger=lambda: _Logger()), params)
    params.update(floor_z_m=1.4, press_thrust=.1, lift_thrust=-.03)
    task = GrabSeaCucumberTask(SimpleNamespace(get_logger=lambda: _Logger()), params)
    assert task._near_floor_enabled and not task._collection_correct


def test_near_floor_ascent_clears_force_before_velocity():
    task = _sea_flow_fake()
    task._near_floor_enabled = True
    task._floor_z, task._bottom_offset, task._force_clearance = 1., 0., .2
    task._lift_thrust = -.03
    task._ascent_speed, task._ascent_period, task._ascent_tolerance = .01, .001, .01
    poses = iter([(0,0,.9,0,0,0), (0,0,.88,0,0,0),
                  (0,0,.70,0,0,0), (0,0,.5,0,0,0)])
    task._measured_pose = lambda: next(poses)
    packets = []
    task._node._publish_body_thrust = lambda value: packets.append(('force',value))
    task._node._publish_body_velocity = lambda vertical_mps=0: packets.append(('velocity', vertical_mps))
    assert task._ascend_to_depth(.5, time.monotonic()+1)
    assert packets[:2] == [('force',-.03), ('force',0.)]
    assert packets[2][0] == 'velocity' and packets[2][1] < 0
    assert packets[-1] == ('velocity',0.)


def test_force_wire_uses_absolute_body_actuator_and_clears_other_axes():
    from zit6_interfaces.msg import ZitSetpoint
    source = Path(__file__).parents[1] / 'uv_task' / 'task_runner.py'
    cls = next(n for n in ast.parse(source.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'TaskRunnerNode')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                  and n.name == '_publish_body_thrust')
    scope = {'math': math, 'ZitSetpoint': ZitSetpoint}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), scope)
    packets = []
    scope['_publish_body_thrust'](SimpleNamespace(pub_setpoint=SimpleNamespace(publish=packets.append)), .1)
    msg = packets[0]
    assert msg.control_key == 0x12 and msg.type_mask == 0
    assert msg.z == pytest.approx(.1)
    assert (msg.x,msg.y,msg.roll,msg.pitch,msg.yaw) == (0.,)*5


def test_sea_rejects_stale_pose_before_direct_velocity_control():
    task = _sea_flow_fake()
    del task._measured_pose  # 检查真实方法，不使用流程测试替身。
    task._node._perception_lock = threading.Lock()
    task._node._robot_pose = (0,0,.8,0,0,0)
    task._node._robot_pose_received = time.monotonic() - 2.
    assert task._measured_pose() is None


@pytest.mark.parametrize('robot_yaw', [0., 90., -45.])
@pytest.mark.parametrize('pixel', [(420.,240.), (320.,340.), (420.,340.)])
def test_sea_camera_mount_180_reverses_body_and_world_correction(robot_yaw, pixel):
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._node = SimpleNamespace(_latest_robot_pose=lambda: (1,2,.8,0,0,robot_yaw))
    task._CX, task._CY, task._FX, task._FY = 320.,240.,500.,500.
    task._projection_depth, task._servo_gain, task._max_xy_step = .8,.8,.08
    detection = SimpleNamespace(pixel_x=pixel[0], pixel_y=pixel[1])
    task._camera_mount_yaw = 0.
    nominal = task._horizontal_step(detection)
    task._camera_mount_yaw = 180.
    corrected = task._horizontal_step(detection)
    assert corrected[0] == nominal[0]
    assert corrected[1:5] == pytest.approx(tuple(-v for v in nominal[1:5]))
    assert corrected[5:] == nominal[5:]
    assert math.hypot(corrected[1], corrected[2]) <= .08 + 1e-9


def test_sea_cucumber_retries_and_delivers_until_five_removed():
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    calls = []
    task._node = SimpleNamespace(
        stopped=False, set_servo=lambda angle, label: calls.append(('servo', angle)),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1)))
    task._logger = _Logger()
    task._total_timeout = 10
    task._search_pose = (3, 1, .8, 0)
    task._search_travel_timeout = 2
    task._target_count = 5
    task._max_failed_attempts = 3
    task._pickup_angle = 0
    task._servo_timeout = task._return_timeout = 1
    task._descent_duration = .1
    task.confirmed_removed = task.delivery_commands = 0
    task._measured_pose = lambda: (0,0,.8,0,0,0)
    task._travel = lambda pose, *args: calls.append(('travel', pose)) or True
    task._servo_horizontally = lambda: (.1,.2,.8,0)
    counts = iter((5,5,5,3,3,0))
    task._count_visible = lambda *args: next(counts)
    task._apply_gripper_offset = task._wait_pre_descent_settle = task._descend = lambda: True
    task._return_to_recorded_pose = lambda pose: True
    task._deliver = lambda pose, deadline, return_to_search: calls.append(('deliver', return_to_search)) or True
    assert task.execute()
    assert calls[0] == ('travel', task._search_pose)
    assert task.confirmed_removed == 5
    assert [c for c in calls if c[0] == 'deliver'] == [('deliver', True), ('deliver', False)]
    assert len([c for c in calls if c[0] == 'servo']) == 3


def test_sea_cucumber_scan_rejects_old_capture_and_accepts_fresh_frame():
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._class_id, task._min_confidence, task._detection_timeout = 2, .35, 1
    now = time.monotonic()
    task._scan_received_after, task._scan_capture_after_ns = now-.2, 100
    stamp = SimpleNamespace(sec=0, nanosec=99)
    detection = SimpleNamespace(class_id=2, confidence=.9, mask_x=[1,2,3], mask_y=[4,5,6])
    task._node = SimpleNamespace(_perception_lock=threading.RLock(), _down_detections={
        'down_left': (now, SimpleNamespace(header=SimpleNamespace(stamp=stamp), detections=[detection]))})
    assert task._best_left_detection() is None
    stamp.nanosec = 101
    assert task._best_left_detection().pixel_x == 2


def _sea_flow_fake():
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._node = SimpleNamespace(stopped=False, set_servo=lambda *a: None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1)))
    task._logger = _Logger()
    task._total_timeout = 10
    task._search_pose = (0,0,.8,0)
    task._search_travel_timeout = task._servo_timeout = task._return_timeout = 1
    task._target_count, task._max_failed_attempts = 5, 3
    task._pickup_angle, task._descent_duration = 0, .1
    task.confirmed_removed = task.delivery_commands = 0
    task._measured_pose = lambda: (0,0,.8,0,0,0)
    task._travel = lambda *a: True
    task._servo_horizontally = lambda: (0,0,.8,0)
    task._apply_gripper_offset = task._wait_pre_descent_settle = task._descend = lambda: True
    task._return_to_recorded_pose = lambda p: True
    task._retry_perception = lambda reason, failures, deadline: failures <= 3
    return task


@pytest.mark.parametrize('failure', ['alignment', 'count_missing', 'count_zero'])
def test_sea_visual_failure_retries_instead_of_stopping(failure):
    task = _sea_flow_fake()
    deliveries = []
    if failure == 'alignment':
        positions = iter((None, (0,0,.8,0)))
        task._servo_horizontally = lambda: next(positions)
        counts = iter((5,0))
    else:
        counts = iter((None if failure == 'count_missing' else 0, 5, 0))
    task._count_visible = lambda *a: next(counts)
    task._deliver = lambda *a, **kw: deliveries.append(True) or True
    assert task.execute()
    assert task.confirmed_removed == 5 and len(deliveries) == 1


def test_sea_unknown_postgrab_count_delivers_without_claiming_success():
    task = _sea_flow_fake()
    counts = iter((5,None,5,0))
    deliveries = []
    task._count_visible = lambda *a: next(counts)
    task._deliver = lambda *a, **kw: deliveries.append(
        (task.confirmed_removed, kw['return_to_search'])) or True
    assert task.execute()
    assert deliveries == [(0,True), (5,False)]


def test_sea_partial_count_accepts_two_but_not_one_fresh_frames(monkeypatch):
    module = import_module('uv_task.grab_sea_cucumber')
    for number in (1,2):
        clock = SimpleNamespace(value=0.)
        task = _sea_flow_fake()
        task._count_frames, task._count_timeout, task._detection_timeout = 3, .25, 1
        task._node._perception_lock = threading.Lock()
        task._node._down_detections = {'down_left': (0., SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=0,nanosec=1))))}
        task._segmented_detections = lambda msg: [object()] * 5
        def sleep(dt):
            clock.value += dt
            if number == 2 and clock.value <= .1:
                task._node._down_detections['down_left'] = (.1, SimpleNamespace(
                    header=SimpleNamespace(stamp=SimpleNamespace(sec=0,nanosec=2))))
        monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: clock.value, sleep=sleep))
        result = task._count_visible(-1., 0, 10., 'before')
        assert result == (5 if number == 2 else None)


def test_task_runner_servo_protocol_and_start_reset_without_ros_node():
    # 仅提取方法执行：无需创建节点，测试不会向DDS发布或触发硬件。
    import ast
    from zit6_interfaces.msg import ZitServo
    file = Path(__file__).parents[1] / 'uv_task' / 'task_runner.py'
    tree = ast.parse(file.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TaskRunnerNode')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ('set_servo', '_do_start')]
    namespace = {'math': math, 'ZitServo': ZitServo, 'BasicMotion': _grab.BasicMotion}
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(file), 'exec'), namespace)
    messages, events = [], []
    fake = SimpleNamespace(pub_servo=SimpleNamespace(publish=messages.append),
                           get_logger=lambda: _Logger(), ANGLE_INIT=0,
                           light_off=lambda: events.append('light'),
                           _send_action_goal=lambda *args, **kw: (events.append('start') or True, 'ok'))
    fake.set_servo = lambda angle, label, **kw: namespace['set_servo'](fake, angle, label, **kw)
    fake.set_servo(math.pi/2, '放')
    assert messages[-1].servo_id == 1
    assert messages[-1].angle == pytest.approx(90.)
    fake.set_servo(math.pi, '180度')
    assert messages[-1].angle == pytest.approx(180.)
    assert namespace['_do_start'](fake)
    assert messages[-1].angle == 0
    assert messages[-1].servo_id == 1
    assert events == ['light', 'start']
    with pytest.raises(ValueError):
        fake.set_servo(90, '错误度数')
    fake.set_servo(math.radians(270), '新爪张开', servo_id=2)
    assert messages[-1].servo_id == 2 and messages[-1].angle == pytest.approx(270.)
    with pytest.raises(ValueError):
        fake.set_servo(math.radians(270), '旧舵机不能270度')
    fake.tasks = [{'name': 'start'}, {'name': 'grab_sea_cucumber', 'params': {'gripper_servo_id': 2}}]
    fake.current_index = 0
    assert namespace['_do_start'](fake)
    assert messages[-1].servo_id == 2 and messages[-1].angle == pytest.approx(150.)


def test_servo2_selection_uses_mount_geometry_and_does_not_change_servo1_profile():
    from uv_task.config_loader import load_task
    params = load_task(Path(__file__).parents[1] / 'config/tasks/grab_sea_cucumber.yaml')[0]['params']
    # 固定测试标定值，不随现场修改的YAML安装尺寸改变预期。
    params['servo2_down_camera_body_xyz'] = [-.130, .030, .0645]
    params['servo2_front_camera_body_xyz'] = [.230, 0., .076]
    params['servo2_gripper_from_front_xyz'] = [0., 0., .1]
    node = SimpleNamespace(get_logger=lambda: _Logger())
    original = GrabSeaCucumberTask(node, params)
    assert original._gripper_servo_id == 1
    assert original._gripper_offset_x == params['gripper_offset_x_m']
    params['gripper_servo_id'] = 2
    params.pop('gripper_offset_x_m')
    params.pop('gripper_offset_y_m')
    task = GrabSeaCucumberTask(node, params)
    assert task._pickup_angle == pytest.approx(math.radians(150))
    assert task._release_angle == pytest.approx(math.radians(270))
    assert (task._gripper_offset_x, task._gripper_offset_y) == pytest.approx((-.36,.030))
    assert task._bottom_offset == pytest.approx(.176)
    params.update(servo2_front_camera_body_xyz=[.4,.03,.09],
                  servo2_gripper_from_front_xyz=[.02,-.01,.1])
    task = GrabSeaCucumberTask(node, params)
    assert (task._gripper_offset_x, task._gripper_offset_y, task._bottom_offset) == pytest.approx((-.55,.010,.19))


def test_servo2_closes_after_press_before_ascent_then_releases_at_delivery():
    task = _sea_flow_fake()
    task._gripper_servo_id = 2
    task._pickup_angle, task._release_angle = math.radians(150), math.radians(270)
    task._close_wait = 0.
    task._drop_pose, task._drop_timeout, task._release_wait = (0,0,.5,0), 1, 0
    task._collection_align = False
    events = []
    task._node.set_servo = lambda angle, label, servo_id: events.append(('servo', servo_id, round(math.degrees(angle))))
    task._count_visible = lambda *a: 5 if a[-1] == 'before' else 0
    task._descend = lambda: events.append('press') or True
    task._return_to_recorded_pose = lambda p: events.append('ascent') or True
    assert task.execute()
    assert events == [('servo',2,270), 'press', ('servo',2,150), 'ascent', ('servo',2,270)]


def test_servo2_failed_descent_does_not_close_or_ascend():
    task = _sea_flow_fake()
    task._gripper_servo_id = 2
    task._release_angle = math.radians(270)
    task._node.set_servo = lambda *a, **kw: None
    task._count_visible = lambda *a: 3
    task._descend = lambda: False
    task._close_after_press = lambda deadline: pytest.fail('failed press must not close')
    task._return_to_recorded_pose = lambda p: pytest.fail('failed press must not ascend')
    assert not task.execute()


def test_execute_retries_when_ball_remains_after_return():
    events = []
    verification_results = iter((False, True))
    task = GrabBallTask.__new__(GrabBallTask)
    task._logger = _Logger()
    task._node = SimpleNamespace(stopped=False)
    task._max_grab_retries = 1
    task._color = 'red'
    task._class_id = 7
    task._descent_speed = 0.4
    task._descent_duration = 10.0

    def servo():
        events.append('servo')
        return [1.0, 2.0, -0.3, 15.0]

    def offset():
        events.append('offset')
        return True

    def settle():
        events.append('settle')
        return True

    def descend():
        events.append('descend')
        return True

    def return_pose(_pose):
        events.append('return')
        return True

    def verify():
        events.append('verify')
        return next(verification_results)

    task._servo_horizontally = servo
    task._apply_gripper_offset = offset
    task._wait_pre_descent_settle = settle
    task._descend = descend
    task._return_to_recorded_pose = return_pose
    task._verify_ball_removed = verify

    assert task.execute()
    assert events == [
        'servo', 'offset', 'settle', 'descend', 'return', 'verify',
        'servo', 'offset', 'settle', 'descend', 'return', 'verify',
    ]


def test_sea_cucumber_ascent_uses_direct_slow_velocity():
    commands = []
    node = SimpleNamespace(
        stopped=False,
        _perception_lock=threading.RLock(),
        _robot_pose=(1.0, 2.0, 0.50, 0.0, 0.0, 0.0),
        _format_motion_context=lambda label: label,
        _cmd_x=1.0, _cmd_y=2.0, _cmd_z=0.50, _cmd_yaw=0.0,
    )

    def send_goal(command, target, axes, **_kwargs):
        commands.append((command, list(target), axes))
        if command == _grab.BasicMotion.Goal.BMOVE:
            pose = list(node._robot_pose)
            pose[2] += target[2]
            node._robot_pose = tuple(pose)
        return True, 'ok'

    node._send_action_goal = send_goal
    velocities = []
    def publish(vertical_mps=0.):
        velocities.append(vertical_mps)
        if vertical_mps < 0:
            pose = list(node._robot_pose)
            pose[2] -= .03  # 模拟艇体到达下一次测量深度。
            node._robot_pose = tuple(pose)
    node._publish_body_velocity = publish
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._node = node
    task._logger = _Logger()
    task._return_timeout = 5.0
    task._ascent_step = 0.03
    task._ascent_step_timeout = 0.5
    task._ascent_pause = 0.0
    task._ascent_tolerance = 0.015
    task._ascent_speed, task._ascent_period = .01, .001

    assert task._return_to_recorded_pose((1.1, 2.1, 0.40, 5.0))
    assert len(velocities) >= 4
    assert all(-.01 <= v <= 0 for v in velocities)
    assert velocities[-1] == 0.
    assert len(commands) == 1  # 上浮不再发送任何BMOVE。
    assert commands[-1][0] == _grab.BasicMotion.Goal.SET
    assert commands[-1][2] == 'xyrz'
    assert commands[-1][1][2] == node._robot_pose[2]


def test_sea_cucumber_ascent_stops_if_measured_depth_does_not_change():
    commands = []
    node = SimpleNamespace(
        stopped=False,
        _perception_lock=threading.RLock(),
        _robot_pose=(1.0, 2.0, 0.50, 0.0, 0.0, 0.0),
        _format_motion_context=lambda label: label,
        _send_action_goal=lambda command, target, axes, **_kwargs:
            (commands.append((command, target, axes)) or True, 'ok'),
    )
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._node = node
    task._logger = _Logger()
    task._return_timeout = 0.2
    task._ascent_step = 0.03
    task._ascent_step_timeout = 0.05
    task._ascent_pause = 0.0
    task._ascent_tolerance = 0.015
    task._ascent_speed, task._ascent_period = .01, .02
    velocities = []
    node._publish_body_velocity = lambda vertical_mps=0.: velocities.append(vertical_mps)

    assert not task._return_to_recorded_pose((1.0, 2.0, 0.40, 0.0))
    assert commands == []
    assert velocities and velocities[-1] == 0.
    assert all(-.01 <= v <= 0 for v in velocities)


def test_sea_cucumber_delivery_ascent_precedes_horizontal_travel():
    commands = []
    node = SimpleNamespace(
        stopped=False,
        _perception_lock=threading.RLock(),
        _robot_pose=(1.0, 2.0, 0.50, 0.0, 0.0, 0.0),
        _format_motion_context=lambda label: label,
        _cmd_x=1.0, _cmd_y=2.0, _cmd_z=0.50, _cmd_yaw=0.0,
    )

    def send_goal(command, target, axes, **_kwargs):
        commands.append((command, list(target), axes))
        if command == _grab.BasicMotion.Goal.SET:
            node._robot_pose = (target[0], target[1], node._robot_pose[2],
                                0., 0., target[3])
        if command == _grab.BasicMotion.Goal.BMOVE:
            pose = list(node._robot_pose)
            pose[2] += target[2]
            node._robot_pose = tuple(pose)
        return True, 'ok'

    node._send_action_goal = send_goal
    velocities = []
    def publish(vertical_mps=0.):
        velocities.append(vertical_mps)
        if vertical_mps < 0:
            pose = list(node._robot_pose)
            pose[2] -= .03
            node._robot_pose = tuple(pose)
    node._publish_body_velocity = publish
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._node = node
    task._logger = _Logger()
    task._ascent_step = 0.03
    task._ascent_step_timeout = 0.5
    task._ascent_pause = 0.0
    task._ascent_tolerance = 0.015
    task._ascent_speed, task._ascent_period = .01, .001

    assert task._travel((3.0, 4.0, 0.40, 10.0), '投放',
                        time.monotonic() + 5.0, 5.0)
    assert len(commands) == 2
    assert velocities and velocities[-1] == 0.
    assert commands[0][0] == _grab.BasicMotion.Goal.WTRAVEL
    assert commands[-1][0] == _grab.BasicMotion.Goal.SET
    assert commands[-1][2] == 'xyrz'
    assert commands[-1][1][3] == 10.0
    assert commands[-1][1][2] >= node._robot_pose[2]


def test_sea_delivery_returns_to_configured_search_before_next_scan():
    task = _sea_flow_fake()
    task._drop_pose = (9., 8., .2, 90.)
    task._drop_timeout = 7.
    task._search_travel_timeout = 11.
    task._release_angle, task._release_wait = 1.57, 0.
    calls = []
    task._travel = lambda pose, label, deadline, timeout: calls.append(
        (pose, timeout)) or True
    assert task._deliver(task._search_pose, time.monotonic() + 2., True)
    assert calls == [(task._drop_pose, 7.), (task._search_pose, 11.)]


def test_sea_delivery_return_failure_is_not_scan_success():
    task = _sea_flow_fake()
    task._drop_pose = (9., 8., .2, 90.)
    task._drop_timeout = 7.
    task._release_angle, task._release_wait = 1.57, 0.
    task._travel = lambda pose, *args: pose == task._drop_pose
    assert not task._deliver(task._search_pose, time.monotonic() + 2., True)


@pytest.mark.parametrize('fallback_success', [True, False])
def test_collection_alignment_failure_travels_back_before_release(fallback_success):
    task = _sea_flow_fake()
    task._drop_pose, task._drop_timeout = (9., 8., .2, 90.), 7.
    task._release_angle, task._release_wait = 1.57, 0.
    task._collection_align = True
    events = []
    deadline = time.monotonic()+20.
    def align(budget):
        assert budget < deadline  # 为兜底留出预算。
        events.append('align failed')
        return False
    task._align_collection = align
    def travel(pose, label, *args):
        events.append(('travel', pose))
        return fallback_success if '兜底' in label else True
    task._travel = travel
    task._command_gripper = lambda *args: events.append('release')
    assert task._deliver(task._search_pose, deadline, True) is fallback_success
    assert events[:3] == [('travel', task._drop_pose), 'align failed', ('travel', task._drop_pose)]
    if fallback_success:
        assert events[3:] == ['release', ('travel', task._search_pose)]
        assert task.delivery_commands == 1
    else:
        assert 'release' not in events and task.delivery_commands == 0


@pytest.mark.parametrize('stopped', [True, False])
def test_collection_fallback_does_not_release_on_stop_or_total_timeout(monkeypatch, stopped):
    clock = [0.]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    task = _sea_flow_fake()
    task._drop_pose, task._drop_timeout = (9., 8., .2, 90.), 7.
    task._release_angle, task._release_wait, task._collection_align = 1.57, 0., True
    events = []
    task._travel = lambda *args: events.append('travel') or True
    task._command_gripper = lambda *args: events.append('release')
    def align(budget):
        task._node.stopped = stopped
        clock[0] = 0. if stopped else 11.
        return False
    task._align_collection = align
    assert not task._deliver(task._search_pose, 10., True)
    assert events == ['travel']


def test_sea_travel_does_not_trust_success_when_measured_pose_is_wrong():
    task = _sea_flow_fake()
    del task._travel
    task._ascent_tolerance = .01
    task._node._send_action_goal = lambda *args, **kwargs: (True, 'ok')
    task._node._format_motion_context = lambda label: label
    # 动作声称成功，但遥测一直在原地：不能在投放区继续扫描。
    assert not task._travel((3., 4., .8, 90.), '返回搜索区',
                            time.monotonic() + .06, .06)
