"""Simple turntable command sequencing without hardware or ROS nodes."""
from pathlib import Path
from types import SimpleNamespace
import math
import threading

import pytest
from uv_msgs.msg import Detection, DetectionArray
from uv_task.config_loader import load_task, load_mission
from uv_task.simple_turntable_task import SimpleTurntableTask


def setup_task(monkeypatch, **overrides):
    from uv_task import simple_turntable_task as module
    clock = SimpleNamespace(t=1., on_sleep=lambda: None)

    def sleep(dt):
        clock.t += dt
        node._robot_pose_received = clock.t
        clock.on_sleep()

    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: clock.t, sleep=sleep))
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    commands, logs = [], []
    node = SimpleNamespace(stopped=False,
        _perception_lock=threading.Lock(), _robot_pose=(0.,0.,.5,0.,0.,0.), _robot_pose_received=clock.t,
        motion_actions=[],
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock.t*1e9))),
        get_logger=lambda: SimpleNamespace(info=logs.append, error=logs.append),
        create_subscription=lambda *_: object(), destroy_subscription=lambda *_: None,
        _publish_body_velocity=lambda **kwargs: commands.append((clock.t, kwargs)))
    def move(command, target, axes, **kwargs):
        from uv_msgs.action import BasicMotion
        node.motion_actions.append((command, target, axes, kwargs))
        if command == BasicMotion.Goal.BMOVE:
            x,y,z,r,p,yaw = node._robot_pose
            a = math.radians(yaw)
            node._robot_pose = (x+target[0]*math.cos(a), y+target[0]*math.sin(a), z,r,p,yaw)
        return True, ''
    node._send_action_goal = move
    path = Path(__file__).parents[1]/'config/tasks/turntable_simple.yaml'
    params = load_task(path)[0]['params']
    # Unit scenarios have no nominal disk unless explicitly requested; field
    # calibration must not silently enable the preposition/fallback branch.
    params.update(disk_pose_odom=[], front_standoff_m=.55, target_lost_timeout_s=1.)
    params.update(overrides)
    return SimpleTurntableTask(node, params), clock, commands


def box(task, clock, cx=.5, cy=.5, width=.5, class_id=3, stamp=None):
    msg = DetectionArray()
    stamp = int(clock.t*1e9) if stamp is None else stamp
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    d = Detection()
    d.class_id, d.confidence = class_id, .9
    d.bbox_x1, d.bbox_x2 = (cx-width/2)*640, (cx+width/2)*640
    d.bbox_y1, d.bbox_y2 = (cy-.1)*480, (cy+.1)*480
    msg.detections = [d]
    task._on_boxes(msg)


@pytest.mark.parametrize('seen_before_loss', [False, True])
def test_target_loss_moves_to_configured_disk_center(monkeypatch, seen_before_loss):
    task, clock, commands = setup_task(monkeypatch, disk_pose_odom=[1.,2.,.6,90.])
    actions = []
    task.node._send_action_goal = lambda *args, **kwargs: actions.append(args) or (True, '')
    if seen_before_loss:
        clock.on_sleep = lambda: box(task, clock, width=.2) if clock.t < 1.5 else None
    assert task.execute()
    assert len(actions) == 4  # 观察位预定位两步 + 盘心兜底两步。
    assert all(a[1] == [1.,2.,.6,90.] and a[2] == 'xyzrz' for a in actions[-2:])
    assert commands[-1][1] == {}


def test_target_loss_without_disk_center_never_guesses_motion(monkeypatch):
    task, clock, commands = setup_task(monkeypatch, disk_pose_odom=[])
    task.node._send_action_goal = lambda *args, **kwargs: pytest.fail('未填盘心不能猜测')
    assert not task.execute()
    assert all(c == {} for _, c in commands)


def test_fixed_heading_preposition_uses_world_route_not_extra_wtravel(monkeypatch):
    task, clock, commands = setup_task(monkeypatch, disk_pose_odom=[1.,2.,.6,90.], preposition_mode='fixed_yaw')
    task._deadline = clock.t+10.
    routes = []
    task.node._follow_world_waypoints = lambda *args: routes.append(args) or True
    task.node._send_action_goal = lambda *args, **kwargs: pytest.fail('不应追加WTRAVEL转向')
    task._preposition()
    assert routes[0][0][0] == pytest.approx([1.,1.22,.524,90.])
    assert routes[0][-1] == 'fixed_yaw'


@pytest.mark.parametrize('yaw', [0., 90., -90.])
def test_centered_box_forward_move_only_and_final_zero(monkeypatch, yaw):
    from uv_msgs.action import BasicMotion
    task, clock, commands = setup_task(monkeypatch)
    task.node._robot_pose = (0.,0.,.5,0.,0.,yaw)
    clock.on_sleep = lambda: box(task, clock)
    assert task.execute()
    actions = task.node.motion_actions
    assert len(actions) == 1
    assert actions[0][:3] == (BasicMotion.Goal.BMOVE, [.08,0.,0.,0.], 'x')
    assert not any(c.get('vertical_mps') or c.get('yaw_rate_deg_s') for _, c in commands)
    assert commands[-1][1] == {}


def test_short_move_action_success_without_actual_motion_times_out(monkeypatch):
    task, clock, commands = setup_task(monkeypatch, bump_move_timeout_s=.2)
    task.node._send_action_goal = lambda *args, **kw: (True, '')
    clock.on_sleep = lambda: box(task, clock)
    assert not task.execute()
    assert commands[-1][1] == {}


def test_small_centered_box_still_approaches_forward_before_move(monkeypatch):
    task, clock, commands = setup_task(monkeypatch)
    clock.on_sleep = lambda: box(task, clock, width=.2 if clock.t < 1.3 else .5)
    assert task.execute()
    assert any(c.get('forward_mps') == .08 for _,c in commands)
    assert len(task.node.motion_actions) == 1


@pytest.mark.parametrize('flipped,expected_sign', [(True, -1), (False, 1)])
def test_box_alignment_direction_and_no_forward_when_off_center(monkeypatch, flipped, expected_sign):
    task, clock, commands = setup_task(monkeypatch, front_camera_upside_down=flipped,
                                     timeout=.3)
    clock.on_sleep = lambda: box(task, clock, cx=.7, cy=.7, width=.2)
    assert not task.execute()
    align = [c for _, c in commands if 'vertical_mps' in c]
    assert align
    assert all(c['forward_mps'] == 0. for c in align)
    assert all(c['yaw_rate_deg_s']*expected_sign > 0 for c in align)
    assert all(c['vertical_mps']*expected_sign > 0 for c in align)
    assert commands[-1][1] == {}


def test_wrong_class_and_old_frames_never_start_motion(monkeypatch):
    task, clock, commands = setup_task(monkeypatch, observation_timeout_s=.2)
    clock.on_sleep = lambda: box(task, clock, class_id=8)
    assert not task.execute()
    assert all(not c for _, c in commands)
    box(task, clock, stamp=1_000_000_000)
    assert task._latest_box() is None


def test_stop_during_bump_ends_task_without_yaw(monkeypatch):
    task, clock, commands = setup_task(monkeypatch)
    def stop(*args, **kw):
        task.node.stopped = True
        return False, 'cancelled'
    task.node._send_action_goal = stop
    clock.on_sleep = lambda: box(task, clock)
    assert not task.execute()
    assert not any(c.get('yaw_rate_deg_s') for _, c in commands)
    assert commands[-1][1] == {}


def test_target_loss_stops_forward_and_does_not_bump(monkeypatch):
    task, clock, commands = setup_task(monkeypatch, box_max_age_s=.1,
                                     target_lost_timeout_s=.15)

    def tick():
        if clock.t < 1.07:
            box(task, clock, width=.2)

    clock.on_sleep = tick
    assert not task.execute()
    assert any(c.get('forward_mps') for _, c in commands)
    assert not any(c.get('yaw_rate_deg_s') for _, c in commands)
    assert commands[-1][1] == {}
    assert not any(c for t, c in commands if t > 1.2)


def test_simple_mission_and_runner_selection(monkeypatch):
    from uv_task.task_runner import TaskRunnerNode
    path = Path(__file__).parents[1]/'config/missions/turn_table.yaml'
    tasks = load_mission(path)
    assert tasks[-1]['params']['mode'] == 'yolo_bump'
    selected = []
    node = SimpleNamespace(_run_major_task_with_light=lambda cls, p, label:
                           selected.append(cls) or True)
    assert TaskRunnerNode._task_turntable(node, tasks[-1]['params'])
    assert selected == [SimpleTurntableTask]


def test_preinput_disk_center_positions_then_uses_new_boxes(monkeypatch):
    from uv_msgs.action import BasicMotion
    task, clock, commands = setup_task(monkeypatch,
                                     disk_pose_odom=[1., 2., .6, 90.])
    actions = []
    original_send = task.node._send_action_goal

    def send(command, target, axes, timeout, task_context):
        actions.append((command, list(target), timeout))
        if command == BasicMotion.Goal.BMOVE:
            return original_send(command, target, axes, timeout=timeout, task_context=task_context)
        clock.t += 1.
        box(task, clock)  # Frames collected during travel must be discarded.
        return True, 'ok'

    task.node._send_action_goal = send
    clock.on_sleep = lambda: box(task, clock)
    assert task.execute()
    assert [a[0] for a in actions] == [BasicMotion.Goal.WTRAVEL, BasicMotion.Goal.SET, BasicMotion.Goal.BMOVE]
    assert actions[0][1] == pytest.approx([1., 1.22, .524, 90.])
    assert actions[1][1] == pytest.approx(actions[0][1])
    assert all(0 < a[2] <= 90. for a in actions)
    assert task._started_stamp == 3_000_000_000
    assert actions[-1][1] == [.08,0.,0.,0.]


def test_preposition_failure_never_starts_contact(monkeypatch):
    task, clock, commands = setup_task(monkeypatch,
                                     disk_pose_odom=[1., 2., .6, 90.])
    calls = []
    task.node._send_action_goal = lambda *args, **kwargs: (calls.append(args) or False, 'failed')
    assert not task.execute()
    assert len(calls) == 1
    assert all(not c for _, c in commands)


@pytest.mark.parametrize('nominal', [[1., 2.], [1., 2., float('nan'), 0.], [1., 2., .01, 0.]])
def test_invalid_preinput_never_commands_motion(monkeypatch, nominal):
    task, clock, commands = setup_task(monkeypatch, disk_pose_odom=nominal)
    task.node._send_action_goal = lambda *args, **kwargs: pytest.fail('无效盘心不能移动')
    assert not task.execute()
    assert all(not c for _, c in commands)
