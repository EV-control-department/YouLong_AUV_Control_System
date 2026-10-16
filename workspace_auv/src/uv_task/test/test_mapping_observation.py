"""Mapping consumes only compact camera observations, including empty frames."""

import json
from importlib import import_module
from pathlib import Path
from types import SimpleNamespace

import pytest
import numpy as np
from uv_msgs.msg import MappingObservation, MappingObservationArray
from uv_task.config_loader import load_task
from uv_task.mapping_task import MappingTask, StaticPositionFilter


def test_visualizer_diagonal_mirror_is_display_only():
    import ast
    source = Path(__file__).parents[4]/'visualization/mapping_visualizer.py'
    tree = ast.parse(source.read_text())
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef)
                 and n.name in ('display_xy', 'display_heading_deg', 'normalize_display_payload')]
    import copy
    scope = {'np': np, 'copy': copy}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), scope)
    xy = np.asarray([(1-r, c-1) for r in range(3) for c in range(3)], dtype=float)
    old = xy.copy()
    view = scope['display_xy'](xy)
    order = sorted(range(9), key=lambda i: (-view[i,1], view[i,0]))
    assert order == list(range(9))  # 俯视图按从上到下、从左到右为0 1 2 / 3 4 5 / 6 7 8。
    assert np.array_equal(xy, old)
    assert np.array_equal(scope['display_xy'](view), xy)  # 对角镜像自身的逆变换。
    for yaw, expected in [(0.,-90.), (90.,0.), (180.,90.), (-90.,-180.)]:
        assert scope['display_heading_deg'](yaw) == expected
    payload = {'cells': {'8': {'id':8,'position':[1.,2.,3.]}, '0': {'id':0}},
               'conclusion': {'assignment': {'8':0}, 'square_cells':[8]}}
    before = json.dumps(payload)
    normalized = scope['normalize_display_payload'](payload)
    assert [c['id'] for c in normalized['cells']] == [0,8]
    assert normalized['cells'][1]['position'] == [1.,2.,3.]
    assert normalized['conclusion'] == payload['conclusion']
    assert json.dumps(payload) == before
    payload['final_assignment'] = {'8': 0, '0': 1}
    payload['cells']['5'] = {'id':5, 'class_id':0, 'label':'square_cone', 'position':[2.,3.,1.]}
    before = json.dumps(payload)
    normalized = scope['normalize_display_payload'](payload)
    assert [(c['id'],c['class_id']) for c in normalized['cells']] == [(0,1),(5,None),(8,0)]
    assert normalized['cells'][1]['final_selected'] is False
    assert normalized['conclusion'] == payload['conclusion']
    assert json.dumps(payload) == before


def test_mapping_runner_import_does_not_require_unrelated_competition_modules():
    runner = import_module('uv_task.task_runner')
    assert callable(runner.TaskRunnerNode._task_mapping_grid)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Node:
    stopped = False

    def _send_action_goal(self, *_args, **_kwargs):
        return True, 'ok'

    def _format_motion_context(self, label):
        return label

    def _publish_body_thrust(self, **_kwargs):
        pass

    def create_publisher(self, *_args):
        return _Publisher()

    def create_subscription(self, *_args):
        return object()

    def create_timer(self, *_args):
        return object()

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000))

    def get_logger(self):
        return SimpleNamespace(info=lambda *_: None, warning=lambda *_: None,
                               error=lambda *_: None)


@pytest.mark.parametrize('failed_label', [None, 'mapping:travel_to_cell_4',
                                         'mapping:travel_to_cell_0', 'mapping:reverse_cell_5'])
def test_dead_camera_with_healthy_position_attempts_complete_route_and_records_defaults(monkeypatch, failed_label):
    from uv_task import mapping_task as module
    clock = [0.]
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: clock[0],
        sleep=lambda seconds: clock.__setitem__(0, clock[0]+seconds)))
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    params = load_task(Path(__file__).parents[1]/'config/tasks/mapping_grid.json')[0]['params']
    params.update(require_complete_survey=True, enable_traversal=True)
    node = _Node()
    node.stopped = False
    task = MappingTask(node, params)
    monkeypatch.setattr(task, '_position_ready', lambda: True)
    travels = []
    monkeypatch.setattr(task, '_travel_to', lambda xy, label: travels.append(label) or label != failed_label)
    monkeypatch.setattr(task, '_observe_cones', lambda: pytest.fail('无摄像头不等待视觉'))
    assert task.execute()
    assert not node.stopped
    assert task.attempted_visit_order == params['visit_order']
    assert task.attempted_traversal_order == list(reversed(params['visit_order']))
    assert 'mapping:reverse_return_to_april_tag' in travels
    payload = json.loads(task.map_pub.messages[-1].data)
    assert payload['config_fallback_mode'] and not payload['verified_complete']
    assert payload['submitted_config'] == params
    assert payload['conclusion']['source'] == 'fallback'
    assert len(payload['conclusion']['assignment']) == 4
    if failed_label:
        assert task.travel_failures[0]['label'] == failed_label
        if 'travel_to_cell' in failed_label:
            cell = int(failed_label.rsplit('_', 1)[1])
            assert cell not in task.visit_order


@pytest.mark.parametrize('recover', [True,False])
def test_mapping_position_loss_blinks_waits_and_never_stops_chain(monkeypatch, recover):
    from uv_task import mapping_task as module
    clock, zeros, lights = [0.], [], []
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda:clock[0],
        sleep=lambda seconds:clock.__setitem__(0,clock[0]+seconds)))
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    task = MappingTask(_Node(), load_task(Path(__file__).parents[1]/'config/tasks/mapping_grid.json')[0]['params'])
    task.node.stopped = False
    task.deadline = 2.
    task.node._publish_body_thrust = lambda:zeros.append(clock[0])
    task.node.set_light = lambda color, *_:lights.append(color)
    monkeypatch.setattr(task,'_position_ready',lambda:recover and clock[0] >= 1.2)
    if recover:
        task._continue_navigation()
        assert lights[-1] == 3 and clock[0] < task.deadline
    else:
        with pytest.raises(RuntimeError, match='等待定位'):
            task._continue_navigation()
        assert lights[-1] == 1
    assert 1 in lights and 0 in lights and zeros
    assert not task.node.stopped and not task.navigation_paused


def test_camera_degradation_preserves_reliable_visual_and_only_fills_missing_cells():
    params=load_task(Path(__file__).parents[1]/'config/tasks/mapping_grid.json')[0]['params']
    task=MappingTask(_Node(),params)
    task.config_fallback_mode=True
    task.class_votes={6:[10,0],0:[0,10],1:[1,0]}
    retained=task._retained_visual_assignment()
    assert retained == {6:0,0:1}  # 一个孤立候选不能代替已标定的默认目标。
    task.final_assignment=task._fill_fallback_assignment(retained)
    assert task.final_assignment[6] == 0 and task.final_assignment[0] == 1
    assert len(task.final_assignment) == 4
    assert not ({0,6} & task.fallback_cells)
    # 弱观测即使已有滤波坐标，补齐目标也必须使用正确配置中心。
    guessed = next(iter(task.fallback_cells))
    task.filters[guessed] = StaticPositionFilter([9.,9.,9.], np.eye(3), 1.)
    task.filters[6] = StaticPositionFilter([1.,2.,3.], np.eye(3), 1.)
    task.publish_map()
    cells = json.loads(task.map_pub.messages[-1].data)['cells']
    assert cells[guessed]['source'] == 'fallback'
    assert cells[guessed]['position'] == task.grid_centers[guessed].tolist()
    assert cells[guessed]['covariance'] is None
    assert cells[6]['source'] == 'vision' and cells[6]['position'] == [1.,2.,3.]


def test_weak_wrong_class_cannot_displace_reliable_cell_on_camera_degradation():
    params=load_task(Path(__file__).parents[1]/'config/tasks/mapping_grid.json')[0]['params']
    task=MappingTask(_Node(),params)
    task.config_fallback_mode=True
    task.class_votes={0:[100,1],1:[99,0],2:[98,0],3:[0,100]}
    assert task._retained_visual_assignment() == {0:0,1:0,3:1}


def test_disabled_motion_observations_do_not_validate_unused_window():
    params=load_task(Path(__file__).parents[1]/'config/tasks/mapping_grid.json')[0]['params']
    MappingTask(_Node(),dict(params,allow_motion_observations=False,motion_window_seconds=0.))


def test_mapping_fixed_yaw_travel_uses_world_segments(monkeypatch):
    params = load_task(Path(__file__).parents[1]/'config/tasks/mapping_grid.json')[0]['params']
    task = MappingTask(_Node(), params)
    monkeypatch.setattr(task, '_ready', lambda: True)
    calls = []
    task.node._follow_world_waypoints = lambda *args: calls.append(args) or True
    assert task._travel_to([1.,2.], '格点测试')
    assert calls[0][0] == [[1.,2.,params['survey_z'],params['survey_yaw_deg']]]
    assert calls[0][-2:] == (.4, 'fixed_yaw')


def _frame(stamp, *observations, processed=True):
    frame = MappingObservationArray()
    frame.header.stamp.sec = stamp
    frame.processed = processed
    frame.reason = 'ok' if processed else 'camera calibration unavailable'
    frame.observations = list(observations)
    return frame


def test_mapping_associates_and_filters_compact_observations():
    params = {
        'timeout': 60, 'grid_center_x': 2, 'grid_center_y': -4,
        'floor_z': 1.994, 'grid_yaw_deg': 0, 'grid_side_m': 2,
        'tag_id': -1, 'measurement_sigma_m': 0.035,
        'process_noise': 0.0001, 'mahalanobis_gate': 11.345,
        'cell_gate_m': 0.48, 'min_observations': 2,
        'class_vote_ratio': 0.67, 'expected_cones': 4,
    }
    task = MappingTask(_Node(), params)
    task.current_cell = 4
    task.state = 'observe_cell'
    task.observation_start_stamp = 0.0
    task._stable_at = lambda timestamp: True
    task._observation_cb(_frame(1))
    cone = MappingObservation()
    cone.kind = MappingObservation.CONE
    cone.class_id = 1
    cone.confidence = 0.9
    cone.depth_m = 1.5
    cone.depth_samples = 50
    cone.world_x, cone.world_y, cone.world_z = 2.01, -4.02, 1.8
    task._observation_cb(_frame(2, cone))
    cone.world_x = 2.02
    task._observation_cb(_frame(3, cone))
    assert task.cell_observations[4]['synchronized_frames'] == 3
    assert task.filters[4].accepted == 2
    assert task.class_votes[4] == [0, 2]
    assert task.perception_stats['processed_frames'] == 3


def test_rejected_camera_frame_is_not_counted_as_synchronized():
    params = {
        'timeout': 60, 'grid_center_x': 2, 'grid_center_y': -4,
        'floor_z': 1.994, 'grid_yaw_deg': 0, 'grid_side_m': 2,
        'tag_id': -1, 'measurement_sigma_m': 0.035,
        'process_noise': 0.0001, 'mahalanobis_gate': 11.345,
        'cell_gate_m': 0.48, 'min_observations': 2,
        'class_vote_ratio': 0.67, 'expected_cones': 4,
    }
    task = MappingTask(_Node(), params)
    task.current_cell = 4
    task.state = 'observe_cell'
    task.observation_start_stamp = 0.0
    task._stable_at = lambda timestamp: True
    task._observation_cb(_frame(1, processed=False))
    assert task.perception_stats['rejected_frames'] == 1
    assert task.cell_observations == {}


def test_mapping_excludes_travel_delayed_and_unstable_frames():
    from uv_msgs.msg import PoseInfo
    params = load_task(Path(__file__).parents[1] / 'config/tasks/mapping_grid.json')[0]['params']
    params['allow_motion_observations'] = False
    task = MappingTask(_Node(), params)
    task.current_cell = 4
    for index in range(11):
        pose = PoseInfo()
        pose.stamp.sec = 2 + index // 10
        pose.stamp.nanosec = (index % 10) * 100_000_000
        task._pose_cb(pose)
    assert task._stable_at(3.0)


    task.state = 'travel_to_cell'
    task.observation_start_stamp = 0.0
    task._observation_cb(_frame(3))
    assert not task.cell_observations
    assert task.camera_ready  # Readiness must not require accepting moving observations.
    task.state = 'observe_cell'
    task.observation_start_stamp = 2.9
    task._observation_cb(_frame(2))
    assert not task.cell_observations
    task.pose_history[-1].robot_yaw = 20.0
    task._observation_cb(_frame(3))
    assert not task.cell_observations
    task.pose_history[-1].robot_yaw = 0.0
    task._observation_cb(_frame(3))
    assert task.cell_observations[4]['synchronized_frames'] == 1
    task.pose_history[-1].robot_x = 0.1
    assert not task._stable_at(3.0)
    task.pose_history[-1].robot_x = 0.0
    for pose in task.pose_history:
        pose.robot_yaw = 179.9
    task.pose_history[-1].robot_yaw = -179.9
    assert task._stable_at(3.0)


def test_mapping_accepts_low_dynamic_motion_but_rejects_fast_yaw():
    from uv_msgs.msg import PoseInfo
    params = load_task(Path(__file__).parents[1] / 'config/tasks/mapping_grid.json')[0]['params']
    task = MappingTask(_Node(), params)
    task.state = 'travel_to_cell'
    task.observation_start_stamp = 0.0
    for index in range(11):
        pose = PoseInfo()
        pose.stamp.sec = 2 + index // 10
        pose.stamp.nanosec = (index % 10) * 100_000_000
        pose.robot_x = index * 0.01
        pose.robot_yaw = index * 0.5
        task._pose_cb(pose)
    task._observation_cb(_frame(3))
    assert task.perception_stats['processed_frames'] == 1
    task.pose_history[-1].robot_yaw = 30.0
    task._observation_cb(_frame(3))
    assert task.perception_stats['unstable_frames'] == 1


def test_unverified_fallback_keeps_visual_cells_and_marks_guesses():
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    params = load_task(path)[0]['params']
    task = MappingTask(_Node(), params)
    task._assume_tag('标记未识别')
    task.final_assignment = task._fill_fallback_assignment({4: 1})
    task.publish_map()

    assert task.final_assignment == {4: 1, 0: 0, 8: 0, 5: 1}
    assert task.fallback_cells == {0, 8, 5}
    payload = json.loads(task.map_pub.messages[-1].data)
    assert payload['fallback_used'] is True
    assert payload['tag']['id'] == 16
    assert payload['tag']['source'] == 'fallback'
    assert payload['cells'][0]['source'] == 'fallback'
    assert payload['cells'][4]['source'] == 'none'


@pytest.mark.parametrize('enabled', [True, False])
def test_fallback_map_retraces_survey_without_vision_positions(monkeypatch, enabled):
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    params = load_task(path)[0]['params']
    params['enable_traversal'] = enabled
    task = MappingTask(_Node(), params)
    task.perception_stats['processed_frames'] = 1
    task.camera_ready = True
    task.pose_history.append(object())
    monkeypatch.setattr(task, '_position_ready', lambda: True)
    monkeypatch.setattr(task, '_camera_available', lambda: True)
    monkeypatch.setattr('uv_task.mapping_task.rclpy.ok', lambda: True)
    travels = []
    monkeypatch.setattr(task, '_travel_to',
                        lambda position, label: travels.append((list(position), label)) or True)
    monkeypatch.setattr(task, '_read_tag',
                        lambda: task._assume_tag('标记未识别') or True)
    monkeypatch.setattr(task, '_observe_cones', lambda: False)
    monkeypatch.setattr(task, '_select_final_assignment', lambda: {})
    assert task.execute() is True
    assert task.state == 'complete_with_fallback'
    assert len(task.final_assignment) == 4
    reverse = list(reversed(params['visit_order']))
    assert task.traversal_order == (reverse if enabled else [])
    assert len(travels) == (19 if enabled else 10)
    if enabled:
        assert [position for position, _ in travels[10:-1]] == [
            task.grid_centers[cell][:2].tolist() for cell in reverse[1:]]
        assert travels[-1][0] == [params['tag_x'], params['tag_y']]
    assert task.filters == {}  # Neither fused positions nor avoidance circles required.


def test_conclusion_uses_fallback_even_when_visual_assignment_disagrees():
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    task = MappingTask(_Node(), load_task(path)[0]['params'])
    task.publish_map()
    assert json.loads(task.conclusion_pub.messages[-1].data)['ready'] is False
    task.final_assignment = {0: 0, 1: 0, 3: 1, 5: 1}
    task.tag_id = 18
    task.tag_filter = SimpleNamespace(position=np.array([1., 2., 1.]),
                                     covariance=np.eye(3), observations=4)
    task.publish_map()
    conclusion = json.loads(task.conclusion_pub.messages[-1].data)
    assert conclusion['ready'] is True
    assert conclusion['assignment'] == {'0': 0, '8': 0, '5': 1, '6': 1}
    assert conclusion['tag']['id'] == 16
    assert conclusion['verified'] is False
    assert conclusion['source'] == 'fallback'
    payload = json.loads(task.map_pub.messages[-1].data)
    assert payload['tag']['id'] == 18
    assert payload['final_assignment'] == {'0': 0, '1': 0, '3': 1, '5': 1}
    assert payload['conclusion'] == conclusion


def test_invalid_conclusion_does_not_publish_ready_assignment():
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    params = load_task(path)[0]['params']
    params['fallback_round_cells'] = [2, 8]
    task = MappingTask(_Node(), params)
    task.final_assignment = {0: 0, 1: 0, 3: 1, 5: 1}
    task.publish_map()
    conclusion = json.loads(task.conclusion_pub.messages[-1].data)
    assert not conclusion['ready'] and not conclusion['valid']
    assert conclusion['assignment'] == {}


def test_reverse_traversal_continues_after_failed_waypoint(monkeypatch):
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    task = MappingTask(_Node(), load_task(path)[0]['params'])
    task.visit_order = list(task.params['visit_order'])
    monkeypatch.setattr(task, '_ready', lambda: True)
    monkeypatch.setattr(task, '_position_ready', lambda: True)
    travels = []
    monkeypatch.setattr(task, '_travel_to',
                        lambda position, label: travels.append(label) or False)
    task._traverse_cones()
    assert len(travels) == 9
    assert len(task.travel_failures) == 9
    assert task.attempted_traversal_order == list(reversed(task.visit_order))
    assert task.traversal_order == [task.visit_order[-1]]


@pytest.mark.parametrize('interrupted', [False, True])
def test_surface_pulse_before_traversal_clears_and_never_uses_set(monkeypatch, interrupted):
    from uv_task import mapping_task as module
    clock = [0.]
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: clock[0],
        sleep=lambda seconds: clock.__setitem__(0, clock[0]+seconds)))
    params = load_task(Path(__file__).parents[1]/'config/tasks/mapping_grid.json')[0]['params']
    task = MappingTask(_Node(), dict(params, survey_z=.5, traversal_z=0.))
    task.visit_order = list(task.params['visit_order'])
    task.node.stopped = False
    monkeypatch.setattr(task, '_ready', lambda: not task.node.stopped)
    monkeypatch.setattr(task, '_position_ready', lambda: True)
    events = []
    task.node._send_action_goal = lambda *args, **kw: pytest.fail('巡检前上浮不应发送SET')
    def thrust(**kwargs):
        events.append(('thrust', clock[0], kwargs))
        if interrupted and kwargs and clock[0] >= .2:
            task.node.stopped = True
    task.node._publish_body_thrust = thrust
    task.node._follow_world_waypoints = lambda points, *args: events.append(('route', points)) or True
    task.params['travel_mode'] = 'fixed_yaw'
    if interrupted:
        with pytest.raises(RuntimeError, match='停止'):
            task._traverse_cones()
    else:
        task._traverse_cones()
        assert len(task.traversal_order) == 9 and not task.node.stopped
    pulses = [event for event in events if event[0] == 'thrust']
    assert all(event[2] == {'vertical_thrust': -.5} for event in pulses[:-1])
    assert pulses[-1][2] == {}
    assert pulses[-1][1] == pytest.approx(.25 if interrupted else .5)
    routes = [event for event in events if event[0] == 'route']
    assert bool(routes) is (not interrupted)
    assert all(event[1][0][2] == 0. for event in routes)
    assert task.params['survey_z'] == .5  # 原始建图参数不被篡改。


def test_reverse_traversal_requires_completed_first_lap():
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    task = MappingTask(_Node(), load_task(path)[0]['params'])
    task.visit_order = [8, 7]
    with pytest.raises(RuntimeError, match='巡检路线未完整'):
        task._traverse_cones()


def test_traversal_mission_explicitly_enables_second_lap():
    from uv_task.config_loader import load_mission
    root = Path(__file__).parents[1] / 'config/missions'
    assert load_mission(root/'mapping_grid.json')[-1]['params']['enable_traversal'] is True
    assert load_mission(root/'mapping_grid_traversal.json')[-1]['params']['enable_traversal'] is True


def test_new_run_clears_previous_latched_conclusion():
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    params = load_task(path)[0]['params']
    node = _Node()
    first = MappingTask(node, params)
    first.final_assignment = {2: 0, 6: 0, 4: 1, 8: 1}
    first.publish_map()
    assert json.loads(first.conclusion_pub.messages[-1].data)['ready']
    second = MappingTask(node, params)
    second.publish_map()
    assert second.conclusion_pub is first.conclusion_pub
    assert not json.loads(second.conclusion_pub.messages[-1].data)['ready']


def test_no_camera_does_not_move_without_a_completed_survey(monkeypatch):
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    task = MappingTask(_Node(), load_task(path)[0]['params'])
    monkeypatch.setattr(task, '_ready', lambda: False)
    monkeypatch.setattr('uv_task.mapping_task.rclpy.ok', lambda: True)
    monkeypatch.setattr(task, '_travel_to', lambda *_: pytest.fail('未完成首圈不能遍历'))
    assert not task.execute()
    assert task.visit_order == task.traversal_order == []
    conclusion = json.loads(task.conclusion_pub.messages[-1].data)
    assert conclusion['ready'] and not conclusion['verified']
    assert task.state == 'failed'  # 可提交默认结果，不冒充完成检索。


def test_disabled_conclusion_keeps_map_without_creating_result_publisher():
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    params = load_task(path)[0]['params']
    params['enable_conclusion'] = False
    task = MappingTask(_Node(), params)
    task.publish_map()
    assert task.conclusion_pub is None
    payload = json.loads(task.map_pub.messages[-1].data)
    assert payload['conclusion_enabled'] is False
    assert payload['conclusion'] is None
    assert len(payload['cells']) == 9


def test_disabling_conclusion_revokes_old_latched_result_once():
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    params = load_task(path)[0]['params']
    node = _Node()
    first = MappingTask(node, params)
    first.final_assignment = {2: 0, 6: 0, 4: 1, 8: 1}
    first.publish_map()
    count = len(first.conclusion_pub.messages)
    params = dict(params, enable_conclusion=False)
    second = MappingTask(node, params)
    second.publish_map()
    second.publish_map()
    assert len(first.conclusion_pub.messages) == count+1
    revoked = json.loads(first.conclusion_pub.messages[-1].data)
    assert not revoked['enabled'] and not revoked['ready']
    assert revoked['assignment'] == {}
    third = MappingTask(node, dict(params, enable_conclusion=True))
    third.publish_map()
    assert json.loads(first.conclusion_pub.messages[-1].data)['enabled']


def test_full_flow_requires_survey_instead_of_skipping_to_next_task(monkeypatch):
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    task = MappingTask(_Node(), dict(load_task(path)[0]['params'], require_complete_survey=True))
    monkeypatch.setattr(task, '_ready', lambda: False)
    monkeypatch.setattr('uv_task.mapping_task.rclpy.ok', lambda: True)
    assert not task.execute()
    assert not task.node.stopped  # 普通阶段结束不再无条件杀死整条链。
    assert task.state == 'failed'
    conclusion = json.loads(task.conclusion_pub.messages[-1].data)
    assert conclusion['ready'] and not conclusion['verified']


def test_full_flow_mission_orders_stages_under_one_start():
    from uv_task.config_loader import load_mission
    path = Path(__file__).parents[1] / 'config/missions/full_flow_debug.yaml'
    tasks = load_mission(path)
    assert [task['name'] for task in tasks] == [
        'start', 'mapping_grid', 'turntable', 'grab_sea_cucumber', 'return_origin']
    assert tasks[1]['params']['enable_traversal']
    assert tasks[1]['params']['enable_conclusion']
    assert tasks[1]['params']['require_complete_survey']
    assert tasks[2]['params']['mode'] == 'yolo_bump'
    assert 'disk_pose_odom' in tasks[2]['params']
    assert tasks[-1]['params']['timeout'] == 150.
