"""Cross-trip rack/frame presets from best-centred calibrated down observations."""
from dataclasses import FrozenInstanceError
from pathlib import Path
from types import MethodType, SimpleNamespace as NS

import numpy as np
import pytest

from test_26rb_grab_ball_ring import Node
from test_26rb_drop_ball_target_rack import rig as drop_rig
from uv_task import down_target_memory as memory
from uv_task.collection_frame_search import CollectionFrameSearch, TargetRackSearch
from uv_task.pickup_alignment import target_world_xy
from uv_task.task_outcome import TaskOutcome
from uv_task.task_runner import TaskRunnerNode
from uv_task.task_state import TaskState


@pytest.fixture
def node(monkeypatch):
    node = Node()
    ids = {'collection_frame_down': 0, 'collection_frame_front': 1,
           'target_rack_down': 9, 'target_rack_front': 10,
           'red_ring': 8, 'pink_golf': 7, 'yellow_golf': 11}
    node._model_mapping = NS(model_class_id=lambda name, **kw: ids.get(name))
    monkeypatch.setattr(memory.time, 'monotonic', lambda: node.clock)
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(node.clock*1e9)))
    node._serial = 0
    node.hook = lambda: None
    return node


def box(node, cid, offset=(0., 0.), *, confidence=.9, size=None):
    cfg = node.camera_configs['down']
    width, height = (cfg.width, cfg.height) if size is None else size
    x, y = width/2.+offset[0], height/2.+offset[1]
    return NS(class_id=cid, confidence=confidence, pixel_x=x, pixel_y=y,
              bbox_x1=x-5., bbox_x2=x+5., bbox_y1=y-5., bbox_y2=y+5.)


def send(node, detections, camera='down_left', capture=None, stamp=None):
    node._serial += 1
    stamp = node.clock if stamp is None else stamp
    whole = int(stamp)
    message = NS(camera_name=camera, image_space=0, capture_id=node._serial if capture is None else capture,
                 header=NS(stamp=NS(sec=whole, nanosec=int((stamp-whole)*1e9))), detections=detections)
    TaskRunnerNode._det_cb(node, message)
    return message


def advance(node, seconds=.1):
    node.clock += seconds
    node.pose_received_at = node.clock


def test_global_callback_records_world_target_not_vehicle_position(node):
    det = box(node, 9, (25., -10.))
    send(node, [det])
    observation = node.state.down_target('rack')
    assert observation is not None
    expected = target_world_xy(node, 'down_left', det, tuple(node.pose), .8)
    assert observation.xy == pytest.approx(expected)
    assert observation.xy != pytest.approx(tuple(node.pose[:2]))
    with pytest.raises(FrozenInstanceError):
        observation.camera = 'down_right'


def test_more_centred_beats_newer_and_higher_confidence(node):
    send(node, [box(node, 9, (10., 0.), confidence=.2)])
    first = node.state.down_target('rack')
    advance(node)
    node.pose[0] += 4.
    send(node, [box(node, 9, (100., 0.), confidence=1.)])
    assert node.state.down_target('rack') is first
    advance(node)
    send(node, [box(node, 9, (5., 0.), confidence=.1)])
    assert node.state.down_target('rack').center_distance < first.center_distance
    assert node.state.down_target('rack').xy != first.xy


def test_one_frame_selects_nearest_bbox_not_largest_or_highest_confidence(node):
    near = box(node, 9, (1., 0.), confidence=.2)
    far = box(node, 9, (100., 0.), confidence=.99)
    far.bbox_x1 -= 50.
    far.bbox_x2 += 50.
    send(node, [far, near])
    expected = target_world_xy(node, 'down_left', near, tuple(node.pose), .8)
    assert node.state.down_target('rack').xy == pytest.approx(expected)


def test_bbox_center_is_metric_and_projection_point_not_pixel_field(node):
    det = box(node, 9)
    center = NS(pixel_x=det.pixel_x, pixel_y=det.pixel_y)
    det.pixel_x += 100.
    send(node, [det])
    expected = target_world_xy(node, 'down_left', center, tuple(node.pose), .8, reference=det)
    assert node.state.down_target('rack').center_distance == 0.
    assert node.state.down_target('rack').xy == pytest.approx(expected)


def test_two_eyes_compete_by_center_distance_and_ties_prefer_newer(node):
    send(node, [box(node, 9, (10., 0.))], 'down_left')
    advance(node)
    send(node, [box(node, 9, (2., 0.))], 'down_right')
    best = node.state.down_target('rack')
    assert best.camera == 'down_right'
    advance(node)
    send(node, [box(node, 9, (10., 0.))], 'down_left')
    assert node.state.down_target('rack') is best
    advance(node)
    send(node, [box(node, 9, (2., 0.))], 'down_left')
    assert node.state.down_target('rack').camera == 'down_left'


def test_rack_and_frame_are_independent_and_survive_pickup_reset(node):
    send(node, [box(node, 0, (20., 0.)), box(node, 9, (-30., 0.))])
    rack = node.state.down_target('rack')
    frame = node.state.down_target('frame')
    node.state.reset_pickup()
    assert node.state.down_target('rack') is rack and node.state.down_target('frame') is frame
    advance(node)
    send(node, [box(node, 0)])
    assert node.state.down_target('rack') is rack
    assert node.state.down_target('frame').center_distance == 0.


@pytest.mark.parametrize('capture', [10, 9])
def test_repeated_or_older_frames_do_not_reproject_with_new_vehicle_pose(node, capture):
    send(node, [box(node, 9, (10., 0.))], capture=10)
    best = node.state.down_target('rack')
    advance(node)
    node.pose[:2] += [3., 4.]
    send(node, [box(node, 9)], capture=capture)
    assert node.state.down_target('rack') is best
    memory.poll_down_targets(node)
    assert node.state.down_target('rack') is best


@pytest.mark.parametrize('case', ['empty', 'stale_source', 'old_cache', 'confidence', 'invalid_tf', 'upward_ray', 'stale_odom', 'nav_invalid'])
def test_invalid_or_missing_observations_preserve_good_preset(node, case):
    send(node, [box(node, 9, (15., 0.))])
    saved = node.state.down_target('rack')
    advance(node)
    if case == 'empty':
        send(node, [])
    elif case == 'stale_source':
        send(node, [box(node, 9)], stamp=node.clock-1.)
    elif case == 'old_cache':
        advance(node, 1.)
        memory.poll_down_targets(node)
    elif case == 'confidence':
        send(node, [box(node, 9, confidence=.001)])
    elif case == 'invalid_tf':
        node.camera_extrinsics['down_left'].translation[0] = float('nan')
        send(node, [box(node, 9)])
    elif case == 'upward_ray':
        node.camera_extrinsics['down_left'].optical_to_body[2, 2] = -1.
        send(node, [box(node, 9)])
    elif case == 'stale_odom':
        node.pose_received_at = node.clock-2.
        send(node, [box(node, 9)])
    else:
        node._down_target_pose_valid = False
        message = send(node, [box(node, 9)])
        node._down_target_pose_valid = True
        memory.poll_down_targets(node)
    assert node.state.down_target('rack') is saved


def test_actual_image_dimensions_and_calibration_reference_are_used(node):
    from uv_camera.image_geometry import normalized_pixel
    info = NS(width=1000, height=500, k=[1000.,0.,480.,0.,1000.,240.,0.,0.,1.], d=[0.]*5)
    references = []
    def normalized(used_node, camera, point, reference=None):
        references.append(reference)
        return normalized_pixel(info.k, info.d, point.pixel_x, point.pixel_y)
    node._detection_geometry = NS(bind=lambda message: True, cache=NS(resolve=lambda message: info), normalized=normalized)
    det = box(node, 9, size=(1000, 500))
    send(node, [det])
    assert node.state.down_target('rack').center_distance == 0.
    assert references == [det]


def test_later_searches_use_saved_world_xy_and_keep_configured_depth(node):
    configured_frame = [5., 6., .12]
    configured_rack = [7., 8., -1.]
    before = CollectionFrameSearch(node, {'collection_frame_position': configured_frame})
    assert before.position == pytest.approx(configured_frame)
    send(node, [box(node, 0), box(node, 9, (10., 0.))])
    frame_xy = node.state.down_target('frame').xy
    rack_xy = node.state.down_target('rack').xy
    node.state.reset_pickup()
    frame_search = CollectionFrameSearch(node, {'collection_frame_position': configured_frame})
    rack_search = TargetRackSearch(node, {'rack_position': configured_rack})
    assert frame_search.position == pytest.approx([*frame_xy, .12])
    assert rack_search.position == pytest.approx([*rack_xy, -1.])
    assert configured_frame == [5., 6., .12] and configured_rack == [7., 8., -1.]


def pose_message(node, generation, valid=True):
    return NS(robot_x=node.pose[0], robot_y=node.pose[1], robot_z=node.pose[2],
              robot_roll=0., robot_pitch=0., robot_yaw=node.pose[5],
              origin_generation=generation, nav_valid=valid, origin_initialized=valid)


def test_origin_generation_change_clears_both_points_and_rejects_old_epoch(node):
    TaskRunnerNode._pose_cb(node, pose_message(node, 1))
    send(node, [box(node, 0), box(node, 9)], capture=10)
    epoch = node.state.down_target_epoch
    advance(node)
    TaskRunnerNode._pose_cb(node, pose_message(node, 1))
    assert node.state.down_target('rack') is not None
    advance(node)
    TaskRunnerNode._pose_cb(node, pose_message(node, 2))
    assert node.state.down_target('rack') is None and node.state.down_target('frame') is None
    assert not node.state.remember_down_target('rack', [1., 2.], 0., node.clock, 'down_left', epoch=epoch)
    advance(node)
    send(node, [box(node, 9)], capture=10)
    assert node.state.down_target('rack') is None
    advance(node)
    send(node, [box(node, 9)], capture=11)
    assert node.state.down_target('rack') is not None


def test_successful_start_resets_memory_but_failed_start_preserves_it(node):
    send(node, [box(node, 0), box(node, 9)])
    saved = node.state.down_target('rack')
    original = node._send_action_goal
    node._send_action_goal = lambda *a, **kw: (False, '模拟失败')
    assert not TaskRunnerNode._do_start(node)
    assert node.state.down_target('rack') is saved
    node._send_action_goal = lambda cmd, target, axes='', **kw: original(cmd, target, axes, **kw)
    assert TaskRunnerNode._do_start(node)
    assert node.state.down_target('rack') is None and node.state.down_target('frame') is None


def test_projection_scale_changes_invalidate_old_estimate(node):
    send(node, [box(node, 9)])
    node.state.configure_down_target('rack', 1.2, .8)
    assert node.state.down_target('rack') is None
    advance(node)
    det = box(node, 9)
    send(node, [det])
    assert node.state.down_target('rack').xy == pytest.approx(target_world_xy(node, 'down_left', det, tuple(node.pose), 1.2))


def test_rack_fallback_uses_best_point_even_if_task_was_constructed_earlier(drop_rig, monkeypatch):
    from importlib import import_module
    module = import_module('uv_task.26rb_drop_ball_target_rack')
    rig = drop_rig
    rig.task._params['release_at_preset_on_no_frame'] = True
    rig.node._last_motion_cleanup_confirmed = True
    rig.task._state.remember_down_target('rack', [2., 3.], .01, rig.clock[0], 'down_left')
    monkeypatch.setattr(module, 'TargetRackSearch', lambda *a: NS(execute=lambda:
        TaskOutcome.failed('26rb_drop_ball_target_rack.no_frame', '普通未发现')))
    rig.task._flash_green = lambda *a, **kw: True
    assert rig.task.execute()
    assert rig.task._preset_drop_xy == (2., 3.)
    assert rig.task._rack_position == (2., 3., -1.)


def test_actual_drop_servo_preserves_best_rack_and_frame_for_next_trip(node, monkeypatch):
    from importlib import import_module
    module = import_module('uv_task.26rb_drop_ball_target_rack')
    monkeypatch.setattr(module.time, 'sleep', node.sleep)
    node._target_rack_down_class_id = 9
    task = module.RB26DropBallTargetRackTask(node, {'rack_position': [9., 9., -1.],
        'down_visual_servo_timeout': 1., 'down_visual_servo_stable_seconds': 10.})
    start = node.clock
    best = []
    def images():
        offset = 0. if .3 <= node.clock-start <= .6 else 80.
        send(node, [box(node, 9, (offset, 0.)), box(node, 0, (offset/2., 0.))])
        if offset == 0.:
            best.append((node.state.down_target('rack'), node.state.down_target('frame')))
    node.hook = images
    images()
    assert task._servo_rack()
    assert task._visual_servo_timed_out and best
    saved_rack, saved_frame = best[-1]
    assert node.state.down_target('rack') is saved_rack
    assert node.state.down_target('frame') is saved_frame
    next_drop = module.RB26DropBallTargetRackTask(node, {'rack_position': [9., 9., -1.]})
    next_frame = CollectionFrameSearch(node, {'collection_frame_position': [8., 8., .12]})
    assert next_drop._rack_position == (*saved_rack.xy, -1.)
    assert tuple(next_frame.position) == (*saved_frame.xy, .12)


@pytest.mark.parametrize('other_task', ['', '26rb_gate_task'])
def test_idle_or_other_task_observations_cannot_replace_drop_points_or_replay_later(node, other_task):
    node._current_task_name = '26rb_drop_ball_target_rack'
    send(node, [box(node, 9, (20., 0.))], capture=10)
    saved = node.state.down_target('rack')
    advance(node)
    node._current_task_name = other_task
    send(node, [box(node, 9)], capture=11)
    assert node.state.down_target('rack') is saved
    node._current_task_name = '26rb_grab_ball_ring'
    memory.poll_down_targets(node)
    assert node.state.down_target('rack') is saved
    advance(node)
    send(node, [box(node, 9)], capture=12)
    assert node.state.down_target('rack').center_distance == 0.
