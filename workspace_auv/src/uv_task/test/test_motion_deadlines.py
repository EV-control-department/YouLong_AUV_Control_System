"""Offline boundary checks for real task/action waits, with delayed DDS futures."""
import ast
import math
import time
from importlib import import_module
from pathlib import Path
from types import MethodType, SimpleNamespace as NS

import numpy as np
import pytest
from uv_msgs.action import BasicMotion
from uv_task.collection_frame_search import FrontDownSearch, SearchFailure
from uv_task.basic_motion_test import RosMotionTest, MotionTestError
from uv_task.search_clock import BlineSearchClock
from test_gate_action_handoff import Future, method
from test_gate_fixed_pass import Task as Gate, GateFailure, Clock as GateClock, Node as GateNode
from test_26rb_grab_ball_ring import rig, mod as pickup


@pytest.fixture(autouse=True)
def ros_available(monkeypatch):
    import rclpy
    monkeypatch.setattr(rclpy, 'ok', lambda: True)


class Clock:
    def __init__(self, limit=400.):
        self.t = 0.
        self.events = []
        self.limit = limit
    def now(self):
        return self.t
    def sleep(self, dt):
        self.t += dt
        if self.t > self.limit:
            raise RuntimeError('unbounded wait')
        for when, callback in list(self.events):
            if self.t >= when:
                self.events.remove((when, callback))
                callback()
    def at(self, when, callback):
        self.events.append((when, callback))
        self.sleep(0.)


class Client:
    def __init__(self, clock, accept_at=0., result_at=.1, success=True,
                 message='', cancel_terminal=True):
        self.clock = clock
        self.goals = []
        self.cancelled = []
        self.ack = Future()
        self.result = Future()
        self.terminal = NS(status=4 if success else 6,
                           result=NS(success=success, message=message,
                                     final_target=[0., 0., .05, 90.]))
        def cancel():
            self.cancelled.append(clock.t)
            if cancel_terminal and not self.result.done():
                self.result.resolve(NS(status=5, result=NS(success=False,
                    message='interrupted by cancellation', final_target=[])))
        self.handle = NS(accepted=True, get_result_async=lambda: self.result,
                         cancel_goal_async=cancel)
        if accept_at is not None:
            clock.at(accept_at, lambda: self.ack.resolve(self.handle))
        if result_at is not None:
            clock.at(result_at, lambda: self.result.resolve(self.terminal))
    def wait_for_server(self, **kwargs):
        return True
    def send_goal_async(self, goal, **kwargs):
        self.goals.append(goal)
        return self.ack


def runner(clock, client, stopped=False):
    ns = dict(BasicMotion=BasicMotion, math=math, rclpy=NS(ok=lambda: True),
              time=NS(monotonic=clock.now, sleep=clock.sleep))
    logger = NS(**{k: lambda *a: None for k in ('info','warn','warning','error')})
    node = NS(stopped=stopped, _debug_timeout=0., _active_goal_handle=None,
              _pending_goal_send_future=None, _action_client=client,
              LIGHT_YELLOW=1, _set_task_phase_light=lambda *a, **k: None,
              get_logger=lambda: logger, _format_motion_context=lambda p: p,
              _default_motion_purpose=lambda *a: 'test')
    for name in ('_confirm_previous_motion', '_send_action_goal', '_send_body_velocity'):
        setattr(node, name, MethodType(method(name, ns), node))
    return node


@pytest.mark.parametrize('command', [3,4,5,8,9,11])
def test_position_line_and_thrust_delivered_after_execution_boundary_succeed(command):
    clock = Clock()
    client = Client(clock, result_at=.22)
    node = runner(clock, client)
    assert node._send_action_goal(command, [0.,0.,.05,90.], 'xyzrz',
                                 timeout=.1, wait_deadline=.1, quiet=True)[0]
    assert client.goals[0].timeout == pytest.approx(.1)
    assert not client.cancelled
    assert clock.t == pytest.approx(.22)


def test_controller_timeout_result_keeps_its_actual_reason():
    clock = Clock()
    client = Client(clock, result_at=.22, success=False,
                    message='motion timeout: yaw error=12deg')
    node = runner(clock, client)
    ok, message = node._send_action_goal(3, [0.,0.,0.,90.], 'xyzrz',
                                        timeout=.1, wait_deadline=.1, quiet=True)
    assert not ok and message == 'motion timeout: yaw error=12deg'
    assert not client.cancelled and node._last_motion_cleanup_confirmed


@pytest.mark.parametrize('command', [3,7])
def test_expired_deadline_never_submits_a_new_goal(command):
    clock = Clock(); client = Client(clock); node = runner(clock, client)
    assert not node._send_action_goal(command, [0.,0.,0.,0.], timeout=5.,
                                      wait_deadline=0., quiet=True)[0]
    assert not client.goals


def test_unresponsive_acceptance_is_bounded_and_late_goal_is_cancelled():
    clock = Clock(limit=5.); client = Client(clock, accept_at=None, result_at=None)
    node = runner(clock, client)
    assert not node._send_action_goal(3, [0.,0.,0.,0.], timeout=60., quiet=True)[0]
    assert 4. <= clock.t < 4.03
    assert node.stopped and node._pending_goal_send_future is client.ack
    client.ack.resolve(client.handle)
    assert client.cancelled
    assert not node._last_motion_cleanup_confirmed


@pytest.mark.parametrize('command,timeout,expected', [(3,.1,2.1),(7,0.,2.),(8,0.,302.)])
def test_missing_client_deadline_cannot_wait_forever(command,timeout,expected):
    clock = Clock(limit=expected+1.)
    client = Client(clock, result_at=None)
    node = runner(clock, client)
    ok, message = node._send_action_goal(command, [0.,0.,0.,0.], timeout=timeout,
                                        cancel_wait_timeout=2., quiet=True)
    assert not ok and '等待动作结果超时' in message
    assert expected <= clock.t < expected+.02
    assert client.cancelled and node._last_motion_cleanup_confirmed


def test_stopped_task_still_waits_for_and_confirms_neutral_velocity():
    clock = Clock(); client = Client(clock, accept_at=.04, result_at=.12)
    node = runner(clock, client, stopped=True)
    assert node._send_body_velocity(wait_deadline=2.)[0]
    assert clock.t == pytest.approx(.13, abs=.011)
    assert not client.cancelled
    assert node._last_motion_cleanup_confirmed
    assert not node._send_body_velocity(forward_mps=.1)[0]
    assert len(client.goals) == 1


def test_unconfirmed_cancel_preserves_old_handle_and_blocks_next_position():
    clock = Clock(limit=10.); client = Client(clock, result_at=None, cancel_terminal=False)
    node = runner(clock, client)
    assert not node._send_action_goal(3,[0.,0.,0.,0.], timeout=.1,
                                      wait_deadline=.1,quiet=True)[0]
    assert node.stopped and node._active_goal_handle is client.handle
    assert not node._send_action_goal(3,[1.,0.,0.,0.],quiet=True)[0]
    assert len(client.goals) == 1


def test_complete_future_at_poll_boundary_is_not_discarded():
    clock = Clock(); client = Client(clock, result_at=.101)
    node = runner(clock, client)
    assert node._send_body_velocity(wait_deadline=.105, cancel_wait_timeout=2.)[0]
    assert clock.t >= .105 and not client.cancelled


def make_gate():
    clock=GateClock(); node=GateNode(clock)
    task=Gate(node, dict(gate_count=1, fixed_pass_code='1111', observation_poses=[[0.,0.,.2,0.],[0.,1.,.2,0.]], observation_xy_radius_m=[.2,.2]))
    task._now=lambda:clock.t; task._sleep=clock.sleep
    return clock,node,task


def test_gate_last_servo_ack_and_stop_have_independent_budgets():
    clock,node,task=make_gate()
    task._deadline=clock.t+.05; task._phase_deadline=task._deadline
    calls=[]
    def send(*args,**kw):
        sent=clock.t; calls.append((args,kw,sent)); clock.sleep(.12)
        assert kw['wait_deadline'] >= clock.t
        return True,''
    node._send_body_velocity=send
    task._velocity(horizontal=(.02,0.))
    task._neutral()
    assert len(calls)==2
    assert calls[0][1]['lease_s']==pytest.approx(.05)
    assert all(kw['wait_deadline']-sent==pytest.approx(2.) for _,kw,sent in calls)
    with pytest.raises(GateFailure,match='总超时'):
        task._velocity(horizontal=(.02,0.))
    assert len(calls)==2


def test_gate_successful_set_delivery_margin_does_not_fail_local_phase():
    clock,node,task=make_gate()
    deadline=clock.t+.1
    def send(*a,**kw):
        clock.sleep(.22); return True,''
    node._send_action_goal=send
    task._action(BasicMotion.Goal.SET,[0.,0.,.2,90.],'xyzrz',deadline)
    task._deadline=clock.t-.01
    with pytest.raises(GateFailure,match='总超时'):
        task._action(BasicMotion.Goal.SET,[0.,0.,.2,90.],'xyzrz',clock.t+5.)


def make_search(clock, client=None):
    node=runner(clock, client or Client(clock))
    node.LIGHT_OFF=0; node.LIGHT_RED=3; node.LIGHT_GREEN=2; node._latest_robot_pose=lambda **kw:(0.,0.,.2,0.,0.,0.)
    node._cmd_x=node._cmd_y=node._cmd_yaw=0.;node._cmd_z=.2
    task=object.__new__(FrontDownSearch)
    task.node=node; task._now=clock.now; task._sleep=clock.sleep
    task.p=dict(search_fallback_rotate_speed_deg_s=30., search_fallback_rotate_degrees=360.,
                search_fallback_rotate_timeout=.1, search_period=.05,
                search_depth_gain=1.,search_max_vertical_speed_mps=.1,
                search_front_align_seconds=.1,search_yaw_gain=1.,search_max_yaw_rate_deg_s=10.,
                search_yaw_tolerance_deg=2.,search_yaw_stable_seconds=.2,
                search_timeout=.1,search_near_timeout=.1,search_speed_mps=.2,
                search_cancel_timeout=.1,search_move_timeout=1.)
    task._bline_active=task._velocity_active=False
    task.correct_depth=True;task.depth=.2;task.target_label='test';task.task_name='test'
    task._light=lambda *a:None;task._frames=lambda *a:[];task._stereo_center=lambda:None
    task._hold=lambda:None;task._flash=lambda *a:None
    task._search_clock=BlineSearchClock(.1,.1,clock.now)
    task._expired=lambda:False
    return node,task


@pytest.mark.parametrize('phase',['front','rotate'])
def test_search_final_velocity_ack_does_not_become_motion_error(phase):
    clock=Clock();node,task=make_search(clock); calls=[]
    def send(*args,**kw):
        sent=clock.t;calls.append((args,kw,sent));clock.sleep(.12)
        assert kw['wait_deadline'] >= clock.t
        return True,''
    node._send_body_velocity=send
    if phase=='front':
        assert not task._align_front()
    else:
        with pytest.raises(SearchFailure,match='原地旋转搜索超时'):
            task._rotate_for_down_frame()
    assert len(calls)==2 and calls[0][1]['lease_s']==pytest.approx(.1)
    assert all(kw['wait_deadline']-sent==pytest.approx(2.) for _,kw,sent in calls)


@pytest.mark.parametrize('phase',['front','rotate'])
def test_search_real_ack_failure_is_not_hidden_by_phase_timeout(phase):
    clock=Clock();node,task=make_search(clock)
    def send(*args,**kw):
        if args:
            clock.sleep(2.01);return False,'真实应答超时'
        return True,''
    node._send_body_velocity=send
    with pytest.raises(SearchFailure,match='速度.*真实应答超时'):
        task._align_front() if phase=='front' else task._rotate_for_down_frame()


def test_pickup_descent_ack_timeout_does_not_trigger_blind_closure(rig):
    node,task=rig; task.descent_timeout=.1
    original=node._send_body_velocity
    def send(*args,**kw):
        result=original(*args,**kw)
        if args:
            node.clock+=2.01;node.pose_received_at=node.clock
            node._last_motion_failure_kind='timeout'
            return False,'等待动作结果超时'
        return result
    node._send_body_velocity=send
    with pytest.raises(pickup.PickupFailure,match='下压速度指令失败'):
        task._descend_to_depth('ring',task.ring)
    assert np.allclose(node.velocity,0.)


def test_old_golf_vertical_phase_caps_lease_and_ack(monkeypatch):
    module=import_module('uv_task.26rb_grab_golf');clock=Clock();calls=[]
    monkeypatch.setattr(module.time,'monotonic',clock.now)
    monkeypatch.setattr(module.time,'sleep',clock.sleep)
    task=object.__new__(module.RB26GrabGolfTask)
    node=NS(stopped=False,_format_motion_context=lambda x:x)
    def send(*args,**kw):
        calls.append((kw.copy(),clock.t));clock.sleep(.12)
        assert kw.get('wait_deadline',0.) >= clock.t
        return True,''
    node._send_body_velocity=send;task._node=node
    task._task_name='golf';task._vertical_period=.05
    task._logger=NS(info=lambda *a:None,error=lambda *a:None)
    assert task._vertical_motion(.05,.1,'下潜')
    assert len(calls)==2 and calls[0][0]['lease_s']==pytest.approx(.1)
    assert all(kw['wait_deadline']-sent==pytest.approx(2.) for kw,sent in calls)


@pytest.mark.parametrize('kind',['search','hit'])
@pytest.mark.parametrize('failure',['accept','cancel'])
def test_manual_bline_unknown_termination_stops_entire_mission(kind,failure):
    clock=Clock(limit=10.)
    client=Client(clock, accept_at=None if failure=='accept' else 0.,
                  result_at=None,cancel_terminal=False)
    node,search=make_search(clock,client)
    if kind=='search':
        task=search
        call=lambda:task._bline(lambda **kw:'timeout')
        error=SearchFailure
    else:
        module=import_module('uv_task.26rb_hit_balls');error=module.MotionFailure
        task=object.__new__(module.RB26HitBallsTask)
        task._node=node;task._now=clock.now;task._sleep=clock.sleep
        task._params=dict(motion_cancel_timeout=.1,motion_accept_timeout=.1,
                          motion_start_timeout=.1,search_period=.05)
        task._yaw_active=task._bline_active=False;task._name='blue'
        task._light=lambda *a:None
        call=lambda:task._run_bline([1.,0.],.2,.1,'test',monitor=lambda:'timeout')
    with pytest.raises(error):
        call()
    assert node.stopped and not node._last_motion_cleanup_confirmed
    if failure=='accept':
        assert node._pending_goal_send_future is client.ack
        client.ack.resolve(client.handle)
        assert client.cancelled
    else:
        assert node._active_goal_handle is client.handle


@pytest.mark.parametrize('kind',['search','hit'])
def test_manual_bline_checks_previous_goal_before_sending(kind):
    clock=Clock();client=Client(clock);node,task=make_search(clock,client)
    node._confirm_previous_motion=lambda _:False
    if kind=='search':
        with pytest.raises(SearchFailure,match='上一条动作'):
            task._bline(lambda **kw:'timeout')
    else:
        module=import_module('uv_task.26rb_hit_balls')
        hit=object.__new__(module.RB26HitBallsTask)
        hit._node=node;hit._now=clock.now;hit._params=dict(motion_cancel_timeout=.1)
        hit._yaw_active=hit._bline_active=False
        with pytest.raises(module.MotionFailure,match='上一条动作'):
            hit._run_bline([1.,0.],.2,.1,'test')
    assert not client.goals


@pytest.mark.parametrize('failure',['accept','cancel'])
def test_diagnostic_action_retains_unknown_goal_until_terminal(monkeypatch,failure):
    from uv_task import basic_motion_test as module
    clock=Clock(limit=10.)
    client=Client(clock,accept_at=None if failure=='accept' else 0.,
                  result_at=None,cancel_terminal=False)
    node=runner(clock,client)
    monkeypatch.setattr(module.time,'sleep',clock.sleep)
    driver=object.__new__(RosMotionTest)
    driver.node=node;driver.action_type=BasicMotion;driver.now=clock.now
    driver.settings=NS(stage='set');driver.check=lambda:None
    with pytest.raises(MotionTestError):
        driver.send(3,[0.,0.,0.,0.],.1)
    assert node.stopped and not node._last_motion_cleanup_confirmed
    if failure=='accept':
        assert node._pending_goal_send_future is client.ack
        client.ack.resolve(client.handle)
        assert client.cancelled
    else:
        assert node._active_goal_handle is client.handle


@pytest.mark.parametrize('name',['ArrowSurfacer','LineFollower'])
def test_legacy_motion_failure_is_never_ignored(name):
    module=import_module('uv_task.arrow_surfacer' if name=='ArrowSurfacer' else 'uv_task.line_follower')
    task=object.__new__(getattr(module,name))
    task._node=NS(_send_action_goal=lambda *a,**kw:(False,'motion timeout'))
    with pytest.raises(RuntimeError,match='motion timeout'):
        task._motion(3,[0.,0.,0.,0.])


def test_legacy_search_submits_setpoint_and_stops_on_failed_ack():
    clock=Clock();node=runner(clock,Client(clock));calls=[]
    ns=dict(BasicMotion=BasicMotion, math=math,time=NS(sleep=clock.sleep))
    search=method('_search_for_class',ns)
    node.camera_extrinsics={'down':1};node._SEARCH_DIRS=[(1.,0.)]*8
    node._cmd_x=node._cmd_y=node._cmd_z=node._cmd_yaw=0.
    node._best_down_detection=lambda _:None
    def send(*args,**kw):
        calls.append((args,kw));return False,'timeout'
    node._send_action_goal=send
    assert not search(node,1,'test')
    assert len(calls)==1 and calls[0][0][0]==10
    assert node._cmd_x==0.


def test_gate_real_ack_timeout_during_optional_yaw_is_fatal():
    clock,node,task=make_gate()
    task._phase_deadline=clock.t+.1
    def send(*a,**kw):
        clock.sleep(2.01);return False,'真实应答超时'
    node._send_body_velocity=send
    with pytest.raises(GateFailure) as failure:
        task._velocity(yaw_rate=5.)
    assert failure.value.kind=='motion'


def test_null_action_terminal_is_unconfirmed_and_blocks_following_goal():
    clock=Clock();client=Client(clock,result_at=None)
    client.result=NS(done=lambda:True,result=lambda:None)
    node=runner(clock,client)
    assert not node._send_action_goal(3,[0.,0.,0.,0.],timeout=.1,quiet=True)[0]
    assert node.stopped and not node._last_motion_cleanup_confirmed
    assert node._active_goal_handle is client.handle


@pytest.mark.parametrize('failed_stage',[0,1,2,3])
def test_sampler_release_never_runs_after_failed_motion_or_alignment(failed_stage):
    calls=[];servo=[];stage=[0]
    def advance(*a,**kw):
        current=stage[0];stage[0]+=1
        calls.append(current)
        return current!=failed_stage,''
    node=NS(_cmd_x=0.,_cmd_y=0.,_cmd_z=.2,_cmd_yaw=0.,
            ANGLE_RELEASE_SAMPLER=1.,
            _model_mapping=NS(configured_class_id=lambda *a:1),
            get_logger=lambda:NS(info=lambda *a:None,error=lambda *a:None),
            _send_action_goal=advance,
            _align_to_class=lambda *a:advance()[0],
            set_servo=lambda *a:servo.append(a))
    execute=method('_task_release_sampler',dict(BasicMotion=BasicMotion))
    assert not execute(node,{})
    assert not servo and calls==list(range(failed_stage+1))


@pytest.mark.parametrize('pending',[False,True])
def test_stop_service_retains_ownership_for_terminal_confirmation(pending):
    calls=[];handle=NS(accepted=True,cancel_goal_async=lambda:calls.append('cancel'))
    future=Future()
    node=NS(_motion_stop_sent=False,stopped=False,
            _active_goal_handle=None if pending else handle,
            _pending_goal_send_future=future if pending else None,
            _cancel_task_light_animation=lambda **kw:None,
            get_logger=lambda:NS(warning=lambda *a:None))
    method('_stop_active_motion',{})(node)
    assert node.stopped and not node._last_motion_cleanup_confirmed
    if pending:
        assert node._pending_goal_send_future is future
        future.resolve(handle)
    else:
        assert node._active_goal_handle is handle
    assert calls==['cancel']


@pytest.mark.parametrize('task_name',['golf_claw','ring_claw','pre_hit','search_position'])
def test_real_task_set_wrappers_allow_terminal_delivery_margin(monkeypatch,task_name):
    clock=Clock();client=Client(clock,result_at=.22);node,search=make_search(clock,client)
    target=[0.,0.,.05,90.]
    if task_name in ('golf_claw','ring_claw'):
        monkeypatch.setattr(pickup.time,'monotonic',clock.now)
        task=object.__new__(pickup.RB26GrabBallRingTask)
        task.node=node;task.p={}
        task._set_pose(target,task_name,timeout=.1,
                       position_tolerance=.15,yaw_tolerance=10.)
        assert client.goals[0].position_tolerance==pytest.approx(.15)
        assert client.goals[0].yaw_tolerance==pytest.approx(10.)
    elif task_name=='pre_hit':
        module=import_module('uv_task.26rb_hit_balls')
        task=object.__new__(module.RB26HitBallsTask)
        task._node=node;task._now=clock.now;task._bline_active=False;task._name='blue'
        task._set(target,'xyzrz',.1,'撞球前定深定位')
    else:
        search._action(target,'xyzrz','搜索前定位',timeout=.1)
    assert not client.cancelled
    assert client.goals[0].timeout==pytest.approx(.1)


def test_old_alignment_checks_final_arrival_and_does_not_report_success_on_failure():
    clock=Clock();node=runner(clock,Client(clock));calls=[]
    node._cmd_x=node._cmd_y=node._cmd_z=node._cmd_yaw=0.
    node._ensure_camera_extrinsics=lambda:True
    node._best_down_detection=lambda _:object()
    node._triangulate=lambda _:(.5,.3,0.)
    def send(command,*args,**kw):
        calls.append(command);return (command==10,'motion timeout' if command==3 else '')
    node._send_action_goal=send
    align=method('_align_to_class',dict(BasicMotion=BasicMotion,time=NS(sleep=clock.sleep)))
    assert not align(node,1,'test')
    assert calls==[10]*200+[3]


def test_hit_bline_never_submits_after_server_discovery_consumes_phase_budget():
    module=import_module('uv_task.26rb_hit_balls');clock=Clock();client=Client(clock)
    node=runner(clock,client)
    def discovery(**kw):
        clock.sleep(.2);return True
    client.wait_for_server=discovery
    task=object.__new__(module.RB26HitBallsTask)
    task._node=node;task._now=clock.now;task._params=dict(motion_cancel_timeout=.1)
    task._yaw_active=task._bline_active=False
    with pytest.raises(module.MotionFailure,match='未发送目标'):
        task._run_bline([1.,0.],.2,.1,'test')
    assert not client.goals
