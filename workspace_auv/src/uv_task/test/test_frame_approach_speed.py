"""Two-speed frame approach with virtual odom and delayed terminal futures."""
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from uv_task import collection_frame_search as mod
from uv_task.config_loader import ConfigError, _validate_params, load_task
from uv_task.golf_search_config import validate_search_params
from test_gate_action_handoff import Future

ROOT = Path(__file__).parents[1]


@pytest.fixture
def setup(monkeypatch):
    monkeypatch.setattr(mod.rclpy, 'ok', lambda: True)
    def build(kind='frame', start_x=0., cancel_latency=.1, stop_delay=.2,
              far_speed=.59, near_speed=.28):
        clock = NS(t=100.)
        pose = np.array([start_x, 0., .2, 0., 0., 0.])
        events, goals = [], []
        client = NS(active=None)
        logs = []
        node = NS(stopped=False, LIGHT_OFF=0, LIGHT_YELLOW=3, LIGHT_GREEN=2, LIGHT_RED=1,
                  _active_goal_handle=None, _pending_goal_send_future=None,
                  _model_mapping=NS(model_class_id=lambda *a, **kw:7),
                  camera_extrinsics={'front_left':object()},
                  _ensure_camera_extrinsics=lambda:True,
                  _latest_robot_pose=lambda:tuple(pose),
                  _format_motion_context=lambda label:label,
                  _set_task_phase_light=lambda *a, **kw:None,
                  _flash_task_light=lambda *a, **kw:True,
                  create_subscription=lambda *a, **kw:object(),
                  destroy_subscription=lambda *a:None,
                  get_logger=lambda:NS(info=logs.append,error=logs.append,warning=logs.append))
        def sleep(dt):
            active = client.active
            if active is not None and not active.result.done():
                moving = dt if active.until is None else min(dt,max(0.,active.until-clock.t))
                pose[0] += active.goal.cruise_speed*moving
            clock.t += dt
            if active is not None and active.until is not None and clock.t >= active.until:
                if not active.result.done():
                    active.result.resolve(NS(status=5,result=NS(success=False,message='cancelled')))
                    events.append(('terminal',clock.t,len(goals)))
            assert clock.t < 130., 'unbounded test wait'
        def send(goal):
            assert client.active is None or client.active.result.done(), 'old line still owns motion'
            result=Future()
            active=NS(goal=goal,result=result,until=None)
            client.active=active
            def cancel():
                events.append(('cancel',clock.t,len(goals)))
                if cancel_latency is not None:
                    active.until=clock.t+cancel_latency
            handle=NS(accepted=True,get_result_async=lambda:result,cancel_goal_async=cancel)
            goals.append(goal);events.append(('send',clock.t,len(goals)))
            return Future(handle)
        client.wait_for_server=lambda **kw:True
        client.send_goal_async=send
        node._action_client=client
        node._confirm_previous_motion=lambda _:client.active is None or client.active.result.done()
        params=dict(search_near_radius_m=.5,search_timeout=10.,search_near_timeout=4.,
                    search_speed_mps=near_speed,search_detection_stop_delay=stop_delay,
                    search_cancel_timeout=.5,search_period=.05)
        if kind=='frame':
            params.update(collection_frame_position=[2.,0.,20.],search_far_speed_mps=far_speed)
            task=mod.CollectionFrameSearch(node,params)
        else:
            params.update(rack_position=[2.,0.,20.])
            task=mod.TargetRackSearch(node,params)
        task._now=lambda:clock.t;task._sleep=sleep
        task._action=lambda *a, **kw:None
        task._hold=lambda:events.append(('hold',clock.t,len(goals)))
        task._observe=lambda prefix,*a,**kw:prefix=='down_'
        task._frames=lambda prefix,*a:([NS(received=clock.t)] if prefix=='down_' and pose[0]>=1.8 else [])
        task._stereo_center=lambda:None
        return NS(task=task,node=node,clock=clock,pose=pose,client=client,goals=goals,events=events)
    return build


def test_full_frame_approach_switches_after_confirmed_cancel_without_resetting_clock(setup):
    rig=setup()
    assert rig.task.execute()
    assert [g.cruise_speed for g in rig.goals]==pytest.approx([.59,.28])
    order=[e[0] for e in rig.events if e[0] in ('send','cancel','terminal')]
    assert order==['send','cancel','terminal','send','cancel','terminal']
    assert rig.goals[0].target[0]==pytest.approx(.59*10.+1.)
    clock=rig.task._search_clock
    assert clock.near_entered_at>2.5
    assert clock.elapsed()==pytest.approx(rig.clock.t-100.)
    assert clock.remaining()==pytest.approx(min(10.-clock.elapsed(),4.-(clock.elapsed()-clock.near_entered_at)))
    assert not rig.node.stopped and rig.node._last_motion_cleanup_confirmed


@pytest.mark.parametrize('start_x',[1.5,1.6])
def test_start_inside_or_on_xy_radius_uses_existing_slow_speed_immediately(setup,start_x):
    rig=setup(start_x=start_x)
    assert rig.task.execute()
    assert [g.cruise_speed for g in rig.goals]==pytest.approx([.28])
    assert rig.task._search_clock.near_entered_at==0.


def test_radius_switch_still_occurs_during_down_detection_stop_delay(setup):
    rig=setup(stop_delay=3.5)
    rig.task._frames=lambda prefix,*a:[NS(received=rig.clock.t)] if prefix=='down_' else []
    assert rig.task.execute()
    assert [g.cruise_speed for g in rig.goals]==pytest.approx([.59,.28])
    assert rig.task.first_down_seen_at==100.


def test_drifting_out_after_first_near_entry_does_not_accelerate_again(setup):
    rig=setup()
    assert rig.task._bline(lambda **kw:'timeout' if rig.task._expired() else None)=='near'
    entered=rig.task._search_clock.near_entered_at
    rig.pose[0]=0.
    assert rig.task._bline(lambda **kw:'down')=='down'
    assert [g.cruise_speed for g in rig.goals]==pytest.approx([.59,.28])
    assert rig.task._search_clock.near_entered_at==entered


def test_unconfirmed_fast_line_cancellation_blocks_slow_line(setup):
    rig=setup(cancel_latency=None)
    assert not rig.task.execute()
    assert len(rig.goals)==1
    assert rig.node.stopped and not rig.node._last_motion_cleanup_confirmed
    assert rig.node._active_goal_handle is not None


def test_equal_configured_speeds_do_not_restart_line_at_radius(setup):
    rig=setup(far_speed=.28)
    assert rig.task.execute()
    assert len(rig.goals)==1


def test_rack_keeps_existing_single_speed_across_radius(setup):
    rig=setup(kind='rack',near_speed=.3)
    assert rig.task.execute()
    assert [g.cruise_speed for g in rig.goals]==pytest.approx([.3])
    assert 'search_far_speed_mps' not in rig.task.p


@pytest.mark.parametrize('file,near_speed',[('26rb_grab_ball_ring.yaml',.28),('26rb_grab_golf.yaml',.15)])
def test_yaml_adds_far_speed_without_changing_near_speed_or_radius(file,near_speed):
    params=load_task(ROOT/'config/tasks'/file)[0]['params']
    assert params['search_far_speed_mps']==.59
    assert params['search_speed_mps']==near_speed
    assert params['search_near_radius_m']==.5


@pytest.mark.parametrize('value',[0.,-.1,True,float('nan'),float('inf'),'fast'])
def test_invalid_far_speed_is_rejected_before_motion(value):
    with pytest.raises(ValueError):
        validate_search_params({'search_far_speed_mps':value})
    with pytest.raises(ConfigError):
        _validate_params('26rb_grab_ball_ring',{'search':{'far_speed_mps':value}})



def test_down_detection_before_radius_still_stops_without_waiting_to_enter(setup):
    rig=setup()
    rig.task._frames=lambda prefix,*a:[NS(received=rig.clock.t)] if prefix=='down_' else []
    assert rig.task.execute()
    assert [goal.cruise_speed for goal in rig.goals]==pytest.approx([.59])
    assert rig.task._search_clock.near_entered_at is None
    assert rig.pose[0]<1.5


def test_radius_is_rechecked_after_server_discovery_before_fast_goal_is_sent(setup):
    rig=setup()
    def server(**kwargs):
        rig.pose[0]=1.6
        return True
    rig.client.wait_for_server=server
    assert rig.task.execute()
    assert [goal.cruise_speed for goal in rig.goals]==pytest.approx([.28])
