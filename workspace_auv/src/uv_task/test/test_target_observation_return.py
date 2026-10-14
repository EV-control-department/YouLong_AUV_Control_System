"""Per-target observation return points for loss recovery and post-grab CHECK."""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest
from test_26rb_grab_ball_ring import rig, mod
from test_pickup_windows import stream


@pytest.mark.parametrize('kind,cid',[('golf',7),('ring',8)])
def test_loss_return_moves_toward_best_target_observation_instead_of_frame(rig,monkeypatch,kind,cid):
    node,task=rig
    controller=task.ball if kind=='golf' else task.ring
    node.pose[2]=task.work_depth
    frame=(3.,-3.,task.work_depth,90.)
    task.state.update_pickup(last_servo_pose=frame)
    task.target_loss_timeout=task.frame_loss_timeout=.15
    task.servo_timeout=2.
    records=[];returns=[];jumped=[]
    started=node.clock
    project=mod.target_world_xy
    def remember(node,camera,det,pose,depth):
        world=project(node,camera,det,pose,depth)
        error=mod.normalized_image_error(node,camera,det)
        records.append((math.hypot(*error),node.clock,tuple(pose)))
        return world
    monkeypatch.setattr(mod,'target_world_xy',remember)
    def images():
        elapsed=node.clock-started
        if elapsed>=.3 and not jumped:
            node.pose[:2]=[1.5,-.8];jumped.append(True)
        stream(node,(cid,) if elapsed<.6 else (),
               error=(.08,.03) if elapsed<.3 else (.22,.10))
    node.hook=images;images()
    original=node._send_body_velocity
    def send(*args,**kw):
        if args and any('进入return' in line for line in node.logs):
            returns.append((node.pose.copy(),mod.body_to_world_rotation(node.pose)@np.asarray(args)))
        return original(*args,**kw)
    node._send_body_velocity=send
    assert task._align_target(kind,controller,started)=='ready'
    best=[r for r in records if r[0]<.1][-1][2]
    assert returns
    pose,velocity=returns[0]
    delta=np.asarray(best[:2])-pose[:2]
    assert np.dot(velocity[:2],delta)>0.
    assert np.dot(velocity[:2],np.asarray(frame[:2])-pose[:2])<0.
    assert task._fallback_pose(kind)==pytest.approx((*best[:2],task.work_depth,best[5]))
    assert getattr(task.state.pickup,kind+'_camera_pose')==pytest.approx(
        (*best[:2],task.work_depth,best[5]))
    assert any('返回本轮target最居中观测点' in line for line in node.logs)


@pytest.mark.parametrize('kind,cid',[('golf',7),('ring',8)])
def test_post_grab_return_uses_best_observation_even_if_ready_pose_is_later(rig,monkeypatch,kind,cid):
    node,task=rig
    controller=task.ball if kind=='golf' else task.ring
    node.pose[2]=task.work_depth
    original_pose=tuple(node.pose)
    started=node.clock;jumped=[]
    def images():
        elapsed=node.clock-started
        if elapsed>=.3 and not jumped:
            node.pose[:2]=[1.,.5];node.pose[5]=35.;jumped.append(True)
        stream(node,(cid,),error=(.005,0.) if elapsed<.3 else (.03,0.))
    node.hook=images;images()
    assert task._align_target(kind,controller,started)=='ready'
    assert task._targets[kind][1][:2]==pytest.approx((1.,.5))
    assert getattr(task.state.pickup,kind+'_camera_pose')==pytest.approx(
        (*original_pose[:2],task.work_depth,original_pose[5]))
    node.pose[2]=.44
    node.calls.clear()
    task._return_frame(controller)
    actions=[c for c in node.calls if c[0]=='action']
    assert [c[3] for c in actions]==['z','xyrz']
    assert actions[-1][2]==pytest.approx((*original_pose[:2],task.work_depth,original_pose[5]))
    assert node.pose[2]==task.work_depth


@pytest.mark.parametrize('recorded',[None,(-1.,1.,.05,30.)])
def test_no_target_observation_retains_original_frame_fallback(rig,recorded):
    node,task=rig
    task.state.update_pickup(last_servo_pose=recorded)
    task.p['collection_frame_position']=[-2.,2.,.6]
    expected=recorded or (-2.,2.,task.work_depth,node.pose[5])
    assert task._fallback_pose('golf')==expected
    assert task._fallback_pose('ring')==expected


def test_ball_and_ring_observation_points_do_not_mix(rig):
    node,task=rig
    node.pose[2]=task.work_depth
    node.hook=lambda:stream(node,(7,))
    node.hook()
    assert task._align_target('golf',task.ball,node.clock)=='ready'
    ball_pose=task._fallback_pose('golf')
    frame=(3.,-3.,task.work_depth,80.)
    task.state.update_pickup(last_servo_pose=frame)
    assert task._fallback_pose('ring')==frame
    node.pose[:2]=[1.5,2.]
    node.hook=lambda:stream(node,(8,))
    node.hook()
    assert task._align_target('ring',task.ring,node.clock)=='ready'
    assert task._fallback_pose('golf')==ball_pose
    assert task._fallback_pose('ring')[:2]==pytest.approx((1.5,2.))


def test_new_attempt_clears_old_target_return_point(rig):
    node,task=rig
    node.pose[2]=task.work_depth
    node.hook=lambda:stream(node,(7,))
    node.hook()
    assert task._align_target('golf',task.ball,node.clock)=='ready'
    frame=(3.,-3.,task.work_depth,80.)
    task.state.update_pickup(last_servo_pose=frame)
    task.servo_timeout=.5;task.frame_loss_timeout=.1
    node.hook=lambda:stream(node)
    node.sleep(.01)  # Start strictly after the previous attempt's cached frames.
    assert task._align_target('golf',task.ball,node.clock)=='unobserved'
    assert task._fallback_pose('golf')!=task.state.pickup.golf_camera_pose
    assert 'golf' not in task._target_return_poses


def test_new_task_clears_all_observation_return_history(rig,monkeypatch):
    node,task=rig
    task._target_return_poses.update(golf=(1.,2.,.05,0.),ring=(2.,3.,.05,0.))
    monkeypatch.setattr(mod,'CollectionFrameSearch',lambda *a:NS(
        execute=lambda:mod.TaskOutcome.failed('search.failed','stop before motion')))
    assert not task.execute()
    assert task._target_return_poses=={}
