"""Return-path perception and acknowledged cancellation before visual handoff."""
import ast
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest
from uv_msgs.action import BasicMotion
from uv_task.grab_sea_cucumber import GrabSeaCucumberTask


def logger():
    return SimpleNamespace(info=lambda m:None, warning=lambda m:None,
                           warn=lambda m:None, error=lambda m:None)


def perception_task(monkeypatch, confidence=.8):
    from uv_task import grab_sea_cucumber as module
    monkeypatch.setattr(module.time,'monotonic',lambda:1.1)
    task=GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._params={'return_observation_confidence':.8}
    task._class_id, task._min_confidence, task._detection_timeout=2,.55,.8
    task._return_observation_started, task._return_capture_after_ns=1.,1_000_000_000
    task._return_last_frame_ns=-1
    task._logger=logger()
    task._measured_pose=lambda:(4.3,-1.1,.13,0.,0.,90.)
    d=SimpleNamespace(class_id=2,confidence=confidence,mask_x=[1.,2.,3.],mask_y=[4.,5.,6.])
    msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=50_000_000)),detections=[d])
    task._node=SimpleNamespace(stopped=False,_perception_lock=threading.Lock(),
        _down_detections={'down_left':(1.05,msg)},
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_100_000_000)))
    return task,msg


@pytest.mark.parametrize('confidence,found',[(.79,False),(.8,True),(.95,True)])
def test_return_threshold_and_duplicate_frame(monkeypatch,confidence,found):
    task,msg=perception_task(monkeypatch,confidence)
    assert task._return_target_visible() is found
    assert not task._return_target_visible()


@pytest.mark.parametrize('problem',['old_capture','stale_message','wrong_class','no_mask'])
def test_return_rejects_noncredible_targets(monkeypatch,problem):
    task,msg=perception_task(monkeypatch)
    if problem=='old_capture':msg.header.stamp.sec=0
    if problem=='stale_message':task._node._down_detections['down_left']=(-1.,msg)
    if problem=='wrong_class':msg.detections[0].class_id=4
    if problem=='no_mask':msg.detections[0].mask_x=[]
    assert not task._return_target_visible()


@pytest.mark.parametrize('found',[False,True])
def test_return_handoff_keeps_actual_xy_and_turns_to_search_yaw(monkeypatch,found):
    task,msg=perception_task(monkeypatch)
    task._ascent_tolerance,task._command_timeout,task._search_travel_timeout=.05,30.,180.
    task._return_target_visible=lambda:found
    task._wait_visual_motion=lambda *a:True
    current=[4.3,-1.1,.13,0.,0.,90.]
    task._measured_pose=lambda:tuple(current)
    actions=[]
    def send(command,target,axes,**kw):
        actions.append((command,list(target),kw))
        if kw.get('interrupt_if') and kw['interrupt_if']():
            return False,'视觉目标触发中断'
        current[0],current[1],current[2],current[5]=target[0],target[1],target[2],target[3]
        return True,'ok'
    task._node._send_action_goal=send
    task._node._format_motion_context=lambda s:s
    assert task._return_observing([4.,-2.,.1,0.],10.)
    assert not task._return_watch_active
    assert task._return_servo_handoff is found
    assert actions[-1][1]==pytest.approx([4.3,-1.1,.13,0.] if found else [4.,-2.,.13,0.])
    assert 'interrupt_if' not in actions[-1][2] if found else True


@pytest.mark.parametrize('acknowledged',[True,False])
def test_action_cancel_waits_for_terminal_result_before_handoff(acknowledged):
    path=Path(__file__).parents[1]/'uv_task/task_runner.py'
    cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='TaskRunnerNode')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_send_action_goal')
    clock=[0.]
    handle=SimpleNamespace(accepted=True,cancels=0)
    def cancel():handle.cancels+=1
    handle.cancel_goal_async=cancel
    result_future=SimpleNamespace(done=lambda:acknowledged and handle.cancels and clock[0]>=.05)
    handle.get_result_async=lambda:result_future
    send_future=SimpleNamespace(done=lambda:True,result=lambda:handle)
    scope={'BasicMotion':BasicMotion,'rclpy':SimpleNamespace(ok=lambda:True),
           'time':SimpleNamespace(monotonic=lambda:clock[0],sleep=lambda dt:clock.__setitem__(0,clock[0]+dt))}
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),scope)
    node=SimpleNamespace(stopped=False,_debug_timeout=-1.,get_logger=logger,
        _format_motion_context=lambda s:s,_default_motion_purpose=lambda *a:'travel',
        _action_client=SimpleNamespace(wait_for_server=lambda **kw:True,send_goal_async=lambda g:send_future))
    success,message=scope['_send_action_goal'](node,BasicMotion.Goal.WTRAVEL,[4.,-2.,.1,0.],interrupt_if=lambda:True)
    assert not success and handle.cancels==1
    if acknowledged:
        assert message=='视觉目标触发中断' and node._active_goal_handle is None
        assert not node.stopped and clock[0]>=.05
    else:
        assert message=='视觉触发取消未确认，禁止接管'
        assert node.stopped and node._active_goal_handle is handle
