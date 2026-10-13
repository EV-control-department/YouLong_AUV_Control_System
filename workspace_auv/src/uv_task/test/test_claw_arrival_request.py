"""Verify that pickup arrival overrides reach the actual action request."""
from types import SimpleNamespace as NS
import pytest
from test_gate_action_handoff import Future, method
from uv_msgs.action import BasicMotion


@pytest.mark.parametrize('old_interface', [False, True])
def test_claw_arrival_override_is_transmitted_or_explicitly_refused(old_interface):
    class OldGoal:
        SET, START, BODY_VELOCITY = 3, 6, 7
    motion = NS(Goal=OldGoal) if old_interface else BasicMotion
    clock = [0.]
    namespace = {'BasicMotion': motion, 'rclpy': NS(ok=lambda: True),
                 'time': NS(monotonic=lambda: clock[0], sleep=lambda dt: clock.__setitem__(0, clock[0]+dt))}
    send = method('_send_action_goal', namespace)
    requests = []
    result = NS(result=NS(success=True, final_target=[1., 2., .05, 90.], message=''))
    handle = NS(accepted=True, get_result_async=lambda: Future(result))
    def dispatch(goal):
        requests.append(goal)
        return Future(handle)
    logger = NS(info=lambda *_: None, error=lambda *_: None, warning=lambda *_: None)
    node = NS(stopped=False, _debug_timeout=0., _active_goal_handle=None,
              _pending_goal_send_future=None, LIGHT_YELLOW=3,
              _confirm_previous_motion=lambda timeout: True,
              _action_client=NS(wait_for_server=lambda **_: True, send_goal_async=dispatch),
              _set_task_phase_light=lambda *a, **k: None, get_logger=lambda: logger)
    ok, message = send(node, 3, [1., 2., .05, 90.], 'xyzrz', timeout=30.,
                       wait_deadline=30., quiet=True, task_context='抓环对爪',
                       position_tolerance=.15, yaw_tolerance=10.)
    if old_interface:
        assert not ok and not requests
        assert '重新构建uv_msgs' in message
    else:
        assert ok and len(requests) == 1
        assert requests[0].position_tolerance == pytest.approx(.15)
        assert requests[0].yaw_tolerance == 10.
        assert requests[0].timeout == 30.



def test_new_arrival_fields_round_trip_through_ros_serialization():
    from rclpy.serialization import serialize_message, deserialize_message
    goal = BasicMotion.Goal(cmd_type=BasicMotion.Goal.SET, axes='xyzrz',
                            target=[.25, 5., .05, 90.07], timeout=30.,
                            position_tolerance=.15, yaw_tolerance=10.)
    restored = deserialize_message(serialize_message(goal), BasicMotion.Goal)
    assert restored.position_tolerance == pytest.approx(.15)
    assert restored.yaw_tolerance == 10.
    assert restored.target == goal.target
