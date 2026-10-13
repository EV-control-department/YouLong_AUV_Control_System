from types import SimpleNamespace as NS
import pytest
from test_start_origin import node, Goal
from uv_control.coordinate import Coordinate

@pytest.mark.parametrize('requested,target,expected', [(0,(0,0,0),10),(-1,(.3,0,0),10),(0,(3,4,0),5/.18+5),(0,(0,0,1.8),15),(7,(3,4,0),7)])
def test_budget(node, requested, target, expected):
    start=Coordinate(x=1,y=2,z=3)
    assert node._position_action_timeout(requested,start,*(a+b for a,b in zip((1,2,3),target))) == pytest.approx(expected)

@pytest.mark.parametrize('cmd,axes,target,expected', [(3,'z',[999,999,4.8,0],15),(3,'rz',[999,999,999,90],10),(4,'xy',[4,6,999,0],5/.18+5),(5,'',[3,4,0,0],5/.18+5),(3,'xyz',[4,6,3,0],7)])
def test_action_uses_resolved_target(node,cmd,axes,target,expected):
    constants=node._execute_motion.__globals__['BasicMotion'].Goal
    for name,value in [('SET',3),('WMOVE',1),('BMOVE',2),('WTRAVEL',4),('BTRAVEL',5)]:
        setattr(constants,name,value)
    pose=Coordinate(x=1,y=2,z=3,rz=90)
    node.get_state=lambda:(pose,pose,None)
    node._motion_block_reasons=lambda:[]
    budgets=[]
    def capture(*args,timeout):
        budgets.append(timeout)
        return False
    node.setxyzrz=capture
    node._travel_world=capture
    goal=Goal(cmd,target,timeout=7 if expected==7 else 0)
    goal.request.axes=axes
    node._execute_motion(goal)
    assert budgets==pytest.approx([expected])
    assert goal.terminal=='aborted'

@pytest.mark.parametrize('budget,expected',[(20,8),(3,3)])
def test_stalled_travel_step_aborts_without_next_target(node,budget,expected):
    ns=node._step_move_world.__globals__
    ns['time']=NS(monotonic=lambda:0)
    ns['rclpy']=NS(ok=lambda:True)
    ns['LATERAL_LAMBDA']=2
    node.get_state=lambda:(Coordinate(),Coordinate(),None)
    node._is_cancelled=lambda:False
    node._calc_step_size=lambda angle:.6
    commands=[]; waits=[]
    node.set_step=lambda **kw:commands.append(kw)
    def wait(angle,timeout):
        waits.append(timeout)
        return False
    node._wait_step_convergence=wait
    node._cmd_and_wait=lambda *a,**kw:pytest.fail('must not send final target after failed step')
    assert not node._step_move_world(2,0,0,0,timeout=budget,step_timeout_limit=8)
    assert len(commands)==1
    assert waits==[expected]

def test_travel_turn_and_move_share_budget(node):
    now=[0]
    node._travel_world.__globals__['time']=NS(monotonic=lambda:now[0])
    node.get_state=lambda:(Coordinate(),Coordinate(),None)
    calls=[]
    def step(*args,**kw):
        calls.append(kw)
        now[0]+=3
        return True
    node._step_move_world=step
    assert node._travel_world(2,0,0,0,timeout=10)
    assert [c['timeout'] for c in calls]==[10,7]
    assert all(c['step_timeout_limit']==8 for c in calls)

def test_failed_turn_never_moves(node):
    node.get_state=lambda:(Coordinate(),Coordinate(),None)
    calls=[]
    def step(*args,**kw):
        calls.append(args)
        return False
    node._step_move_world=step
    assert not node._travel_world(2,0,0,0,timeout=10)
    assert len(calls)==1


@pytest.mark.parametrize('errors,override,expected', [
    ((.14, -.14, .09, 9.), True, True),
    ((.151, 0., 0., 0.), True, False),
    ((0., -.151, 0., 0.), True, False),
    ((0., 0., .101, 0.), True, False),
    ((0., 0., 0., 10.1), True, False),
    ((.14, 0., 0., 0.), False, False),
    ((0., 0., 0., 9.), False, False),
    ((.09, -.09, .09, 4.), False, True),
])
def test_set_arrival_overrides_are_per_goal_and_preserve_depth(node, errors, override, expected):
    ns = node._execute_motion.__globals__
    for name, value in [('SET', 3), ('WMOVE', 1), ('BMOVE', 2), ('WTRAVEL', 4), ('BTRAVEL', 5)]:
        setattr(ns['BasicMotion'].Goal, name, value)
    clock = [0.]
    ns['time'] = NS(monotonic=lambda: clock[0], sleep=lambda dt: clock.__setitem__(0, clock[0]+dt))
    ns['rclpy'] = NS(ok=lambda: True)
    pose = Coordinate()
    target = Coordinate(x=errors[0], y=errors[1], z=errors[2], rz=errors[3])
    node.get_state = lambda: (pose, target, target)
    node._target_error_snapshot = lambda: (pose, target, target)
    node._motion_block_reasons = lambda: []
    node._is_cancelled = lambda: False
    node.set_world = lambda *args: None
    goal = Goal(3, [target.x, target.y, target.z, target.rz], timeout=.2)
    goal.request.axes = 'xyzrz'
    if override:
        goal.request.position_tolerance = .15
        goal.request.yaw_tolerance = 10.
    result = node._execute_motion(goal)
    assert result.success is expected
    assert goal.terminal == ('succeeded' if expected else 'aborted')
    assert node._action_axes is None
    if expected and override:
        # A subsequent plain SET must return to the original XY/yaw bounds.
        plain = Goal(3, [target.x, target.y, target.z, target.rz], timeout=.2)
        plain.request.axes = 'xyzrz'
        assert not node._execute_motion(plain).success


@pytest.mark.parametrize('field,value', [('position_tolerance', -1.),
    ('position_tolerance', float('nan')), ('yaw_tolerance', -1.),
    ('yaw_tolerance', float('inf')), ('yaw_tolerance', 181.)])
def test_set_rejects_invalid_arrival_override_before_motion(node, field, value):
    ns = node._execute_motion.__globals__
    for name, number in [('SET', 3), ('WMOVE', 1), ('BMOVE', 2), ('WTRAVEL', 4), ('BTRAVEL', 5)]:
        setattr(ns['BasicMotion'].Goal, name, number)
    node.get_state = lambda: (Coordinate(), Coordinate(), None)
    node._motion_block_reasons = lambda: []
    node.setxyzrz = lambda *a, **k: pytest.fail('invalid overrides must not start SET')
    goal = Goal(3)
    setattr(goal.request, field, value)
    with pytest.raises(ValueError):
        node._execute_motion(goal)
