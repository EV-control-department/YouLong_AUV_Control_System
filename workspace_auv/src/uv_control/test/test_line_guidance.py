"""BLINE geometry and deterministic kinematic acceptance (no actuators)."""
import math

import pytest

from uv_control.line_guidance import (
    LineConfig, LineGuidance, dot, norm, rotate_body_to_world,
    rotate_world_to_body, validate_goal,
)


@pytest.mark.parametrize('yaw', [0.0, 47.0, 90.0, -140.0])
@pytest.mark.parametrize('delta', [(1, 0, 0), (0, 1, 0), (1, -1, .4), (0, 0, 1)])
def test_frozen_endpoint_and_full_velocity_rotation(yaw, delta):
    start = (2, -3, .6)
    guide = LineGuidance(start, yaw, delta, .15, LineConfig())
    offset = rotate_body_to_world(delta, 0, 0, yaw)
    expected = tuple(a + b for a, b in zip(start, offset))
    assert guide.end == pytest.approx(expected)
    # A later yaw change affects velocity conversion, never the fixed endpoint.
    vector = (.1, -.05, .02)
    body = rotate_world_to_body(vector, 22, -15, guide.yaw)
    assert rotate_body_to_world(body, 22, -15, guide.yaw) == pytest.approx(vector)
    guide.step(start, (0, 0, 0), .05)
    assert guide.end == pytest.approx(expected)
    if delta[:2] == (0, 0):
        assert guide.yaw == pytest.approx(yaw)


@pytest.mark.parametrize('speed', [-.1, math.nan, math.inf, -math.inf])
def test_invalid_cruise_speed(speed):
    with pytest.raises(ValueError):
        validate_goal([1, 0, 0, 0], '', speed, 0, LineConfig())


@pytest.mark.parametrize('target,axes', [
    ([0, 0, 0, 0], ''), ([0, 0, 0, 0], ''),
    ([1, 0, 0], ''), ([1, 0, math.nan, 0], ''), ([1, 0, 0, 0], 'x'),
])
def test_invalid_displacement(target, axes):
    with pytest.raises(ValueError):
        validate_goal(target, axes, .15, 0, LineConfig())


def test_default_and_custom_speed():
    assert validate_goal([1, 0, 0, 0], '', 0, 0, LineConfig()) == .15
    assert validate_goal([1, 0, 0, 0], 'xyz', .17, 15, LineConfig()) == .17


@pytest.mark.parametrize('length', [.03, .3, 2.0])
@pytest.mark.parametrize('offset', [(0, 0, 0), (0, .5, .4), (.15, 0, 0)])
def test_tracking_capture_brake_overshoot_converges(length, offset):
    cfg = LineConfig()
    guide = LineGuidance((0, 0, 0), 0, (length, 0, 0), .15, cfg)
    position = offset
    measured = (0, 0, 0)
    phases = set()
    for _ in range(2400):
        previous = guide.velocity
        command, phase, sample = guide.step(position, measured, .05)
        phases.add(phase)
        assert norm(command) <= cfg.max_speed + 1e-9
        assert norm(tuple(v - p for v, p in zip(command, previous))) <= .0025 + 1e-9
        # Simple velocity-loop lag, distinct from guidance command.
        measured = tuple(v + .2 * (c - v) for v, c in zip(measured, command))
        position = tuple(p + v * .05 for p, v in zip(position, measured))
        if guide.reached(position, measured, 0):
            break
    else:
        pytest.fail(f'failed to converge: position={position}, velocity={measured}')
    assert all(abs(b - p) <= .1 for p, b in zip(position, guide.end))
    if offset[1]:
        assert 'CAPTURE' in phases


def test_capture_hysteresis_and_reverse():
    guide = LineGuidance((0, 0, 0), 0, (1, 0, 0), .15, LineConfig())
    assert guide.step((0, .4, 0), (0, 0, 0), .05)[1] == 'CAPTURE'
    assert guide.step((0, .2, 0), (0, 0, 0), .05)[1] == 'CAPTURE'
    assert guide.step((0, .1, 0), (0, 0, 0), .05)[1] == 'CRUISE'
    for _ in range(200):
        velocity, phase, sample = guide.step((1.2, 0, 0), (0, 0, 0), .05)
    assert phase == 'TERMINAL' and -.05 <= dot(velocity, guide.direction) < 0


def test_brake_uses_measured_velocity():
    guide = LineGuidance((0, 0, 0), 0, (1, 0, 0), .15, LineConfig())
    _, phase, _ = guide.step((.5, 0, 0), (.2, 0, 0), .05)
    assert phase == 'BRAKE'


@pytest.mark.parametrize('speed', [.18, .25, .28, .32, 1.0, 1e300])
def test_overspeed_is_accepted_and_clamped(speed):
    applied = validate_goal([1, 0, 0, 0], 'xyz', speed, 0, LineConfig())
    assert 0 < applied < .28
    assert applied == pytest.approx(min(speed, .28 - 1e-6))


def test_overspeed_default_is_clamped_instead_of_rejected():
    cfg = LineConfig(default_speed=.5, max_speed=1.)
    cfg.validate()
    assert validate_goal([1, 0, 0, 0], '', 0, 0, cfg) < .28


@pytest.mark.parametrize('terminal', [False, True])
def test_total_speed_is_hard_limited_even_with_large_parameters(terminal):
    cfg = LineConfig(max_speed=1., cross_speed=1., terminal_speed=1.,
                     capture_enter=10., capture_exit=5.)
    cfg.validate()
    guide = LineGuidance((0, 0, 0), 0, (100, 0, 0), 1., cfg)
    position = (101, .5, .5) if terminal else (0, .5, .5)
    for _ in range(500):
        command, _, _ = guide.step(position, (0, 0, 0), .05)
        assert norm(command) < .32
        if not terminal:
            assert dot(command, guide.direction) < .28
    assert norm(command) > .3  # exercises combined along/cross limiting


def test_lower_configured_total_limit_is_respected():
    cfg = LineConfig(max_speed=.12)
    cfg.validate()
    assert validate_goal([1, 0, 0, 0], '', 1., 0, cfg) == .12


@pytest.mark.parametrize('attitude', [(0, 0, 0), (22, -15, 47), (89, 74, -140)])
def test_float32_body_transport_remains_below_total_limit(attitude):
    import struct
    cfg = LineConfig(max_speed=1., cross_speed=1., capture_enter=10., capture_exit=5.)
    guide = LineGuidance((0, 0, 0), 0, (100, 0, 0), 1., cfg)
    for _ in range(300):
        command, _, _ = guide.step((0, .5, .5), (0, 0, 0), .05)
    body = rotate_world_to_body(command, *attitude)
    transported = struct.unpack('fff', struct.pack('fff', *body))
    assert norm(transported) < .32


@pytest.mark.parametrize('position,yaw,target', [
    ((2., -3., .5), 47., (0., 0., 0.)),
    ((2., -3., .5), -140., (4., 1., .8)),
])
def test_world_line_endpoint_does_not_depend_on_body_heading(position, yaw, target):
    guide = LineGuidance(position, yaw, target, .15, LineConfig(),
                         world=True, final_yaw=135.)
    assert guide.end == pytest.approx(target)
    assert guide.length == pytest.approx(norm(tuple(b-a for a,b in zip(position,target))))
    assert guide.final_yaw == 135.


def test_zero_length_world_line_and_rotation_only_body_line():
    cfg = LineConfig()
    assert validate_goal([0,0,0,90], 'xyz', .15, 0, cfg) == .15
    assert validate_goal([0,0,0,0], 'xyz', .15, 0, cfg, world=True) == .15
    guide = LineGuidance((0,0,0), 35, (0,0,0), .15, cfg, world=True, final_yaw=90)
    assert guide.length == 0
    assert guide.reached((0,0,0), (0,0,0), 35)
    assert not guide.reached((0,0,0), (0,0,0), 35, final=True)
    assert guide.reached((0,0,0), (0,0,0), 90, final=True)
