"""BLINE geometry and deterministic kinematic acceptance (no actuators)."""
import math
from dataclasses import asdict
from pathlib import Path

import pytest
import yaml

from uv_control.line_guidance import (
    CRUISE_SPEED_LIMIT, TOTAL_SPEED_LIMIT, LineConfig, LineGuidance,
    dot, norm, rotate_body_to_world, rotate_world_to_body, validate_goal,
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
    assert validate_goal([1, 0, 0, 0], '', 0, 0, LineConfig()) == .30
    assert validate_goal([1, 0, 0, 0], 'xyz', .17, 15, LineConfig()) == .17


def test_builtin_profile_matches_launch_yaml():
    path = Path(__file__).parents[1] / 'config/default.yaml'
    parameters = yaml.safe_load(path.read_text())['/basic_motion']['ros__parameters']
    configured = {key[len('line.'):]: value for key, value in parameters.items()
                  if key.startswith('line.')}
    assert asdict(LineConfig()) == configured


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
        assert norm(tuple(v - p for v, p in zip(command, previous))) <= (
            cfg.vector_acceleration * .05 + 1e-9)
        # Simple velocity-loop lag, distinct from guidance command.
        measured = tuple(v + .2 * (c - v) for v, c in zip(measured, command))
        position = tuple(p + v * .05 for p, v in zip(position, measured))
        if guide.reached(position, measured, 0):
            break
    else:
        pytest.fail(f'failed to converge: position={position}, velocity={measured}')
    assert all(abs(b - p) <= cfg.position_tolerance for p, b in zip(position, guide.end))
    if offset[1]:
        assert 'CAPTURE' in phases


def test_capture_hysteresis_and_reverse():
    guide = LineGuidance((0, 0, 0), 0, (1, 0, 0), .15, LineConfig())
    assert guide.step((0, .4, 0), (0, 0, 0), .05)[1] == 'CAPTURE'
    assert guide.step((0, .2, 0), (0, 0, 0), .05)[1] == 'CAPTURE'
    assert guide.step((0, .1, 0), (0, 0, 0), .05)[1] == 'CRUISE'
    for _ in range(200):
        velocity, phase, sample = guide.step((1.2, 0, 0), (0, 0, 0), .05)
    assert phase == 'TERMINAL'
    assert dot(velocity, guide.direction) == pytest.approx(-guide.config.reverse_speed)


def test_brake_uses_measured_velocity():
    guide = LineGuidance((0, 0, 0), 0, (1, 0, 0), .15, LineConfig())
    assert guide.step((.5, 0, 0), (0, 0, 0), .05)[1] == 'CRUISE'
    _, phase, _ = guide.step((.5, 0, 0), (.5, 0, 0), .05)
    assert phase == 'BRAKE'


@pytest.mark.parametrize('speed', [.18, .30, .60, .79, .80, .81, 1.0, 1e300])
def test_overspeed_is_accepted_and_clamped(speed):
    applied = validate_goal([1, 0, 0, 0], 'xyz', speed, 0, LineConfig())
    assert 0 < applied < .80
    assert applied == pytest.approx(min(speed, CRUISE_SPEED_LIMIT))


def test_overspeed_default_is_clamped_instead_of_rejected():
    cfg = LineConfig(default_speed=1., max_speed=1.)
    cfg.validate()
    assert validate_goal([1, 0, 0, 0], '', 0, 0, cfg) == CRUISE_SPEED_LIMIT


@pytest.mark.parametrize('terminal', [False, True])
def test_total_speed_is_hard_limited_even_with_large_parameters(terminal):
    cfg = LineConfig(max_speed=1., cross_speed=1., terminal_speed=1.,
                     capture_enter=10., capture_exit=5.)
    cfg.validate()
    guide = LineGuidance((0, 0, 0), 0, (100, 0, 0), 1., cfg)
    position = (101, 2, 2) if terminal else (0, .5, .5)
    for _ in range(500):
        command, _, _ = guide.step(position, (0, 0, 0), .05)
        assert norm(command) <= TOTAL_SPEED_LIMIT + 1e-12
        assert norm(command) < .80
        if not terminal:
            assert dot(command, guide.direction) <= CRUISE_SPEED_LIMIT + 1e-12
    assert norm(command) > TOTAL_SPEED_LIMIT - .01  # exercises combined limiting


def test_lower_configured_total_limit_is_respected():
    cfg = LineConfig(max_speed=.12)
    cfg.validate()
    assert validate_goal([1, 0, 0, 0], '', 1., 0, cfg) == .12
    guide = LineGuidance((0, 0, 0), 0, (10, 0, 0), .8, cfg)
    for _ in range(100):
        command, _, _ = guide.step((0, .2, 0), (0, 0, 0), .05)
        assert norm(command) <= .12 + 1e-12


@pytest.mark.parametrize('attitude', [(0, 0, 0), (22, -15, 47), (89, 74, -140)])
def test_float32_body_transport_remains_below_total_limit(attitude):
    import struct
    cfg = LineConfig(max_speed=1., cross_speed=1., capture_enter=10., capture_exit=5.)
    guide = LineGuidance((0, 0, 0), 0, (100, 0, 0), 1., cfg)
    for _ in range(300):
        command, _, _ = guide.step((0, .5, .5), (0, 0, 0), .05)
    body = rotate_world_to_body(command, *attitude)
    transported = struct.unpack('fff', struct.pack('fff', *body))
    assert norm(transported) < .80


@pytest.mark.parametrize('world', [False, True])
@pytest.mark.parametrize('offset', [(0, .2, 0), (0, .15, .1)])
def test_full_speed_drift_gets_correction_without_breaking_slew_limits(world, offset):
    cfg = LineConfig()
    guide = LineGuidance((0, 0, 0), 0, (100, 0, 0), .8, cfg, world=world)
    guide.velocity = (TOTAL_SPEED_LIMIT, 0, 0)
    for _ in range(100):
        previous = guide.velocity
        command, phase, _ = guide.step(offset, previous, .05)
        assert phase == 'CRUISE'
        assert dot(command, offset) < 0
        assert norm(command) < .8
        assert norm(tuple(a-b for a, b in zip(command, previous))) <= cfg.vector_acceleration*.05 + 1e-12
        assert -cfg.deceleration*.05 - 1e-12 <= command[0]-previous[0] <= cfg.acceleration*.05 + 1e-12
    assert command[1:] == pytest.approx(tuple(-cfg.cross_gain*v for v in offset[1:]))
    assert command[0] < TOTAL_SPEED_LIMIT


def test_terminal_transition_respects_deceleration_then_reaches_low_speed():
    cfg = LineConfig()
    guide = LineGuidance((0, 0, 0), 0, (1, 0, 0), .8, cfg)
    guide.velocity = (.5, 0, 0)
    for _ in range(100):
        previous = guide.velocity
        command, phase, _ = guide.step((.7, 0, 0), previous, .05)
        assert phase == 'TERMINAL'
        assert -cfg.deceleration*.05 - 1e-12 <= command[0]-previous[0] <= cfg.acceleration*.05 + 1e-12
        assert norm(tuple(a-b for a, b in zip(command, previous))) <= cfg.vector_acceleration*.05 + 1e-12
    assert norm(command) <= cfg.terminal_speed + 1e-12


def test_short_segment_stays_in_low_speed_terminal_phase():
    cfg = LineConfig()
    guide = LineGuidance((0, 0, 0), 0, (.3, 0, 0), .8, cfg)
    for _ in range(100):
        command, phase, _ = guide.step((0, 0, 0), (0, 0, 0), .05)
        assert phase == 'TERMINAL'
        assert norm(command) <= cfg.terminal_speed + 1e-12


@pytest.mark.parametrize('speed', [.3, .8])
@pytest.mark.parametrize('lag', [.1, .3, .5])
@pytest.mark.parametrize('offset', [(0, 0, 0), (0, .5, .4)])
def test_long_segment_converges_with_lag_and_cross_track_offset(speed, lag, offset):
    cfg = LineConfig()
    guide = LineGuidance((0, 0, 0), 0, (10, 0, 0), speed, cfg)
    position, measured = offset, (0, 0, 0)
    peak = 0.
    for _ in range(2400):
        if guide.reached(position, measured, 0):
            break
        previous = guide.velocity
        command, _, _ = guide.step(position, measured, .05)
        assert norm(command) < .8
        assert norm(tuple(a-b for a, b in zip(command, previous))) <= cfg.vector_acceleration*.05 + 1e-12
        peak = max(peak, command[0])
        alpha = 1-math.exp(-.05/lag)
        measured = tuple(v+alpha*(c-v) for v, c in zip(measured, command))
        position = tuple(p+v*.05 for p, v in zip(position, measured))
    else:
        pytest.fail(f'did not converge: position={position}, velocity={measured}')
    assert peak > .95*min(speed, CRUISE_SPEED_LIMIT)
    assert guide.reached(position, measured, 0)


def test_goal_position_tolerance_override_remains_effective():
    guide = LineGuidance((0, 0, 0), 0, (1, 0, 0), .3, LineConfig())
    assert guide.reached((.95, 0, 0), (0, 0, 0), 0)
    assert not guide.reached((.95, 0, 0), (0, 0, 0), 0, position_tolerance=.01)


def test_arrival_requires_low_measured_speed_and_aligned_heading():
    guide = LineGuidance((0, 0, 0), 0, (1, 0, 0), .3, LineConfig())
    assert not guide.reached((1, 0, 0), (.05, 0, 0), 0)
    assert not guide.reached((1, 0, 0), (0, 0, 0), 4)
    assert guide.reached((1, 0, 0), (.03, 0, 0), 2)


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
