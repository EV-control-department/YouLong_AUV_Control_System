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


@pytest.mark.parametrize('speed', [-.1, .18, .25, math.nan, math.inf])
def test_invalid_cruise_speed(speed):
    with pytest.raises(ValueError):
        validate_goal([1, 0, 0, 0], '', speed, 0, LineConfig())


@pytest.mark.parametrize('target,axes', [
    ([0, 0, 0, 0], ''), ([1, 0, 0, 1], ''),
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
