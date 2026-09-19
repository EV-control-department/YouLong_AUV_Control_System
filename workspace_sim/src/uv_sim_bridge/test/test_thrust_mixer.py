import pytest

from uv_sim_bridge.thrust_mixer import ThrustMixer


@pytest.fixture()
def mixer():
    return ThrustMixer(heave_factor=0.8)


def test_horizontal_axes_match_measured_stonefish_directions(mixer):
    # These are the commands that produce the measured physical directions
    # in Stonefish; inverted_setpoint does not invert the fluid thrust axis.
    assert mixer.mix6(1.0, 0.0, 0.0, 0.0, 0.0, 0.0) == [1.0, 1.0, 0.0, 0.0, -1.0, -1.0]
    assert mixer.mix6(0.0, 1.0, 0.0, 0.0, 0.0, 0.0) == [1.0, -1.0, 0.0, 0.0, 1.0, -1.0]
    assert mixer.mix6(0.0, 0.0, 0.0, 0.0, 0.0, 1.0) == [-1.0, 1.0, 0.0, 0.0, 1.0, -1.0]


def test_vertical_axis_matches_measured_stonefish_directions(mixer):
    assert mixer.mix6(0.0, 0.0, 1.0, 0.0, 0.0, 0.0) == [0.0, 0.0, 0.8, 0.8, 0.0, 0.0]


def test_clamps_combined_commands(mixer):
    assert mixer.mix6(1.0, 1.0, 1.0, 0.0, 0.0, 1.0) == [1.0, 1.0, 0.8, 0.8, 1.0, -1.0]
