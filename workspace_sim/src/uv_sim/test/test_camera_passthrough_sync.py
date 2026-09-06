"""Regression tests for the simulator's stitched stereo image timing."""

from types import SimpleNamespace

from uv_sim.camera_passthrough import (
    FRONT_STEREO_STITCH_SLOP_SEC,
    CameraPassthrough,
)


def _image_at(seconds: int, nanoseconds: int = 0):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds)))


def test_stitch_accepts_one_contemporaneous_stereo_pair():
    assert CameraPassthrough._synchronized_pair_key(
        _image_at(12, 10_000_000), _image_at(12, 35_000_000)) == (
            12, 10_000_000, 12, 35_000_000)


def test_stitch_does_not_mix_a_new_image_with_the_previous_stereo_frame():
    assert CameraPassthrough._synchronized_pair_key(
        _image_at(13, 130_000_000), _image_at(13, 0)) is None


def test_front_simulator_pair_accepts_its_render_offset():
    assert CameraPassthrough._synchronized_pair_key(
        _image_at(14, 100_000_000), _image_at(14, 0),
        slop_sec=FRONT_STEREO_STITCH_SLOP_SEC) == (
            14, 100_000_000, 14, 0)
