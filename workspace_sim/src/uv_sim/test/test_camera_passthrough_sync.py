"""Regression tests for the simulator's stitched stereo image timing."""

from types import SimpleNamespace

from uv_sim.camera_passthrough import CameraPassthrough


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
        _image_at(13, 100_000_000), _image_at(13, 0)) is None
