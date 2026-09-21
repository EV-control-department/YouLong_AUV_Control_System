from types import SimpleNamespace

import numpy as np
import pytest

from uv_camera.camera_tf import (
    CAMERA_FRAME_IDS,
    CameraExtrinsicsProvider,
    CameraExtrinsicsUnavailable,
    quaternion_to_rotation,
)


def _transform(x, y, z, quaternion=(0.0, 0.0, 0.0, 1.0)):
    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y, z=z),
            rotation=SimpleNamespace(
                x=quaternion[0], y=quaternion[1],
                z=quaternion[2], w=quaternion[3]),
        )
    )


class FakeBuffer:
    def __init__(self, transforms):
        self.transforms = transforms
        self.calls = []

    def lookup_transform(self, target, source, time, timeout=None):
        self.calls.append((target, source))
        if source not in self.transforms:
            raise LookupError(source)
        return self.transforms[source]


def test_camera_frame_mapping_and_lookup_direction():
    transforms = {
        frame: _transform(index, 0.0, 0.0)
        for index, frame in enumerate(CAMERA_FRAME_IDS.values())
    }
    buffer = FakeBuffer(transforms)
    provider = CameraExtrinsicsProvider(
        object(), base_frame="base_link", buffer=buffer, listener=object())
    snapshot = provider.snapshot()

    assert set(snapshot) == set(CAMERA_FRAME_IDS)
    assert all(target == "base_link" for target, _ in buffer.calls)
    assert [source for _, source in buffer.calls] == list(CAMERA_FRAME_IDS.values())
    assert snapshot["front_left"].translation.tolist() == [0.0, 0.0, 0.0]


def test_quaternion_to_rotation_matrix():
    rotation = quaternion_to_rotation(
        (0.0, 0.0, np.sin(np.pi / 4.0), np.cos(np.pi / 4.0)))
    np.testing.assert_allclose(
        rotation, [[0.0, -1.0, 0.0],
                   [1.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0]], atol=1e-8)


def test_missing_camera_tf_is_explicit():
    buffer = FakeBuffer({})
    provider = CameraExtrinsicsProvider(
        object(), base_frame="base_link", timeout_sec=0.0,
        buffer=buffer, listener=object())
    with pytest.raises(CameraExtrinsicsUnavailable, match="missing camera TF"):
        provider.snapshot()


def test_invalid_quaternion_is_rejected():
    with pytest.raises(ValueError, match="non-zero"):
        quaternion_to_rotation((0.0, 0.0, 0.0, 0.0))
