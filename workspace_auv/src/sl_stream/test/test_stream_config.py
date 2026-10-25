from types import SimpleNamespace as NS

import pytest

from sl_stream.camera_streamer import _catalog_frame_rate, _next_pts, gst_pipeline_description
from sl_stream.service import _descriptor_identity, stream_name


def _camera(camera_id, name, enabled=True, width=1280, height=720):
    return NS(
        id=camera_id, name=name, ice_service=f'youlong/sl_camera/{name}',
        width=width, height=height, enabled=enabled,
        online=True, encoding='JPEG', requested_fps=30.0, negotiated_fps=29.7,
        actual_fps=28.5,
    )


def _catalog(cameras):
    return NS(
        cameras=cameras, configured_count=len(cameras),
        enabled_count=sum(camera.enabled for camera in cameras),
        online_count=sum(getattr(camera, 'online', False) for camera in cameras),
    )


def test_route_identity_discovers_variable_camera_counts_and_config():
    catalog = _catalog([
        _camera(8, 'down', width=2560, height=960),
        _camera(3, 'sl'),
    ])

    assert _descriptor_identity(catalog) == {
        8: ('down', 'youlong/sl_camera/down', 2560, 960, True),
        3: ('sl', 'youlong/sl_camera/sl', 1280, 720, True),
    }
    assert [stream_name(camera.id) for camera in catalog.cameras] == ['sl_8', 'sl_3']


def test_invalid_camera_catalog_is_rejected():
    duplicate = _camera(2, 'one')
    with pytest.raises(ValueError, match='duplicate or invalid camera ID'):
        _descriptor_identity(_catalog([duplicate, duplicate]))

    mismatch = _catalog([_camera(1, 'one')])
    mismatch.enabled_count = 0
    with pytest.raises(ValueError, match='counts'):
        _descriptor_identity(mismatch)


def test_catalog_frame_rate_and_runtime_encoder_parameters():
    camera = _camera(1, 'front')
    assert _catalog_frame_rate(camera, 24.0) == 24.0
    camera.actual_fps = 0.0
    assert _catalog_frame_rate(camera, 30.0) == pytest.approx(29.7)
    camera.negotiated_fps = camera.requested_fps = 0.0
    assert _catalog_frame_rate(camera, 30.0) == 10.0
    camera.actual_fps = 0.5
    assert _catalog_frame_rate(camera, 30.0) == pytest.approx(0.5)
    camera.actual_fps = 60.0
    assert _catalog_frame_rate(camera, 60.0) == 30.0

    description = gst_pipeline_description(2560, 960, 30, 2_000_000,
                                           3_000_000, 'software', 9)
    assert 'width=2560,height=960' in description
    assert 'bitrate=2000 key-int-max=30' in description
    assert 'vbv-maxrate=3000:vbv-bufsize=3000' in description
    assert 'bframes=0' in description
    assert 'h265parse config-interval=-1 ! mpegtsmux' in description

    jetson = gst_pipeline_description(1280, 720, 30, 2_000_000,
                                      3_000_000, 'jetson', 9)
    assert 'nvv4l2h265enc control-rate=0 bitrate=2000000' in jetson
    assert 'peak-bitrate=3000000' in jetson


def test_source_timestamp_mapping_is_monotonic_across_clock_resets():
    state = {'source_base': None, 'last_source': -1, 'last_pts': -1}
    assert _next_pts(5_000_000_000, state, 25.0) == (0, 40_000_000)
    assert _next_pts(5_040_000_000, state, 25.0) == (40_000_000, 40_000_000)
    pts, duration = _next_pts(1_000_000_000, state, 25.0)
    assert (pts, duration) == (80_000_000, 40_000_000)
