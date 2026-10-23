"""Configurable camera lists and independent metadata semantics."""

from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from sl_camera.catalog import descriptor
from sl_camera.profile import load_profile, load_settings


def default_document():
    path = Path(__file__).parents[1] / 'config/profiles/default.yaml'
    return yaml.safe_load(path.read_text())


def write_profile(tmp_path, document):
    path = tmp_path / 'collection.yaml'
    path.write_text(yaml.safe_dump(document))
    return path


def test_defaults_are_fully_explicit_and_uncalibrated(tmp_path):
    profile = load_profile(str(write_profile(tmp_path, default_document())))
    assert [(x.id, x.name, x.device, x.width, x.height) for x in profile.cameras] == [
        (1, 'sl', '/dev/video0', 1280, 720), (2, 'down', '/dev/video3', 2560, 960)]
    for spec in profile.cameras:
        message = descriptor(spec)
        assert not message.calibrated
        assert message.calibrations == []
        assert message.actual_fps == 0
        assert message.encoding == 'JPEG'
    assert descriptor(profile.cameras[1]).eye_width == 1280


@pytest.mark.parametrize('count', [0, 1, 2, 3])
def test_any_number_of_cameras(tmp_path, count):
    document = default_document()
    template = document['cameras'][0]
    document['cameras'] = [
        dict(template, id=10 + index, name=f'camera_{index}', device=f'/dev/test{index}',
             ice_service=f'collection/camera_{index}', frame_id=f'camera_{index}_frame')
        for index in range(count)]
    assert len(load_profile(str(write_profile(tmp_path, document))).cameras) == count


@pytest.mark.parametrize('field', ['id', 'name', 'ice_service', 'device'])
def test_duplicate_identity_or_enabled_device_rejected(tmp_path, field):
    document = default_document()
    document['cameras'][1][field] = document['cameras'][0][field]
    with pytest.raises(ValueError, match='duplicate'):
        load_profile(str(write_profile(tmp_path, document)))


@pytest.mark.parametrize('key,value', [
    ('name', '../unsafe'), ('id', 0), ('enabled', 'false'), ('width', 0),
    ('fps', float('nan')), ('pixel_format', 'YUYV'), ('layout', 'unknown'),
    ('calibrations', {}), ('ice_service', '../unsafe'),
])
def test_bad_camera_definitions_rejected(tmp_path, key, value):
    document = default_document()
    document['cameras'][0][key] = value
    with pytest.raises(ValueError):
        load_profile(str(write_profile(tmp_path, document)))


def test_populated_calibration_can_be_carried_without_pixel_processing(tmp_path):
    profile = load_profile(str(write_profile(tmp_path, default_document())))
    spec = replace(profile.cameras[0], calibrations=[{
        'eye': 'mono', 'camera_info': {
            'width': 1280, 'height': 720,
            'k': [1000., 0., 640., 0., 1000., 360., 0., 0., 1.],
            'd': [0., 0., 0., 0., 0.], 'distortion_model': 'plumb_bob'}}])
    result = descriptor(spec)
    assert result.calibrated
    assert result.calibrations[0].camera_info.k[0] == 1000


def test_recording_settings_do_not_read_camera_definitions(tmp_path):
    from sl_record.recorder import _parse_args

    document = default_document()
    document['cameras'] = 'not a camera list; the recorder must rely on DDS'
    document['recording'].update(video_fps=12, bag_duration=7, bag_storage='sqlite3')
    path = write_profile(tmp_path, document)
    assert load_settings(str(path))[2]['video_fps'] == 12
    args = _parse_args(['--profile', str(path), '--video-fps', '20'])
    assert args.video_fps == 20
    assert args.bag_duration == 7
    assert args.bag_storage == 'sqlite3'
    with pytest.raises(ValueError):
        load_profile(str(path))
