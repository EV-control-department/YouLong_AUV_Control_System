"""Profiles control acquisition independently of available-frame recording."""

from pathlib import Path

import pytest
import yaml

from sl_camera.profile import camera_node_specs, load_profile


def default_document():
    path = Path(__file__).parents[1] / 'config/profiles/default.yaml'
    return yaml.safe_load(path.read_text())


def test_default_devices_and_reused_down_camera_defaults(tmp_path):
    from uv_camera.camera_config import load_camera_config

    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump(default_document()))
    profile = load_profile(str(path))
    sl, down = camera_node_specs(profile)
    assert sl['parameters'][0]['device'] == '/dev/video0'
    assert sl['parameters'][0]['width'] == 1280
    assert sl['parameters'][0]['height'] == 720
    params = down['parameters'][0]
    assert down['package'] == 'uv_camera'
    assert not params['enable_front'] and params['enable_down']
    assert not params['sim_mode'] and params['camera_mode'] == 'real'
    assert params['down_camera_device'] == '/dev/video3'
    config = load_camera_config('down', mode='real')
    assert config.capture_resolution == (2560, 960)
    assert config.eye_resolution == (1280, 960)


@pytest.mark.parametrize('enable_sl,enable_down,packages', [
    (True, True, ['sl_camera', 'uv_camera']),
    (True, False, ['sl_camera']),
    (False, True, ['uv_camera']),
    (False, False, []),
])
def test_startup_switches(tmp_path, enable_sl, enable_down, packages):
    document = default_document()
    document['cameras']['sl']['enabled'] = enable_sl
    document['cameras']['down']['enabled'] = enable_down
    path = tmp_path / 'switches.yaml'
    path.write_text(yaml.safe_dump(document))
    assert [spec['package'] for spec in camera_node_specs(load_profile(str(path)))] == packages


def test_recorder_cli_uses_profile_defaults_and_cli_overrides(tmp_path):
    from sl_record.recorder import _parse_args

    document = default_document()
    document['recording'].update(video_fps=12, bag_duration=7,
                                 bag_storage='sqlite3', output_root='/data/example')
    path = tmp_path / 'custom.yaml'
    path.write_text(yaml.safe_dump(document))
    args = _parse_args(['--profile', str(path)])
    assert (args.video_fps, args.bag_duration, args.bag_storage, args.output_root) == (
        12, 7, 'sqlite3', '/data/example')
    assert args.raw_cameras == ('sl', 'down')
    override = _parse_args(['--profile', str(path), '--video-fps', '20'])
    assert override.video_fps == 20


def test_disabling_capture_does_not_disable_external_recording(tmp_path):
    from sl_record.recorder import _parse_args

    document = default_document()
    document['cameras']['sl']['enabled'] = False
    document['cameras']['down']['enabled'] = False
    path = tmp_path / 'external.yaml'
    path.write_text(yaml.safe_dump(document))
    assert camera_node_specs(load_profile(str(path))) == []
    assert _parse_args(['--profile', str(path)]).raw_cameras == ('sl', 'down')


def test_profile_rejects_bad_enable_and_missing_device(tmp_path):
    document = default_document()
    path = tmp_path / 'bad.yaml'
    document['cameras']['sl']['enabled'] = 'false'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='enabled'):
        load_profile(str(path))
    document['cameras']['sl']['enabled'] = False
    document['cameras']['down']['device'] = ''
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='device'):
        load_profile(str(path))
