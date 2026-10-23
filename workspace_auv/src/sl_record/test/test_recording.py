"""Dynamic camera identities, version association, restarts, and archive integrity."""

from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from sl_camera.catalog import descriptor
from sl_camera.profile import load_profile
from sl_msgs.msg import CameraCatalog
from sl_record.frames import DiscoveredFrames
from sl_record.recorder import BagCommand, _parse_args
from uv_image_transport import ENCODING_JPEG, FrameHeader, FramePacket
from uv_record.recorder import Recorder, _parse_args as uv_parse_args
from uv_record.recover import recover_raw_camera


def specs(count=1):
    template = load_profile().cameras[0]
    return [replace(template, id=index + 10, name=f'custom_{index}', width=64, height=32,
                    device=f'/dev/test{index}', frame_id=f'test{index}_frame',
                    ice_service=f'collection/{index}') for index in range(count)]


def catalog(definitions, instance='instance-a', version=17):
    message = CameraCatalog()
    message.source_instance_id = instance
    message.config_version = version
    message.cameras = [descriptor(spec) for spec in definitions]
    message.configured_count = len(message.cameras)
    message.enabled_count = sum(item.enabled for item in message.cameras)
    message.online_count = sum(item.online for item in message.cameras)
    return message


def jpeg():
    image = np.zeros((32, 64, 3), np.uint8)
    image[:, :32] = (20, 80, 210)
    return cv2.imencode('.jpg', image)[1].tobytes()


def packet(spec, timestamp, capture_id=1, version=17):
    return FramePacket(FrameHeader(capture_id, timestamp, 0, spec.id, spec.width, spec.height,
                                   0, encoding=ENCODING_JPEG, camera_info_version=version), jpeg())


def fake_readers(monkeypatch, sources):
    opened = []

    def make_reader(service):
        opened.append(service)
        values = iter(sources.get(service, []) + [None])
        return SimpleNamespace(read=lambda: next(values), close=lambda: None,
                               request_stop=lambda: None)

    monkeypatch.setattr('sl_record.frames.Iceoryx2Reader', make_reader)
    return opened


def join_writers(manager):
    for writer in manager.writers.values():
        writer.thread.join(2)
        assert not writer.thread.is_alive()
    assert manager.error is None


def records(root, spec):
    path = root / 'camera/raw' / f'camera_{spec.id}_{spec.name}' / 'frames.jsonl'
    return [json.loads(line) for line in path.read_text().splitlines()] if path.is_file() else []


@pytest.mark.parametrize('count', [1, 2, 3])
def test_discovers_arbitrary_camera_count_and_original_bytes(tmp_path, monkeypatch, count):
    definitions = specs(count)
    opened = fake_readers(monkeypatch, {
        spec.ice_service: [packet(spec, 1_000_000_000)] for spec in definitions})
    manager = DiscoveredFrames(tmp_path)
    manager.on_catalog(catalog(definitions))
    join_writers(manager)
    assert set(opened) == {spec.ice_service for spec in definitions}
    for spec in definitions:
        rows = records(tmp_path, spec)
        assert len(rows) == 1
        row = rows[0]
        assert (row['camera_id'], row['camera_name'], row['config_version']) == (
            spec.id, spec.name, 17)
        assert row['source_instance_id'] == 'instance-a'
        assert (tmp_path / 'camera/raw' / f'camera_{spec.id}_{spec.name}' / row['path']).read_bytes() == jpeg()
    assert manager.stop()


@pytest.mark.parametrize('source_fps,expected', [(15, 15), (30, 30), (60, 30)])
def test_frame_limit_retains_actual_timestamps(tmp_path, monkeypatch, source_fps, expected):
    spec = specs()[0]
    times = [1_000_000_000 + round(i * 1_000_000_000 / source_fps) for i in range(source_fps)]
    fake_readers(monkeypatch, {spec.ice_service: [packet(spec, ts, i) for i, ts in enumerate(times)]})
    manager = DiscoveredFrames(tmp_path)
    manager.on_catalog(catalog([spec]))
    join_writers(manager)
    rows = records(tmp_path, spec)
    assert len(rows) == expected
    assert all(row['timestamp_ns'] == times[row['capture_id']] for row in rows)
    assert sum(manager.snapshot()['fps_skipped_frames'].values()) == source_fps - expected
    assert manager.stop()


def test_missing_disabled_and_empty_catalogs_do_not_block(tmp_path, monkeypatch):
    a, b, c = specs(3)
    c = replace(c, enabled=False)
    opened = fake_readers(monkeypatch, {a.ice_service: [packet(a, 1_000_000_000)]})
    manager = DiscoveredFrames(tmp_path)
    manager.on_catalog(catalog([a, b, c]))
    join_writers(manager)
    assert set(opened) == {a.ice_service, b.ice_service}
    assert manager.snapshot()['alignment_status'] == 'aligned'
    assert records(tmp_path, b) == []
    manager.on_catalog(catalog([], version=18))
    assert manager.writers == {}
    assert manager.stop()


def test_wrong_id_version_and_dimensions_are_rejected(tmp_path, monkeypatch):
    spec = specs()[0]
    good = packet(spec, 1_000_000_000)
    bad_id = FramePacket(replace(good.header, camera_group=99), good.payload)
    bad_version = packet(spec, 1_000_000_000, version=999)
    fake_readers(monkeypatch, {spec.ice_service: [bad_id, bad_version, good]})
    manager = DiscoveredFrames(tmp_path)
    manager.on_catalog(catalog([spec]))
    join_writers(manager)
    assert len(records(tmp_path, spec)) == 1
    assert sum(manager.snapshot()['rejected_frames'].values()) == 2
    assert manager.stop()


def test_camera_restart_and_catalog_changes_preserve_all_files(tmp_path, monkeypatch):
    a, b = specs(2)
    sources = {a.ice_service: [packet(a, 1_000_000_000)]}
    fake_readers(monkeypatch, sources)
    manager = DiscoveredFrames(tmp_path)
    manager.on_catalog(catalog([a]))
    join_writers(manager)
    sources[a.ice_service] = [packet(a, 100_000_000, version=18)]
    sources[b.ice_service] = [packet(b, 100_000_000, version=18)]
    manager.on_catalog(catalog([a, b], instance='instance-b', version=18))
    join_writers(manager)
    # A delayed message from the previous producer cannot revert the definitions.
    manager.on_catalog(catalog([a], instance='instance-a', version=17))
    rows = records(tmp_path, a)
    assert [row['frame_sequence'] for row in rows] == [0, 1]
    assert [row['source_instance_id'] for row in rows] == ['instance-a', 'instance-b']
    assert [row['timestamp_ns'] for row in rows] == [1_000_000_000, 100_000_000]
    assert len(records(tmp_path, b)) == 1
    history = [json.loads(line) for line in (tmp_path / 'metadata/camera_catalogs.jsonl').read_text().splitlines()]
    assert len(history) == 2
    assert all(row['catalog_sequence'] in (1, 2) for row in rows)
    assert manager.stop()


def test_invalid_catalog_cannot_create_unsafe_paths(tmp_path, monkeypatch):
    fake_readers(monkeypatch, {})
    message = catalog(specs())
    message.cameras[0].name = '../unsafe'
    manager = DiscoveredFrames(tmp_path)
    manager.on_catalog(message)
    assert manager.writers == {}
    assert not (tmp_path / 'camera').exists()
    assert 'catalog_rejected' in (tmp_path / 'events.jsonl').read_text()


def test_timestamp_rollback_starts_new_epoch_without_replacing_source_time(tmp_path, monkeypatch):
    spec = specs()[0]
    times = [1_000_000_000, 1_010_000_000, 100_000_000, 140_000_000, 0]
    fake_readers(monkeypatch, {
        spec.ice_service: [packet(spec, value, index) for index, value in enumerate(times)]})
    manager = DiscoveredFrames(tmp_path)
    manager.on_catalog(catalog([spec]))
    join_writers(manager)
    rows = records(tmp_path, spec)
    assert [row['timestamp_ns'] for row in rows] == [times[index] for index in (0, 2, 3, 4)]
    assert [row['timestamp_epoch'] for row in rows] == [0, 1, 1, 2]
    assert not rows[-1]['timestamp_aligned']
    assert manager.snapshot()['alignment_status'] == 'degraded'
    assert manager.stop()


def test_reusing_session_preserves_image_and_catalog_sequences(tmp_path, monkeypatch):
    spec = specs()[0]
    sources = {spec.ice_service: [packet(spec, 1_000_000_000)]}
    fake_readers(monkeypatch, sources)
    manager = DiscoveredFrames(tmp_path)
    manager.on_catalog(catalog([spec]))
    join_writers(manager)
    assert manager.stop()
    history = tmp_path / 'metadata/camera_catalogs.jsonl'
    with history.open('a') as stream:
        stream.write('{"catalog_sequence":')
    sources[spec.ice_service] = [packet(spec, 2_000_000_000, version=18)]
    manager = DiscoveredFrames(tmp_path)
    manager.on_catalog(catalog([spec], instance='new-instance', version=18))
    join_writers(manager)
    assert manager.stop()
    rows = records(tmp_path, spec)
    assert [row['frame_sequence'] for row in rows] == [0, 1]
    assert [row['catalog_sequence'] for row in rows] == [1, 2]
    entries = [json.loads(line) for line in history.read_text().splitlines()]
    assert [entry['catalog_sequence'] for entry in entries] == [1, 2]
    assert 'camera_catalog_history_recovered' in (tmp_path / 'events.jsonl').read_text()


def test_disk_failure_is_reported(tmp_path, monkeypatch):
    spec = specs()[0]
    fake_readers(monkeypatch, {spec.ice_service: [packet(spec, 1_000_000_000)]})
    original = Path.write_bytes

    def full(path, payload):
        if path.suffix == '.jpg':
            raise OSError(28, 'No space left on device')
        return original(path, payload)

    monkeypatch.setattr(Path, 'write_bytes', full)
    manager = DiscoveredFrames(tmp_path)
    manager.on_catalog(catalog([spec]))
    for writer in manager.writers.values():
        writer.thread.join(2)
    assert 'No space left' in manager.snapshot()['error']
    manager.stop()


def test_archive_start_failure_marks_session_failed_and_cleans_lock(tmp_path, monkeypatch):
    import sl_record.recorder as module

    closed = []

    def full_disk():
        raise OSError(28, 'No space left on device')

    monkeypatch.setattr(module, 'SlRecorder', lambda *_: SimpleNamespace(
        start=full_disk, stop=lambda: closed.append(True)))
    args = _parse_args(['--session-dir', str(tmp_path)])
    assert module.run_recorder(args) == 1
    manifest = json.loads((tmp_path / 'manifest.json').read_text())
    assert manifest['status'] == 'FAILED'
    assert 'No space left' in manifest['error']
    assert not (tmp_path / 'session.lock').exists()
    assert closed == [True]


def test_sl_recovery_repairs_catalog_history_and_snapshot(tmp_path):
    from sl_record.recover import recover_one
    from uv_record.session import initialise_session

    initialise_session(tmp_path)
    (tmp_path / 'session.lock').write_text(json.dumps({'pid': 99999999}))
    valid = {'catalog_sequence': 7, 'catalog': {
        'source_instance_id': 'saved-instance', 'config_version': 17, 'cameras': []}}
    history = tmp_path / 'metadata/camera_catalogs.jsonl'
    history.write_text(json.dumps(valid) + '\n{"catalog_sequence":')
    snapshot = tmp_path / 'metadata/camera_catalog.json'
    snapshot.write_text('{"torn":')
    assert recover_one(tmp_path)
    assert json.loads(history.read_text()) == valid
    assert json.loads(snapshot.read_text()) == valid
    assert json.loads((tmp_path / 'manifest.json').read_text())['status'] == 'RECOVERED'
    assert not (tmp_path / 'session.lock').exists()


def test_truncated_frame_and_index_tail_recovery(tmp_path, monkeypatch):
    spec = specs()[0]
    fake_readers(monkeypatch, {spec.ice_service: [packet(spec, 1_000_000_000),
                                                packet(spec, 1_100_000_000, 2)]})
    manager = DiscoveredFrames(tmp_path)
    manager.on_catalog(catalog([spec]))
    join_writers(manager)
    manager.stop()
    rows = records(tmp_path, spec)
    directory = tmp_path / 'camera/raw' / f'camera_{spec.id}_{spec.name}'
    (directory / rows[-1]['path']).write_bytes(jpeg()[:-20])
    with (directory / 'frames.jsonl').open('a') as stream:
        stream.write('{"torn":')
    assert recover_raw_camera(directory)
    assert len(records(tmp_path, spec)) == 1


def test_bag_settings_match_uv_and_include_custom_catalog(tmp_path, monkeypatch):
    import sl_record.recorder as module

    help_text = '--regex --exclude-topic-types --max-bag-duration --use-sim-time'
    monkeypatch.setattr(module, '_rosbag_record_help', lambda: help_text)
    monkeypatch.setattr(module, '_select_bag_storage', lambda _: 'sqlite3')
    paths = SimpleNamespace(bag=tmp_path / 'bag')
    args = _parse_args([])
    command = BagCommand(paths, args)
    uv = Recorder.__new__(Recorder)
    uv.paths, uv.args = paths, uv_parse_args([])
    uv.bag_storage, uv._bag_record_help = 'sqlite3', help_text
    assert command.command(0) == uv._bag_command(0)
    args.catalog_topic = '/custom/camera_catalog'
    command = BagCommand(paths, args)
    import re
    assert re.search(command._bag_topic_regex(), args.catalog_topic)
    assert 'sensor_msgs/msg/Image' in command.command(0)
