"""Verify camera isolation and reuse of the existing bag/session behavior."""

import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from auv_protocol.topics import ICEORYX_CAMERA_DOWN, ICEORYX_CAMERA_FRONT, ICEORYX_CAMERA_SL
from sl_record.recorder import _parse_args
from uv_image_transport import CAMERA_SL, ENCODING_JPEG, FrameHeader, FramePacket
from uv_record.raw_recorder import RawFrameRecorder
from uv_record.recorder import Recorder, _parse_args as uv_parse_args
from uv_record.recover import recover_raw_camera
from uv_record.session import initialise_session


def payload():
    return cv2.imencode('.jpg', np.zeros((32, 64, 3), np.uint8))[1].tobytes()


def test_defaults_and_raw_only():
    sl, uv = _parse_args([]), uv_parse_args([])
    assert sl.raw_cameras == ('sl', 'down')
    assert sl.optional_cameras
    assert sl.video_fps == 30.0
    assert Path(sl.output_root).parts[-2:] == ('records', 'sl_sessions')
    assert sl.record_mode == uv.record_mode == 'raw'
    assert uv.video_fps == 5.0
    assert RawFrameRecorder('/unused').cameras == ('front', 'down')
    with pytest.raises(SystemExit):
        _parse_args(['--record-mode', 'go2rtc'])
    for fps in ('0', '-1', 'nan', 'inf'):
        with pytest.raises(SystemExit):
            _parse_args(['--video-fps', fps])


def record_frames(root, timestamps, monkeypatch, capture_ids=None, down_timestamps=()):
    jpeg = payload()
    ids = capture_ids if capture_ids is not None else range(len(timestamps))
    packets = [FramePacket(FrameHeader(
        capture_id, timestamp, 0, CAMERA_SL, 64, 32, 0, encoding=ENCODING_JPEG), jpeg)
        for capture_id, timestamp in zip(ids, timestamps)]
    down_packets = [FramePacket(FrameHeader(
        index, timestamp, index, 2, 64, 32, 0, encoding=ENCODING_JPEG), jpeg)
        for index, timestamp in enumerate(down_timestamps)]
    # Front is present but excluded; down may appear independently of sl.
    front = FramePacket(FrameHeader(1, 1_000_000_000, 1, 1, 64, 32, 0,
                                    encoding=ENCODING_JPEG), jpeg)
    sources = {ICEORYX_CAMERA_FRONT: [front, None],
               ICEORYX_CAMERA_DOWN: down_packets + [None],
               ICEORYX_CAMERA_SL: packets + [None]}
    opened = []

    def reader(service):
        opened.append(service)
        values = iter(sources[service])
        return SimpleNamespace(read=lambda: next(values), close=lambda: None)

    monkeypatch.setattr('uv_record.raw_recorder.Iceoryx2Reader', reader)
    recorder = Recorder.__new__(Recorder)
    recorder.paths = SimpleNamespace(root=root)
    recorder.args = _parse_args([])
    recorder._start_raw_recorder()
    for thread in recorder.raw_recorder.threads:
        thread.join(2)
    assert recorder.raw_recorder.stop()
    assert recorder.raw_recorder.error is None
    assert set(opened) == {ICEORYX_CAMERA_SL, ICEORYX_CAMERA_DOWN}
    assert {path.name for path in (root / 'camera/raw').iterdir()} == {'sl', 'down'}
    index = root / 'camera/raw/sl/frames.jsonl'
    entries = [json.loads(line) for line in index.read_text().splitlines()]
    assert all((index.parent / entry['path']).read_bytes() == jpeg for entry in entries)
    return recorder.raw_recorder, entries


@pytest.mark.parametrize('source_fps,expected_count', [(15, 15), (30, 30), (60, 30)])
def test_available_sl_fps_and_source_timestamps(tmp_path, monkeypatch, source_fps, expected_count):
    # Round to nanoseconds, just as a source clock does at fractional frame periods.
    timestamps = [1_000_000_000 + round(index * 1_000_000_000 / source_fps)
                  for index in range(source_fps)]
    recorder, entries = record_frames(tmp_path, timestamps, monkeypatch)
    assert len(entries) == expected_count
    assert all(entry['timestamp_ns'] == timestamps[entry['capture_id']] for entry in entries)
    assert [entry['frame_sequence'] for entry in entries] == list(range(expected_count))
    assert recorder.snapshot()['fps_limit'] == 30.0
    assert recorder.snapshot()['fps_skipped_frames']['sl'] == source_fps - expected_count
    assert recorder.snapshot()['frames']['down'] == 0
    assert recorder.snapshot()['alignment_status'] == 'aligned'


def test_both_cameras_saved_independently(tmp_path, monkeypatch):
    timestamps = [1_000_000_000, 1_100_000_000]
    down_timestamps = [2_000_000_000, 2_050_000_000, 2_100_000_000]
    recorder, _ = record_frames(tmp_path, timestamps, monkeypatch,
                                down_timestamps=down_timestamps)
    down_dir = tmp_path / 'camera/raw/down'
    records = [json.loads(line) for line in (down_dir / 'frames.jsonl').read_text().splitlines()]
    assert [record['timestamp_ns'] for record in records] == down_timestamps
    assert all(record['camera_group'] == 'down' for record in records)
    assert all((down_dir / record['path']).read_bytes() == payload() for record in records)
    assert recorder.snapshot()['frames'] == {'sl': 2, 'down': 3}
    assert recorder.snapshot()['available_cameras'] == ['sl', 'down']
    assert recorder.snapshot()['alignment_status'] == 'aligned'


def test_only_down_available(tmp_path, monkeypatch):
    recorder, sl_entries = record_frames(
        tmp_path, [], monkeypatch, down_timestamps=[2_000_000_000, 2_050_000_000])
    assert sl_entries == []
    assert recorder.snapshot()['frames'] == {'sl': 0, 'down': 2}
    assert recorder.snapshot()['available_cameras'] == ['down']
    assert recorder.snapshot()['waiting_for_cameras'] == ['sl']
    assert recorder.snapshot()['alignment_status'] == 'aligned'


def test_restart_and_clock_reset_keep_unique_images(tmp_path, monkeypatch):
    _, entries = record_frames(
        tmp_path, [1_000_000_000, 1_100_000_000, 100_000_000, 200_000_000],
        monkeypatch, capture_ids=[12, 13, 1, 2])
    assert [entry['capture_id'] for entry in entries] == [12, 13, 1, 2]
    assert [entry['timestamp_epoch'] for entry in entries] == [0, 0, 1, 1]
    assert len({entry['path'] for entry in entries}) == 4


@pytest.mark.parametrize('modern_cli', [True, False])
def test_bag_command_matches_uv_record(tmp_path, monkeypatch, modern_cli):
    commands = []
    monkeypatch.setattr('uv_record.recorder._discover_ros_topics',
                        lambda _: ['/auv/camera/sl/status', '/auv/state/odom'])
    for args in (uv_parse_args([]), _parse_args([])):
        recorder = Recorder.__new__(Recorder)
        recorder.paths = SimpleNamespace(bag=tmp_path / 'bag')
        recorder.args = args
        recorder.bag_storage = 'sqlite3'
        recorder._bag_record_help = (
            '--regex --exclude-topic-types --max-bag-duration '
            '--disable-keyboard-controls --use-sim-time' if modern_cli else '')
        commands.append(recorder._bag_command(0))
    assert commands[0] == commands[1]
    if modern_cli:
        assert '--exclude-topic-types' in commands[0]
        assert 'sensor_msgs/msg/Image' in commands[0]
        assert 'sensor_msgs/msg/CompressedImage' in commands[0]
        assert 'stereo_msgs/msg/DisparityImage' in commands[0]


def test_sl_recovery_keeps_complete_jpeg_and_cleans_torn_index(tmp_path, monkeypatch):
    _, entries = record_frames(tmp_path, [1_000_000_000, 1_100_000_000], monkeypatch)
    directory = tmp_path / 'camera/raw/sl'
    (directory / entries[-1]['path']).write_bytes(payload()[:-20])
    with (directory / 'frames.jsonl').open('a') as handle:
        handle.write('{"partial":')
    assert recover_raw_camera(directory)
    kept = [json.loads(line) for line in (directory / 'frames.jsonl').read_text().splitlines()]
    assert kept == entries[:1]


def test_sl_mcap_exports_source_bytes_and_times(tmp_path, monkeypatch):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    from uv_record.mcap_export import export_session_mcap
    from uv_record.recorder import _mcap_storage_available

    if not _mcap_storage_available():
        pytest.skip('MCAP storage plugin is optional')
    initialise_session(tmp_path)
    timestamps = [5_000_000_000, 5_100_000_000]
    _, entries = record_frames(tmp_path, timestamps, monkeypatch)
    result = export_session_mcap(tmp_path, 'raw', fps=30)
    assert result['camera_streams'] == ['sl']
    assert result['camera_frames'] == 2
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(tmp_path / 'session.mcap'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    images, metadata = [], []
    try:
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic.endswith('/compressed'):
                assert topic == '/auv/record/camera/sl/compressed'
                image = deserialize_message(data, CompressedImage)
                stamp = image.header.stamp
                assert stamp.sec * 1_000_000_000 + stamp.nanosec == timestamp
                assert bytes(image.data) == payload()
                images.append(timestamp)
            elif topic.endswith('/frame_metadata'):
                metadata.append(json.loads(deserialize_message(data, String).data))
    finally:
        reader.close()
    assert images == timestamps
    assert metadata == entries


def test_stop_without_any_camera_frames(tmp_path, monkeypatch):
    import time
    import uuid
    from uv_record import raw_recorder

    pytest.importorskip('iceoryx2')
    monkeypatch.setitem(raw_recorder.SERVICES, 'sl', 'youlong/test/empty/' + uuid.uuid4().hex)
    recorder = RawFrameRecorder(tmp_path, cameras=('sl',), fps=30, optional_cameras=True)
    recorder.start()
    try:
        deadline = time.monotonic() + 3
        while not recorder.readers and time.monotonic() < deadline:
            time.sleep(0.01)
        assert recorder.readers
    finally:
        stopped = recorder.stop(timeout=2)
    assert stopped
    assert recorder.error is None
    assert recorder.snapshot()['frames'] == {'sl': 0}
    assert recorder.snapshot()['alignment_status'] == 'waiting'
