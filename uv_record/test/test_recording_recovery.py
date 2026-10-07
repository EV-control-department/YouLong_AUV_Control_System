"""Regressions for incomplete indexes, torn writes and incremental syncing."""

import json
import os
import sqlite3
from unittest.mock import patch

from uv_record.jpeg_archive import (
    JpegArchiveWriter, archive_entries, load_chunk_entries, read_payload,
)
from uv_record.player import BagPlayback, _bag_part_paths
from uv_record.performance import ProcessSampler
from uv_record.recorder import SegmentSyncer
from uv_record.recover import recover_frame_alignment, recover_raw_camera


def test_competition_profile_limits_and_explicit_overrides(monkeypatch):
    from uv_record.recorder import _parse_args
    monkeypatch.setattr('sys.argv', ['record'])
    args = _parse_args()
    assert args.profile == 'competition'
    assert args.video_source == 'mjpeg' and args.port == 8090
    assert args.video_format == 'ts' and args.video_bitrate_kbps == 1200
    assert args.video_fps == 8 and args.video_width == 960
    assert args.duration_minutes == 60 and args.max_session_gb == 4.8
    assert args.go2rtc_stream_mode == 'unannotated'
    monkeypatch.setattr('sys.argv', ['record', '--profile', 'dataset', '--video-fps', '3'])
    args = _parse_args()
    assert args.video_format == 'jpeg' and args.video_width == 0
    assert args.video_fps == 3 and args.max_session_gb == 0


def test_video_command_actually_limits_fps_width_and_bitrate():
    from types import SimpleNamespace
    from uv_record.mjpeg_proxy import _ts_command, _jpeg_command
    args = SimpleNamespace(fps=8, max_width=960, bitrate_kbps=1200,
                           ffmpeg='ffmpeg', url='http://localhost:8090/front',
                           output_dir='/tmp/video', segment_duration=60,
                           video_codec='libx264', start_number=0, playlist='/tmp/video/index.m3u8')
    command = _ts_command(args)
    filters = command[command.index('-vf')+1]
    assert 'select=' in filters and '0.125' in filters and 'scale=' in filters
    assert command[command.index('-maxrate')+1] == '1200k'
    assert command[command.index('-threads')+1] == '2'
    assert '-fps_mode' not in _jpeg_command(args)  # FFmpeg on Foxy supports -vsync.
    args.wallclock_input = True
    command = _ts_command(args)
    assert '-start_at_zero' in command
    assert command.index('-use_wallclock_as_timestamps') < command.index('-i')


def test_session_size_and_replay_telemetry(tmp_path):
    from uv_record.recorder import _session_size
    from uv_record.telemetry import ReplayTelemetry
    metadata = tmp_path / 'metadata'
    metadata.mkdir()
    pose = {'receive_time_unix_ns': 100, 'pose':
            {'x': 1, 'y': 2, 'z': .3, 'yaw': 90, 'roll': 0, 'pitch': 0}}
    (metadata / 'trajectory.jsonl').write_text(json.dumps(pose)+'\n')
    mapping = {'receive_time_unix_ns': 100, 'state': 'complete', 'cells':
               {'4': {'position': [1.1, 2.2, .8], 'label': 'round_cone', 'source': 'vision'}}}
    (metadata / 'mapping.jsonl').write_text(json.dumps(mapping)+'\n')
    assert _session_size(tmp_path) > 0
    replay = ReplayTelemetry(tmp_path)
    assert replay.at('pose', 99) is None
    assert replay.at('pose', 100)['pose']['yaw'] == 90
    image, log = replay.render(100)
    assert image.shape == (480, 640, 3)
    assert image.max() > 28


def test_player_never_registers_actuator_or_action_topics():
    player = BagPlayback.__new__(BagPlayback)
    for topic in ('/zit6/cmd/setpoint', '/zit6/cmd/servo', '/cmd_vel',
                  '/basic_motion/_action/feedback'):
        assert not player._register_topic(topic, 'std_msgs/msg/String')


def test_dataset_export_splits_both_cameras_without_overwrite(tmp_path, monkeypatch):
    import cv2
    import numpy as np
    from uv_record.player import export_frames
    session = tmp_path / 'session'
    for camera in ('front', 'down'):
        image = np.full((48, 128, 3), 100, np.uint8)
        ok, jpeg = cv2.imencode('.jpg', image)
        assert ok
        writer = JpegArchiveWriter(session/'video'/camera, fps=2, chunk_seconds=60)
        writer.write(jpeg.tobytes(), timestamp_ns=1234, sequence=0,
                     metadata={'timestamp_aligned': False})
        writer.close()
    output = tmp_path / 'export'
    monkeypatch.setattr('sys.argv', ['export_frames', str(session), '--output', str(output), '--split-stereo'])
    assert export_frames() == 0
    for camera in ('front_left', 'front_right', 'down_left', 'down_right'):
        images = list((output/camera).glob('*.png'))
        assert len(images) == 1
        assert cv2.imread(str(images[0])).shape == (48, 64, 3)
        assert (output/camera/'frames.jsonl').is_file()


def test_compact_map_snapshots_keep_labels_and_bounded_point_sets(tmp_path):
    from types import SimpleNamespace
    from uv_record.telemetry import TelemetryRecorder
    recorder = TelemetryRecorder(tmp_path)
    written = []
    recorder._write = lambda kind, value, rate: written.append(value)
    recorder._map(SimpleNamespace(data=json.dumps({'state': 'complete', 'cells': [
        {'id': 4, 'label': 'round_cone', 'measurements': [{'position': [1, 2, 3]}]*120}
    ]})))
    assert written[0]['cells']['0']['label'] == 'round_cone'
    assert len(written[0]['cells']['0']['measurements']) == 20


def archive(tmp_path):
    writer = JpegArchiveWriter(tmp_path, fps=10, chunk_seconds=2)
    for i in range(3):
        writer.write(b'\xff\xd8' + bytes([i]) * 100 + b'\xff\xd9', i + 1, i)
    writer.close()
    return tmp_path / 'chunk_000000.mjpg'


def test_missing_index_tail_recovers_every_complete_frame(tmp_path):
    chunk = archive(tmp_path)
    index = chunk.with_suffix('.jsonl')
    index.write_text(index.read_text().splitlines()[0] + '\n')
    entries = load_chunk_entries(chunk)
    assert [e.sequence for e in entries] == [0, 1, 2]
    assert all(read_payload(e) is not None for e in entries)


def test_torn_last_frame_keeps_valid_prefix(tmp_path):
    chunk = archive(tmp_path)
    with chunk.open('r+b') as handle:
        handle.truncate(chunk.stat().st_size - 20)
    assert [e.sequence for e in load_chunk_entries(chunk)] == [0, 1]


def test_complete_index_does_not_rescan_video_payload(tmp_path):
    chunk = archive(tmp_path)
    with patch('uv_record.jpeg_archive.scan_chunk', side_effect=AssertionError('unexpected full scan')):
        assert len(load_chunk_entries(chunk)) == 3


def test_gap_in_index_recovers_missing_middle_frame(tmp_path):
    chunk = archive(tmp_path)
    index = chunk.with_suffix('.jsonl')
    lines = index.read_text().splitlines(keepends=True)
    index.write_text(lines[0] + lines[2])
    assert [e.sequence for e in load_chunk_entries(chunk)] == [0, 1, 2]


def test_syncer_syncs_growing_tail_and_skips_unchanged_shutdown_files(tmp_path):
    bag = tmp_path / 'part_0.mcap'
    bag.write_bytes(b'first')
    syncer = SegmentSyncer([tmp_path], patterns=('*.mcap',))
    with patch('uv_record.recorder._sync_file', return_value=True) as sync:
        syncer.sync_once()
        sync.assert_called_once_with(bag)
        bag.write_bytes(b'first second')
        syncer.sync_once()
        assert sync.call_count == 2
        syncer.sync_all()
        assert sync.call_count == 2


def test_bag_discovery_accepts_foxy_sqlite_and_jazzy_mcap(tmp_path):
    (tmp_path / 'part_000').mkdir()
    (tmp_path / 'part_000' / 'bag_0.db3').write_bytes(b'sqlite')
    (tmp_path / 'part_001').mkdir()
    (tmp_path / 'part_001' / 'bag_1.mcap').write_bytes(b'mcap')
    assert [path.suffix for path in _bag_part_paths(tmp_path)] == [
        '.db3', '.mcap']


def test_sqlite_bag_playback_without_rosbag2_py(tmp_path):
    part = tmp_path / 'part_000'
    part.mkdir()
    bag = part / 'bag_0.db3'
    connection = sqlite3.connect(str(bag))
    connection.executescript(
        'CREATE TABLE topics ('
        'id INTEGER PRIMARY KEY, name TEXT, type TEXT, '
        'serialization_format TEXT, offered_qos_profiles TEXT);'
        'CREATE TABLE messages ('
        'id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, '
        'data BLOB);')
    connection.execute(
        'INSERT INTO topics VALUES (1, "/test", "std_msgs/msg/String", '
        '"cdr", "")')
    connection.execute(
        'INSERT INTO messages VALUES (1, 1, 100, ?)',
        (sqlite3.Binary(b'payload'),))
    connection.commit()
    connection.close()

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class Node:
        def __init__(self):
            self.publishers = {}

        def create_publisher(self, _message_type, topic, _qos):
            publisher = Publisher()
            self.publishers[topic] = publisher
            return publisher

    node = Node()
    modules = (
        object(), None, lambda payload, _type: payload,
        lambda _type: object())
    with patch('uv_record.player._rosbag_modules', return_value=modules):
        playback = BagPlayback(tmp_path, node)
        assert playback.topic_count == 1
        assert playback.start_ns == 100
        assert playback.publish_until(100) == 1
        assert node.publishers['/test'].messages == [b'payload']
        playback.close()


def test_sampler_reports_current_process_without_commandline_secrets():
    sampler = ProcessSampler(os.getpid())
    snapshot = sampler.sample()
    process = next(p for p in snapshot['processes'] if p['pid'] == os.getpid())
    assert process['rss_bytes'] > 0
    assert 'cmdline' not in process
    second = sampler.sample()
    current = next(p for p in second['processes'] if p['pid'] == os.getpid())
    assert current['cpu_percent'] is not None


def test_jpeg_archive_persists_source_frame_alignment_metadata(tmp_path):
    writer = JpegArchiveWriter(tmp_path, fps=10, chunk_seconds=2)
    metadata = {
        'source_timestamp_ns': 5_123_000_000,
        'timestamp_aligned': True,
        'capture_id': 14,
        'stereo_pair_id': 9,
        'timestamp_epoch': 2,
    }
    writer.write(
        b'\xff\xd8' + b'frame' + b'\xff\xd9',
        timestamp_ns=5_123_000_000, sequence=7, metadata=metadata)
    writer.close()

    entry = archive_entries(tmp_path)[0]
    assert entry.timestamp_ns == 5_123_000_000
    assert entry.metadata['capture_id'] == 14
    assert entry.metadata['stereo_pair_id'] == 9
    assert entry.metadata['timestamp_epoch'] == 2

def test_raw_recovery_discards_missing_and_truncated_frames(tmp_path):
    import cv2
    import numpy as np

    directory = tmp_path / 'camera' / 'raw' / 'front'
    directory.mkdir(parents=True)
    complete = directory / 'complete.png'
    assert cv2.imwrite(str(complete), np.zeros((2, 3, 3), dtype=np.uint8))
    truncated = directory / 'truncated.png'
    truncated.write_bytes(b'\x89PNG\r\n\x1a\npartial')
    records = [
        {'path': 'complete.png', 'timestamp_ns': 100, 'width': 3, 'height': 2},
        {'path': 'truncated.png', 'timestamp_ns': 200, 'width': 3, 'height': 2},
        {'path': 'missing.png', 'timestamp_ns': 300, 'width': 3, 'height': 2},
    ]
    index = directory / 'frames.jsonl'
    index.write_text(''.join(json.dumps(item) + '\n' for item in records))

    assert recover_raw_camera(directory)
    recovered = [json.loads(line) for line in index.read_text().splitlines()]
    assert [item['path'] for item in recovered] == ['complete.png']

def test_frame_alignment_recovery_discards_partial_jsonl_tail(tmp_path):
    directory = tmp_path / 'video' / 'front'
    directory.mkdir(parents=True)
    index = directory / 'frame_alignment.jsonl'
    record = {'sequence': 4, 'receive_time_unix_ns': 500,
              'source_timestamp_ns': None, 'timestamp_aligned': False}
    index.write_text(json.dumps(record) + '\n{"sequence":')

    assert recover_frame_alignment(directory)
    assert [json.loads(line) for line in index.read_text().splitlines()] == [record]
