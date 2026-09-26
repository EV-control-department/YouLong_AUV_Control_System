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
