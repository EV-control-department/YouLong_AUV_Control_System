"""Regressions for incomplete indexes, torn writes and incremental syncing."""

import os
from unittest.mock import patch

from uv_log.jpeg_archive import JpegArchiveWriter, load_chunk_entries, read_payload
from uv_log.performance import ProcessSampler
from uv_log.recorder import SegmentSyncer


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
    with patch('uv_log.jpeg_archive.scan_chunk', side_effect=AssertionError('unexpected full scan')):
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
    with patch('uv_log.recorder._sync_file', return_value=True) as sync:
        syncer.sync_once()
        sync.assert_called_once_with(bag)
        bag.write_bytes(b'first second')
        syncer.sync_once()
        assert sync.call_count == 2
        syncer.sync_all()
        assert sync.call_count == 2


def test_sampler_reports_current_process_without_commandline_secrets():
    sampler = ProcessSampler(os.getpid())
    snapshot = sampler.sample()
    process = next(p for p in snapshot['processes'] if p['pid'] == os.getpid())
    assert process['rss_bytes'] > 0
    assert 'cmdline' not in process
    second = sampler.sample()
    current = next(p for p in second['processes'] if p['pid'] == os.getpid())
    assert current['cpu_percent'] is not None
