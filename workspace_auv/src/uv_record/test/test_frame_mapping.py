"""Timestamp mapping contracts shared by go2rtc and raw frame records."""

from types import SimpleNamespace

import cv2
import numpy as np

from uv_record.frame_mapping import FrameMappingSubscriber
from uv_record.mjpeg_proxy import (
    _alignment_record, _parse_showinfo, _pts_for_frame,
)
from uv_record.player import RawFrameVideo
from uv_record.raw_recorder import RawFrameRecorder
from uv_record.recorder import Recorder, RAW_STREAMS, _select_bag_storage


def _message(instance, pts, stamp, epoch=0, sequence=0):
    sec, nanosec = divmod(stamp, 1_000_000_000)
    return SimpleNamespace(
        stream_instance_id=instance,
        camera_name='front',
        stream_mode='unannotated',
        frame_sequence=sequence,
        presentation_timestamp_ns=pts,
        source_stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
        capture_id=sequence + 10,
        stereo_pair_id=sequence + 20,
        timestamp_epoch=epoch,
        output_fps=10.0,
    )


def _mapping_cache():
    mapping = FrameMappingSubscriber.__new__(FrameMappingSubscriber)
    mapping.camera = 'front'
    mapping.stream_mode = 'unannotated'
    import threading
    mapping.lock = threading.Lock()
    mapping.maps = {}
    mapping.latest_instance = None
    mapping.active_instance = None
    mapping.last_pts_ns = None
    mapping.awaiting_generation = False
    return mapping


def test_go2rtc_pts_maps_to_source_stamp_and_pair_ids():
    mapping = _mapping_cache()
    mapping._on_mapping(_message('stream-a', 1_000_000_000, 5_123_000_000, sequence=3))
    result = mapping.match(1_003_000_000)
    assert result['source_timestamp_ns'] == 5_123_000_000
    assert result['capture_id'] == 13
    assert result['stereo_pair_id'] == 23


def test_pts_reset_selects_new_stream_generation_and_epoch():
    mapping = _mapping_cache()
    mapping._on_mapping(_message('stream-a', 0, 9_000_000_000, sequence=0))
    mapping._on_mapping(_message('stream-a', 100_000_000, 9_100_000_000, sequence=1))
    assert mapping.match(100_000_000)['stream_instance_id'] == 'stream-a'

    mapping._on_mapping(_message(
        'stream-b', 0, 10_000_000, epoch=1, sequence=0))
    result = mapping.match(5_000_000)
    assert result['stream_instance_id'] == 'stream-b'
    assert result['source_timestamp_ns'] == 10_000_000
    assert result['timestamp_epoch'] == 1


def test_unmatched_pts_does_not_substitute_receive_time():
    mapping = _mapping_cache()
    mapping._on_mapping(_message('stream-a', 1_000_000_000, 5_000_000_000))
    assert mapping.match(2_000_000_000) is None


def test_raw_recorder_writes_header_timestamp_and_camera_metadata(tmp_path, monkeypatch):
    packet = SimpleNamespace(
        header=SimpleNamespace(
            timestamp_ns=4_321_000_000, capture_id=42, stereo_pair_id=40,
            camera_info_version=7, width=2, height=1, stride=6),
        bgr=lambda: np.zeros((1, 2, 3), dtype=np.uint8),
    )

    class Reader:
        def __init__(self, _service):
            self.values = [packet, packet, None]

        def read(self):
            return self.values.pop(0)

        def close(self):
            pass

    monkeypatch.setattr('uv_record.raw_recorder.Iceoryx2Reader', Reader)
    monkeypatch.setattr(cv2, 'imwrite', lambda *_args, **_kwargs: True)
    recorder = RawFrameRecorder(tmp_path, cameras=('front',))
    recorder.start()
    recorder.threads[0].join(timeout=2.0)
    recorder.stop()

    index = tmp_path / 'camera' / 'raw' / 'front' / 'frames.jsonl'
    import json
    items = [json.loads(line) for line in index.read_text().splitlines()]
    item = items[0]
    assert item['timestamp_ns'] == 4_321_000_000
    assert item['capture_id'] == 42
    assert item['frame_sequence'] == 0
    assert item['path'] == 'frame_00000000000000000000.png'
    assert items[1]['capture_id'] == 42
    assert items[1]['path'] == 'frame_00000000000000000001.png'
    assert item['stereo_pair_id'] == 40
    assert item['camera_info_version'] == 7
    assert item['stride'] == 6
    assert item['timestamp_aligned'] is True


def test_raw_player_normalizes_source_clock_reset(tmp_path):
    camera_dir = tmp_path / 'front'
    camera_dir.mkdir()
    frame = np.zeros((2, 2, 3), dtype=np.uint8)
    for name in ('a.png', 'b.png', 'c.png'):
        assert cv2.imwrite(str(camera_dir / name), frame)
    import json
    records = [
        {'path': 'a.png', 'timestamp_ns': 1_000_000_000,
         'timestamp_epoch': 0},
        {'path': 'b.png', 'timestamp_ns': 1_100_000_000,
         'timestamp_epoch': 0},
        {'path': 'c.png', 'timestamp_ns': 50_000_000,
         'timestamp_epoch': 1},
    ]
    (camera_dir / 'frames.jsonl').write_text(
        ''.join(json.dumps(item) + '\n' for item in records))
    video = RawFrameVideo(camera_dir, 10.0)
    assert video._times_ns == [1_000_000_000, 1_100_000_000, 1_200_000_000]
    assert video.read_to_ns(1_200_000_000) is not None
    video.close()


def test_recorded_stream_choice_is_exclusive():
    recorder = Recorder.__new__(Recorder)
    recorder.record_mode = 'raw'
    recorder.args = SimpleNamespace(go2rtc_stream_mode='both')
    assert recorder._recorded_streams() == {}

    recorder.record_mode = 'go2rtc'
    recorder.args = SimpleNamespace(go2rtc_stream_mode='unannotated')
    assert recorder._recorded_streams() == RAW_STREAMS


def test_auto_bag_storage_falls_back_to_sqlite3_without_mcap(monkeypatch):
    monkeypatch.setattr(
        'uv_record.recorder._mcap_storage_available', lambda: False)
    assert _select_bag_storage('auto') == 'sqlite3'


def test_unmapped_go2rtc_frame_is_explicitly_degraded():
    mapping = _mapping_cache()
    record = _alignment_record(mapping, sequence=7, pts_ns=700_000_000)
    assert record['source_timestamp_ns'] is None
    assert record['timestamp_ns'] == 0
    assert record['timestamp_aligned'] is False
    assert record['receive_time_unix_ns'] > 0



def test_showinfo_pts_is_matched_by_decoder_frame_number():
    import queue

    pts_queue = queue.Queue()
    pending = {}
    pts_queue.put((1, 100_000_000))
    assert _pts_for_frame(pts_queue, pending, 0, timeout=0.001) is None
    assert _pts_for_frame(pts_queue, pending, 1, timeout=0.001) == 100_000_000

    frame_index, pts_ns = _parse_showinfo(
        b'[Parsed_showinfo_0] n: 12 pts: 900 pts_time:0.900000'
    )
    assert frame_index == 12
    assert pts_ns == 900_000_000

def test_pts_reset_waits_for_late_stream_generation_mapping():
    mapping = _mapping_cache()
    mapping._on_mapping(_message('stream-a', 0, 9_000_000_000, sequence=0))
    mapping._on_mapping(_message(
        'stream-a', 100_000_000, 9_100_000_000, sequence=1))
    assert mapping.match(100_000_000)['stream_instance_id'] == 'stream-a'
    assert mapping.match(0) is None

    mapping._on_mapping(_message(
        'stream-b', 0, 10_000_000, epoch=1, sequence=0))
    mapping._on_mapping(_message(
        'stream-b', 100_000_000, 110_000_000, epoch=1, sequence=1))
    result = mapping.match(100_000_000)
    assert result['stream_instance_id'] == 'stream-b'
    assert result['source_timestamp_ns'] == 110_000_000
