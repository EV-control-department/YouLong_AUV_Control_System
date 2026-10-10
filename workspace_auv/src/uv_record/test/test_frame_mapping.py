"""Timestamp mapping contracts shared by go2rtc and raw frame records."""

from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from uv_image_transport import ENCODING_JPEG, FrameHeader, FramePacket

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
    encoded = cv2.imencode('.jpg', np.zeros((16, 32, 3), dtype=np.uint8))[1].tobytes()
    packet = FramePacket(
        FrameHeader(42, 4_321_000_000, 40, 1, 32, 16, 0,
                    encoding=ENCODING_JPEG, camera_info_version=7), encoded)

    class Reader:
        def __init__(self, _service):
            self.values = [packet, packet, None]

        def read(self):
            return self.values.pop(0)

        def close(self):
            pass

    monkeypatch.setattr('uv_record.raw_recorder.Iceoryx2Reader', Reader)
    for function in ('imwrite', 'imencode', 'imdecode'):
        monkeypatch.setattr(cv2, function, lambda *_a, **_k: pytest.fail('raw used a pixel codec'))
    monkeypatch.setattr(FramePacket, 'bgr', lambda *_: pytest.fail('raw decoded BGR'))
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
    assert item['path'] == 'frame_00000000000000000000.jpg'
    assert items[1]['capture_id'] == 42
    assert items[1]['path'] == 'frame_00000000000000000001.jpg'
    assert item['stereo_pair_id'] == 40
    assert item['camera_info_version'] == 7
    assert item['stride'] == 0
    assert item['encoding'] == 'JPEG'
    assert item['probe_version'] == 2
    assert (index.parent / item['path']).read_bytes() == encoded
    assert recorder.error is None
    assert 'jpeg_write_ms' in item['probe']
    assert 'packet_to_bgr_ms' not in item['probe']
    assert recorder.snapshot()['diagnostics']['front']['total_jpeg_bytes'] == 2 * len(encoded)
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


def test_jpeg_raw_player_and_legacy_png_mcap_formats(tmp_path):
    import json
    from uv_record.mcap_export import _camera_records, _camera_sources
    directory = tmp_path / 'camera' / 'raw' / 'front'
    directory.mkdir(parents=True)
    image = np.zeros((16, 32, 3), np.uint8)
    image[:, :16] = (200, 20, 10)
    records = []
    for index, extension in enumerate(('jpg', 'png', 'jpg')):
        name = f'frame_{index}.{extension}'
        assert cv2.imwrite(str(directory / name), image)
        records.append({'path': name, 'timestamp_ns': (100, 200, 10)[index],
                        'timestamp_epoch': int(index == 2), 'capture_id': index})
    (directory / 'frames.jsonl').write_text(''.join(json.dumps(x) + '\n' for x in records))
    video = RawFrameVideo(directory, 10)
    assert video._times_ns[-1] > video._times_ns[-2]
    frame = video.read_to_ns(video._times_ns[-1])
    assert frame.shape == image.shape
    assert frame[8, 4, 0] > frame[8, 20, 0] + 150
    source, = _camera_sources(tmp_path, 'raw')
    emitted = list(_camera_records(source, 10, 2, lambda message: message))
    images = [message for _, topic, message in emitted if topic.endswith('/compressed')]
    assert [message.format for message in images] == ['jpeg', 'png', 'jpeg']
    for record, message in zip(records, images):
        assert bytes(message.data) == (directory / record['path']).read_bytes()
    metadata = [json.loads(message.data) for _, topic, message in emitted
                if topic.endswith('/frame_metadata')]
    assert [item['capture_id'] for item in metadata] == [0, 1, 2]
    assert [item['format'] for item in metadata] == ['jpeg', 'png', 'jpeg']


def test_raw_discards_bad_jpeg_and_preserves_clock_reset(tmp_path, monkeypatch):
    from dataclasses import replace
    from uv_image_transport import InvalidFrameError
    import json
    payload = cv2.imencode('.jpg', np.zeros((16, 32, 3), np.uint8))[1].tobytes()
    first = FramePacket(FrameHeader(9, 1_000_000_000, 8, 1, 32, 16, 0,
                                    encoding=ENCODING_JPEG), payload)
    reset = FramePacket(replace(first.header, capture_id=1, stereo_pair_id=1,
                                timestamp_ns=100_000_000), payload)
    values = iter([InvalidFrameError('bad transport sample'),
                   FramePacket(first.header, payload[:-2]), first, reset, None])

    def read():
        value = next(values)
        if isinstance(value, Exception):
            raise value
        return value

    reader = SimpleNamespace(read=read, close=lambda: None)
    monkeypatch.setattr('uv_record.raw_recorder.Iceoryx2Reader', lambda _: reader)
    recorder = RawFrameRecorder(tmp_path, cameras=('front',))
    recorder.start()
    recorder.threads[0].join(timeout=2)
    assert recorder.stop()
    assert recorder.error is None
    index = tmp_path / 'camera/raw/front/frames.jsonl'
    items = [json.loads(line) for line in index.read_text().splitlines()]
    assert [item['timestamp_epoch'] for item in items] == [0, 1]
    assert [item['capture_id'] for item in items] == [9, 1]
    assert [item['frame_sequence'] for item in items] == [0, 1]
    assert all((index.parent / item['path']).read_bytes() == payload for item in items)


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
