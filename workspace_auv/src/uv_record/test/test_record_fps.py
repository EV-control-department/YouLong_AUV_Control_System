"""Check actual frame sampling, timestamp retention and encoder output."""

from dataclasses import replace
import json
import shutil
import subprocess
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from uv_image_transport import ENCODING_JPEG, FrameHeader, FramePacket
from uv_record.mjpeg_proxy import _jpeg_command, _parse_showinfo, _ts_command
from uv_record.raw_recorder import RawFrameRecorder
from uv_record.recorder import Recorder, _parse_args


def test_raw_samples_each_camera_at_five_fps_and_preserves_reset(tmp_path, monkeypatch):
    payload = cv2.imencode('.jpg', np.zeros((16, 32, 3), np.uint8))[1].tobytes()
    header = FrameHeader(1, 1_000_000_000, 1, 1, 32, 16, 0,
                         encoding=ENCODING_JPEG)

    class Reader:
        def __init__(self, _service):
            self.packets = iter([
                FramePacket(replace(header, capture_id=i, stereo_pair_id=i,
                                    timestamp_ns=1_000_000_000 + i * 40_000_000), payload)
                for i in range(25)
            ] + [FramePacket(replace(header, timestamp_ns=100_000_000), payload), None])

        def read(self):
            return next(self.packets)

        def close(self):
            pass

    monkeypatch.setattr('uv_record.raw_recorder.Iceoryx2Reader', Reader)
    recorder = RawFrameRecorder(tmp_path)
    recorder.start()
    for thread in recorder.threads:
        thread.join(2)
    assert recorder.stop()
    assert recorder.error is None
    for camera in ('front', 'down'):
        directory = tmp_path / 'camera/raw' / camera
        records = [json.loads(line) for line in (directory / 'frames.jsonl').read_text().splitlines()]
        assert [record['timestamp_ns'] for record in records] == [
            1_000_000_000, 1_200_000_000, 1_400_000_000, 1_600_000_000,
            1_800_000_000, 100_000_000]
        assert [record['timestamp_epoch'] for record in records] == [0] * 5 + [1]
        assert [record['capture_id'] for record in records[:5]] == [0, 5, 10, 15, 20]
        assert all((directory / record['path']).read_bytes() == payload for record in records)
        assert len(list(directory.glob('*.jpg'))) == 6
    assert recorder.snapshot()['fps_limit'] == 5.0
    assert recorder.snapshot()['fps_skipped_frames'] == {'front': 20, 'down': 20}


def test_raw_uses_monotonic_sampling_without_source_stamp(tmp_path, monkeypatch):
    payload = cv2.imencode('.jpg', np.zeros((16, 32, 3), np.uint8))[1].tobytes()
    header = FrameHeader(1, 0, 1, 1, 32, 16, 0, encoding=ENCODING_JPEG)
    packet = FramePacket(header, payload)
    values = iter([packet] * 25 + [None])
    clock = [1_000_000_000]

    def read():
        clock[0] += 40_000_000
        return next(values)

    monkeypatch.setattr('uv_record.raw_recorder.Iceoryx2Reader',
                        lambda _: SimpleNamespace(read=read, close=lambda: None))
    monkeypatch.setattr('uv_record.raw_recorder.time.perf_counter_ns', lambda: clock[0])
    recorder = RawFrameRecorder(tmp_path, cameras=('front',))
    recorder.start()
    recorder.threads[0].join(2)
    assert recorder.stop()
    assert recorder.error is None
    assert recorder.snapshot()['frames'] == {'front': 5}
    assert recorder.snapshot()['unaligned_frames'] == {'front': 5}


def test_recording_limit_is_forwarded_to_raw_worker(tmp_path, monkeypatch):
    recorder = Recorder.__new__(Recorder)
    recorder.paths = SimpleNamespace(root=tmp_path)
    recorder.args = SimpleNamespace(video_fps=5.0)
    seen = []
    monkeypatch.setattr('uv_record.recorder.RawFrameRecorder',
                        lambda root, fps: SimpleNamespace(start=lambda: seen.append((root, fps))))
    recorder._start_raw_recorder()
    assert seen == [(tmp_path, 5.0)]


def test_cli_defaults_to_five_fps(monkeypatch):
    monkeypatch.setattr('sys.argv', ['record'])
    assert _parse_args().video_fps == 5.0


@pytest.mark.parametrize('mode', ['jpeg', 'ts'])
def test_real_ffmpeg_outputs_five_fps_without_retiming(tmp_path, mode):
    if not shutil.which('ffmpeg') or not shutil.which('ffprobe'):
        pytest.skip('FFmpeg/ffprobe not installed')
    source = tmp_path / 'source.mkv'
    subprocess.run([
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-f', 'lavfi',
        '-i', 'testsrc2=size=32x16:rate=30:duration=2', '-c:v', 'ffv1',
        str(source),
    ], check=True, capture_output=True, timeout=15)
    args = SimpleNamespace(
        ffmpeg='ffmpeg', url=str(source), fps=5.0, output_dir=str(tmp_path),
        segment_duration=1.0, video_codec='libx264', start_number=0,
        playlist=str(tmp_path / 'index.m3u8'))
    result = subprocess.run(
        _jpeg_command(args) if mode == 'jpeg' else _ts_command(args),
        check=True, capture_output=True, timeout=15)
    pts = [pts_ns for n, pts_ns in
           (_parse_showinfo(line) for line in result.stderr.splitlines())
           if n is not None]
    assert len(pts) == 10, pts
    # MKV uses a millisecond time base. Selected source times remain unchanged.
    assert pts == list(range(0, 2_000_000_000, 200_000_000))
    if mode == 'jpeg':
        assert result.stdout.count(b'\xff\xd8') == 10
    else:
        # Passthrough must prevent the muxer from duplicating sampled frames.
        counts = []
        for segment in tmp_path.glob('*.ts'):
            probe = subprocess.run([
                'ffprobe', '-v', 'error', '-f', 'mpegts', '-select_streams', 'v:0', '-count_frames',
                '-show_entries', 'stream=nb_read_frames', '-of', 'json', str(segment)
            ], check=True, capture_output=True, timeout=10)
            counts.append(int(json.loads(probe.stdout)['streams'][0]['nb_read_frames']))
        assert sum(counts) == 10
