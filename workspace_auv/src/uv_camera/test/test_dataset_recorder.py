import json
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from uv_camera.dataset_recorder import DatasetRecorder


def _header(sec, nanosec, frame_id):
    return SimpleNamespace(
        stamp=SimpleNamespace(sec=sec, nanosec=nanosec), frame_id=frame_id)


def test_png_frames_are_pixel_exact_and_indexed(tmp_path):
    frame = np.zeros((12, 20, 3), dtype=np.uint8)
    frame[:, :, 0] = np.arange(20, dtype=np.uint8)
    frame[:, :, 1] = 73
    frame[3:9, 4:15, 2] = 251

    recorder = DatasetRecorder(tmp_path, queue_size=1, png_compression=9)
    session_dir = recorder.session_dir
    assert recorder.submit(
        frame, 'front_left', _header(12, 34, 'front_left'), 99)
    recorder.close()

    rows = [json.loads(line) for line in
            (session_dir / 'frames.jsonl').read_text().splitlines()]
    assert len(rows) == 1
    assert rows[0]['channel'] == 'front_left'
    assert rows[0]['stamp_ns'] == 12_000_000_034
    assert rows[0]['stereo_pair_id'] == 99
    assert rows[0]['path'] == 'images/front_left/000000000000.png'

    restored = cv2.imread(str(session_dir / rows[0]['path']), cv2.IMREAD_COLOR)
    assert restored is not None
    np.testing.assert_array_equal(restored, frame)

    status = json.loads((session_dir / 'status.json').read_text())
    assert status['state'] == 'stopped'
    assert status['frames_submitted'] == 1
    assert status['frames_written'] == 1


def test_restart_creates_a_new_session_without_overwriting_old_data(tmp_path):
    first = DatasetRecorder(tmp_path)
    first_session = first.session_dir
    first.submit(np.zeros((2, 2, 3), dtype=np.uint8), 'down_right')
    first.close()

    second = DatasetRecorder(tmp_path)
    second_session = second.session_dir
    second.close()

    assert second_session != first_session
    assert (first_session / 'frames.jsonl').read_text().count('\n') == 1


def test_webp_lossless_is_pixel_exact(tmp_path):
    pytest.importorskip('PIL')
    frame = np.random.default_rng(42).integers(
        0, 256, (32, 48, 3), dtype=np.uint8)

    recorder = DatasetRecorder(tmp_path, image_format='webp_lossless')
    session_dir = recorder.session_dir
    recorder.submit(frame, 'front_right')
    recorder.close()

    row = json.loads((session_dir / 'frames.jsonl').read_text())
    assert row['image_format'] == 'webp_lossless'
    assert row['path'].endswith('.webp')

    from PIL import Image
    restored_rgb = np.asarray(Image.open(session_dir / row['path']).convert('RGB'))
    restored_bgr = restored_rgb[:, :, ::-1]
    np.testing.assert_array_equal(restored_bgr, frame)
