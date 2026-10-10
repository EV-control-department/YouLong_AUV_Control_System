"""Recover metadata for sessions interrupted without a clean shutdown."""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import cv2

from .jpeg_archive import recover_directory
from .recorder import SegmentSyncer
from .session import default_output_root, finish_session, load_manifest


def _pid_alive(pid) -> bool:
    try:
        os.kill(int(pid), 0)
        return True
    except (TypeError, ValueError, ProcessLookupError, PermissionError, OSError):
        return False



def _rewrite_jsonl(path: Path, keep_record) -> bool:
    """Remove a partial/corrupt JSONL tail while retaining valid records."""
    try:
        lines = path.read_text(encoding='utf-8').splitlines()
    except OSError:
        return False
    valid = []
    changed = False
    for line in lines:
        try:
            record = json.loads(line)
        except (TypeError, ValueError, json.JSONDecodeError):
            changed = True
            continue
        if not isinstance(record, dict) or not keep_record(record):
            changed = True
            continue
        valid.append(json.dumps(record, ensure_ascii=False))
    if not changed and len(valid) == len(lines):
        return False
    temporary = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
    with temporary.open('w', encoding='utf-8') as handle:
        handle.write(''.join(line + '\n' for line in valid))
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)
    return True


def recover_raw_camera(directory: Path) -> bool:
    """Keep only raw frame index entries whose JPEG or legacy PNG files are complete."""
    index = directory / 'frames.jsonl'
    if not index.is_file():
        return False

    def valid(record):
        filename = str(record.get('path', ''))
        frame = (directory / filename).resolve()
        if frame.parent != directory.resolve() or not frame.is_file():
            return False
        try:
            if frame.stat().st_size <= 0 or int(record.get('timestamp_ns', -1)) < 0:
                return False
            if frame.suffix.lower() in ('.jpg', '.jpeg'):
                from uv_image_transport.jpeg import jpeg_dimensions
                jpeg_dimensions(frame.read_bytes())
            image = cv2.imread(str(frame), cv2.IMREAD_COLOR)
            if image is None:
                return False
            expected_width = int(record.get('width', image.shape[1]))
            expected_height = int(record.get('height', image.shape[0]))
            return image.shape[1] == expected_width and image.shape[0] == expected_height
        except (OSError, TypeError, ValueError):
            return False

    return _rewrite_jsonl(index, valid)


def recover_frame_alignment(directory: Path) -> bool:
    index = directory / 'frame_alignment.jsonl'
    if not index.is_file():
        return False
    return _rewrite_jsonl(
        index, lambda item: isinstance(item.get('sequence'), int)
        and isinstance(item.get('receive_time_unix_ns'), int))

def recover_one(session_dir: Path) -> bool:
    try:
        manifest = load_manifest(session_dir)
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return False
    if manifest.get('status') not in (None, 'RUNNING'):
        return False
    try:
        lock = json.loads((session_dir / 'session.lock').read_text())
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        lock = {}
    if _pid_alive(lock.get('pid')):
        return False

    video_root = session_dir / 'video'
    directories = [
        path for path in video_root.glob('*') if path.is_dir()
    ] if video_root.is_dir() else []
    raw_root = session_dir / 'camera' / 'raw'
    raw_directories = [
        path for path in raw_root.glob('*') if path.is_dir()
    ] if raw_root.is_dir() else []
    syncer = SegmentSyncer(
        directories + raw_directories + [session_dir / 'bag'],
        patterns=(
            '*.ts', '*.mjpg', '*.jpg', '*.jpeg', '*.png', '*.jsonl', '*.mcap', '*.db3',
            'metadata.yaml'),
    )
    syncer.sync_all()
    for directory in directories:
        recover_directory(directory)
        recover_frame_alignment(directory)
    for directory in raw_directories:
        recover_raw_camera(directory)
    syncer.sync_all()
    finish_session(
        session_dir,
        'RECOVERED',
        recovered_at_unix_ns=time.time_ns(),
        recovery_note='Recovered after recorder process did not close cleanly',
    )
    return True


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', default=str(default_output_root()))
    parser.add_argument('--session-dir')
    return parser.parse_args()


def main():
    args = _parse_args()
    if args.session_dir:
        candidates = [Path(args.session_dir).expanduser().resolve()]
    else:
        root = Path(args.root).expanduser().resolve()
        candidates = [path for path in root.iterdir() if path.is_dir()] \
            if root.is_dir() else []
    recovered = 0
    for session_dir in sorted(candidates):
        if recover_one(session_dir):
            recovered += 1
            print(f'uv_record: recovered {session_dir}', flush=True)
    print(f'uv_record: {recovered} session(s) recovered', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
