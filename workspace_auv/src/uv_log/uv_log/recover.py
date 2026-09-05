"""Recover metadata for sessions interrupted without a clean shutdown."""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

from .jpeg_archive import recover_directory
from .recorder import SegmentSyncer
from .session import default_output_root, finish_session, load_manifest


def _pid_alive(pid) -> bool:
    try:
        os.kill(int(pid), 0)
        return True
    except (TypeError, ValueError, ProcessLookupError, PermissionError, OSError):
        return False


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
    syncer = SegmentSyncer(
        directories + [session_dir / 'bag'],
        patterns=('*.ts', '*.mjpg', '*.jsonl', '*.mcap', 'metadata.yaml'),
    )
    syncer.sync_all()
    for directory in directories:
        recover_directory(directory)
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
            print(f'uv_log: recovered {session_dir}', flush=True)
    print(f'uv_log: {recovered} session(s) recovered', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
