"""Session directory creation and crash-safe metadata helpers."""

from __future__ import annotations

import json
import os
import socket
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path


@dataclass(frozen=True)
class SessionPaths:
    """Canonical paths belonging to one recording session."""

    root: Path
    bag: Path
    video: Path
    logs: Path
    metadata: Path


def project_root() -> Path:
    """Return the repository root containing the ROS workspaces.

    The recorder and player are often started from ``~`` (or by a launch
    service), so using ``Path.cwd()`` as the default would put sessions in an
    unexpected directory.  The package is installed inside this repository,
    including with a regular non-symlink install, which lets us locate the
    root from this module path.
    """
    module_path = Path(__file__).resolve()
    for parent in (module_path.parent, *module_path.parents):
        if ((parent / 'workspace_auv').is_dir()
                and (parent / 'workspace_sim').is_dir()):
            return parent

    # This is only a fallback for a package installed outside this checkout.
    # It keeps standalone use functional while the workspace install always
    # takes the repository-root branch above.
    return Path.cwd().resolve()


def default_output_root() -> Path:
    """Return the default session directory for this project."""
    return project_root() / 'sessions'


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


def _git_commit() -> str | None:
    try:
        result = subprocess.run(
            ['git', 'rev-parse', '--short', 'HEAD'],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        commit = result.stdout.strip()
        return commit or None
    except (OSError, subprocess.SubprocessError):
        return None


def _fsync_directory(path: Path) -> None:
    try:
        fd = os.open(path, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except OSError:
        # Some filesystems do not allow fsync on directories. File data is
        # still synced by write_json_atomic and the segment syncer.
        pass


def write_json_atomic(path: Path, value: dict) -> None:
    """Atomically replace a JSON file and synchronise it to disk."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
    payload = json.dumps(value, ensure_ascii=False, indent=2) + '\n'
    fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def _session_paths(root: Path) -> SessionPaths:
    return SessionPaths(
        root=root,
        bag=root / 'bag',
        video=root / 'video',
        logs=root / 'logs',
        metadata=root / 'metadata',
    )


def initialise_session(session_dir: str | Path) -> SessionPaths:
    """Create the directory tree and initial metadata for a session."""
    root = Path(session_dir).expanduser().resolve()
    paths = _session_paths(root)
    for directory in (
        paths.root, paths.bag, paths.video, paths.logs, paths.metadata,
        paths.logs / 'nodes', paths.logs / 'ros',
    ):
        directory.mkdir(parents=True, exist_ok=True)

    manifest_path = paths.root / 'manifest.json'
    if not manifest_path.exists():
        manifest = {
            'format': 'uv_log_session_v1',
            'status': 'RUNNING',
            'unclean': False,
            'started_at_utc': _utc_now(),
            'started_unix_ns': time.time_ns(),
            'host': socket.gethostname(),
            'pid': os.getpid(),
            'git_commit': _git_commit(),
            'bag': {},
            'video': {},
            'logs': {},
        }
        write_json_atomic(manifest_path, manifest)
    else:
        # A caller may intentionally reuse a pre-created directory (the
        # launch file does this). Make the active state explicit again.
        try:
            manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError):
            manifest = {'format': 'uv_log_session_v1'}
        manifest.update({
            'status': 'RUNNING',
            'unclean': False,
            'pid': os.getpid(),
        })
        write_json_atomic(manifest_path, manifest)

    lock = {
        'pid': os.getpid(),
        'host': socket.gethostname(),
        'started_at_utc': _utc_now(),
        'started_unix_ns': time.time_ns(),
    }
    write_json_atomic(paths.root / 'session.lock', lock)
    write_json_atomic(paths.root / 'heartbeat.json', {
        'status': 'RUNNING',
        'pid': os.getpid(),
        'updated_at_utc': _utc_now(),
        'updated_unix_ns': time.time_ns(),
    })
    return paths


def create_session(output_root: str | Path, session_name: str | None = None) -> SessionPaths:
    """Create a unique timestamped session below ``output_root``."""
    root = Path(output_root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    base_name = session_name or datetime.now().strftime('%Y%m%d_%H%M%S')
    candidate = root / base_name
    suffix = 1
    while candidate.exists():
        candidate = root / f'{base_name}_{suffix:02d}'
        suffix += 1
    return initialise_session(candidate)


def load_manifest(session_dir: str | Path) -> dict:
    path = Path(session_dir).expanduser().resolve() / 'manifest.json'
    with path.open(encoding='utf-8') as handle:
        return json.load(handle)


def update_manifest(session_dir: str | Path, **updates) -> dict:
    """Merge updates into manifest.json using an atomic replacement."""
    root = Path(session_dir).expanduser().resolve()
    try:
        manifest = load_manifest(root)
    except (FileNotFoundError, json.JSONDecodeError):
        manifest = {'format': 'uv_log_session_v1'}
    manifest.update(updates)
    write_json_atomic(root / 'manifest.json', manifest)
    return manifest


def append_event(session_dir: str | Path, event: dict) -> None:
    """Append one small event record and force it to stable storage."""
    path = Path(session_dir).expanduser().resolve() / 'events.jsonl'
    path.parent.mkdir(parents=True, exist_ok=True)
    record = dict(event)
    record.setdefault('time_unix_ns', time.time_ns())
    with path.open('a', encoding='utf-8') as handle:
        handle.write(json.dumps(record, ensure_ascii=False) + '\n')
        handle.flush()
        os.fsync(handle.fileno())


def update_heartbeat(session_dir: str | Path, **values) -> None:
    payload = {
        'status': 'RUNNING',
        'pid': os.getpid(),
        'updated_at_utc': _utc_now(),
        'updated_unix_ns': time.time_ns(),
        **values,
    }
    write_json_atomic(
        Path(session_dir).expanduser().resolve() / 'heartbeat.json', payload)


def finish_session(session_dir: str | Path, status: str = 'STOPPED', **values) -> None:
    """Mark a session complete without deleting its data files."""
    root = Path(session_dir).expanduser().resolve()
    update_manifest(
        root,
        status=status,
        unclean=status not in ('STOPPED', 'COMPLETED'),
        finished_at_utc=_utc_now(),
        finished_unix_ns=time.time_ns(),
        **values,
    )
    write_json_atomic(root / 'heartbeat.json', {
        'status': status,
        'pid': os.getpid(),
        'updated_at_utc': _utc_now(),
        'updated_unix_ns': time.time_ns(),
    })
    try:
        (root / 'session.lock').unlink()
        _fsync_directory(root)
    except OSError:
        pass
