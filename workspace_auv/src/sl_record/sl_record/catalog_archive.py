"""Restore the durable catalog prefix before recovery or session reuse."""

import json
import os
from pathlib import Path

from uv_record.session import append_event, write_json_atomic


def restore_catalog_archive(session_dir):
    root = Path(session_dir)
    history = root / 'metadata/camera_catalogs.jsonl'
    if not history.is_file():
        return 0
    raw = history.read_bytes()
    valid, latest, sequence = [], None, 0
    for line in raw.splitlines(keepends=True):
        try:
            item = json.loads(line)
            number = item['catalog_sequence']
            catalog = item['catalog']
            if (not isinstance(number, int) or number <= sequence
                    or not catalog['source_instance_id'] or not catalog['config_version']
                    or not isinstance(catalog['cameras'], list)):
                break
        except (ValueError, TypeError, KeyError):
            break
        valid.append(line.rstrip(b'\r\n') + b'\n')
        latest, sequence = item, number
    recovered = b''.join(valid)
    if recovered != raw:
        temporary = history.with_suffix('.jsonl.tmp')
        with temporary.open('wb') as stream:
            stream.write(recovered)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(history)
        append_event(root, {'event': 'camera_catalog_history_recovered',
                            'last_catalog_sequence': sequence})
    snapshot = root / 'metadata/camera_catalog.json'
    if latest is not None:
        write_json_atomic(snapshot, latest)
    elif snapshot.exists():
        snapshot.unlink()
    return sequence
