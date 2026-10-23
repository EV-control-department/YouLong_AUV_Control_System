"""Recover SL sessions with the existing generic archive recovery utility."""

import argparse
from pathlib import Path

from uv_record.recover import recover_one as recover_session

from .catalog_archive import restore_catalog_archive
from .recorder import default_output_root


def recover_one(path):
    if not recover_session(path):
        return False
    restore_catalog_archive(path)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-dir')
    parser.add_argument('--root', default=str(default_output_root()))
    args = parser.parse_args()
    root = Path(args.root).expanduser()
    sessions = ([Path(args.session_dir).expanduser()] if args.session_dir
                else [path for path in root.iterdir() if path.is_dir()] if root.is_dir() else [])
    for path in sessions:
        if recover_one(path):
            print(f'sl_record: recovered {path}')
    return 0
