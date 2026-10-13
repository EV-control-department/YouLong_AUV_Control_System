#!/usr/bin/env python3
"""Start the local record review station directly from a repository checkout."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'uv_record'))
from uv_record.review import main

if __name__ == '__main__':
    raise SystemExit(main())
