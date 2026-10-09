#!/usr/bin/env python3
"""Run task compatibility tests under ROS Foxy / Python 3.8 without motion."""
import argparse
from pathlib import Path
import os
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--system-opencv', action='store_true',
                        help='Use Ubuntu system OpenCV/NumPy instead of the project venv')
    args = parser.parse_args()
    if sys.version_info[:2] != (3, 8) or os.environ.get('ROS_DISTRO') != 'foxy':
        parser.error('Run inside ROS Foxy with Python 3.8 after sourcing its setup.bash')
    sys.dont_write_bytecode = True
    repo = Path(__file__).resolve().parents[1]
    workspace = repo / 'workspace_auv'
    for package in sorted((workspace / 'src').iterdir()):
        if package.is_dir():
            sys.path.insert(0, str(package))
    if args.system_opencv:
        system_packages = Path('/usr/lib/python3/dist-packages')
        if not list(system_packages.glob('cv2*')):
            parser.error('System OpenCV is unavailable; install the declared python3-opencv dependency')
        sys.path.insert(0, str(system_packages))

    import cv2
    import numpy as np
    import pytest
    import rclpy

    package = workspace / 'src/uv_task'
    sources = [path for path in package.rglob('*.py') if '__pycache__' not in path.parts]
    for path in sources:
        compile(path.read_text(encoding='utf-8'), str(path), 'exec')
    print('Compatibility baseline: ROS Foxy / Python ' + sys.version.split()[0], flush=True)
    print('OpenCV ' + cv2.__version__ + ' / NumPy ' + np.__version__, flush=True)
    print('Compiled {} Python files; testing source package {}'.format(len(sources), package),
          flush=True)
    return pytest.main(['-q', '-p', 'no:cacheprovider', str(package / 'test')])


if __name__ == '__main__':
    raise SystemExit(main())
