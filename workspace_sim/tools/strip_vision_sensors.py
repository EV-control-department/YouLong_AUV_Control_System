#!/usr/bin/env python3
"""Write a vision-sensor-free copy of a Stonefish scene.

``stonefish_simulator_nogpu`` (the console simulator) refuses any scenario that
declares camera sensors ("Cameras not supported in console mode!"), which makes
the physics-only simulator unusable for scenes built for the GPU one -- even
though DVL/IMU/odometry/pressure do not need rendering at all.

This helper strips the ``type="camera"`` sensor blocks (line based, so every
other byte of the scene is preserved) and can rewrite an ``<include file=...>``
target so a world scene can point at the stripped vehicle scene::

    python3 tools/strip_vision_sensors.py \\
        src/stonefish_ros2/Data/xunyun_fixed.scn /tmp/xunyun_headless.scn

    python3 tools/strip_vision_sensors.py \\
        src/stonefish_ros2/Data/canyon_vision_dvl.scn /tmp/canyon_headless.scn \\
        --retarget xunyun_fixed.scn=xunyun_headless.scn

Note that the output must stay in the same directory as the original scene
when mesh filenames are relative, which is the case for the XUNYUN vehicle.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

SENSOR_OPEN = re.compile(r"<sensor\b")
SENSOR_CLOSE = re.compile(r"</sensor>")
CAMERA_TYPE = re.compile(r'type\s*=\s*"camera"')
SENSOR_NAME = re.compile(r'name\s*=\s*"([^"]+)"')


def strip_camera_sensors(
    text: str, keep: str = ""
) -> tuple[str, int, int]:
    """Remove ``<sensor ... type="camera" ...>`` blocks verbatim.

    ``keep`` is a substring of the camera sensor name to *retain* (e.g.
    ``"down_cam"``): the console simulator rejects any camera, but the GPU one is
    limited by how many cameras it must render per frame, so being able to drop
    one pair is the way to tell "rendering is the bottleneck" from "something
    else is" (2026-09-16).  Returns ``(text, removed, kept)``.
    """
    out: list[str] = []
    skipping = False
    removed = kept = 0
    for line in text.splitlines(keepends=True):
        if skipping:
            if SENSOR_CLOSE.search(line):
                skipping = False
            continue
        if SENSOR_OPEN.search(line) and CAMERA_TYPE.search(line):
            name = SENSOR_NAME.search(line)
            if keep and name and keep in name.group(1):
                kept += 1
                out.append(line)
                if not SENSOR_CLOSE.search(line):
                    skipping = True
                continue
            removed += 1
            if not SENSOR_CLOSE.search(line):
                skipping = True
            continue
        out.append(line)
    return "".join(out), removed, kept


def retarget_includes(text: str, replacements: dict[str, str]) -> str:
    for old, new in replacements.items():
        text = text.replace(f'file="{old}"', f'file="{new}"')
    return text


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument(
        "--retarget",
        action="append",
        default=[],
        metavar="OLD=NEW",
        help="rewrite <include file=\"OLD\"> to NEW (repeatable)",
    )
    parser.add_argument(
        "--keep-camera",
        default="",
        metavar="SUBSTRING",
        help=(
            "keep camera sensors whose name contains SUBSTRING (e.g. down_cam) "
            "and drop the others; default drops all of them"
        ),
    )
    args = parser.parse_args(argv)

    replacements: dict[str, str] = {}
    for item in args.retarget:
        if "=" not in item:
            parser.error(f"--retarget expects OLD=NEW, got {item!r}")
        old, new = item.split("=", 1)
        replacements[old] = new

    text = args.source.read_text(encoding="utf-8")
    stripped, removed, kept = strip_camera_sensors(text, args.keep_camera)
    stripped = retarget_includes(stripped, replacements)
    args.destination.write_text(stripped, encoding="utf-8")

    print(
        f"{args.source} -> {args.destination}: removed {removed} camera sensor(s)"
        + (f", kept {kept}" if kept else "")
        + (f", retargeted {len(replacements)} include(s)" if replacements else "")
    )
    if removed == 0 and not replacements and not kept:
        print("warning: nothing was stripped or retargeted", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
