#!/usr/bin/env python3
"""T2: derive every sensor extrinsic from the scene file, and verify the docs.

The scene file is the machine-readable source of truth for the **simulated**
vehicle; ``docs/frames/extrinsics_reference.md`` embeds the resulting matrices
inside a fenced YAML block.  This tool derives the transforms from the scene
and can diff them against that block, so the documentation cannot silently
drift from the simulation.

    # print the derivation
    python3 workspace_sim/tools/derive_extrinsics.py

    # regenerate the fenced block for the doc (stdout)
    python3 workspace_sim/tools/derive_extrinsics.py --emit

    # fail (exit 1) when the doc disagrees with the scene
    python3 workspace_sim/tools/derive_extrinsics.py --check

Transforms use the AQUA-SLAM vocabulary (``docs/frames/extrinsics_reference.md``):
``T_<a>_<b>`` maps a point from frame ``b`` to frame ``a``.  ``c`` is the
**left camera optical** frame; it is the reference because AQUA-SLAM's ``Tbc``,
``T_gyro_c`` and ``T_dvl_c`` are all defined against it.

Exit codes: 0 pass, 1 mismatch, 2 environment/scene problem.
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "workspace_sim" / "src" / "uv_sim"))

from uv_sim.geometry import (  # noqa: E402
    DEFAULT_SCENE,
    SCENE_SENSOR_NAMES,
    dvl_beam_azimuths_from_scene,
    invert_rigid,
    parse_scn_sensors,
)

DOC_PATH = REPO_ROOT / "docs" / "frames" / "extrinsics_reference.md"

#: Fence marker in the doc.  The YAML block directly after it is the table
#: this tool checks against; regenerate it with ``--emit``.
FENCE_MARKER = "<!-- extrinsics-table: begin -->"
FENCE_END = "<!-- extrinsics-table: end -->"

#: Sensor frame that AQUA-SLAM calls ``c`` (left camera optical frame).
CAMERA_FRAME = "front_cam_left"

#: Sensor frame AQUA-SLAM calls ``gyro``; the scene IMU sits at the body
#: origin, so this is only nominally different from ``base_link``.
GYRO_FRAME = "imu_filter"

#: Sensor frame AQUA-SLAM calls ``dvl``.
DVL_FRAME = "dvl"

#: Human-facing notes kept in the emitted YAML (not derived from the scene).
SENSOR_NOTES = {
    "dynamics": "odometry ground truth, publishes TF world_ned -> Vehicle/base_link",
    "pressure": "depth sensor",
    "dvl": "4-beam, theta=30 deg off vertical",
    "imu_filter": "200 Hz with injected noise",
    "front_cam_left": "AQUA-SLAM reference frame c",
    "front_cam_right": "stereo right eye",
    "down_cam_left": "down-looking pair, left eye",
    "down_cam_right": "down-looking pair, right eye",
}


@dataclass
class Extrinsics:
    """Derived transforms for one scene file."""

    scene: str
    platform: str
    sensors: dict
    transforms: dict

    def as_yaml_dict(self) -> dict:
        return {
            "platform": self.platform,
            "source_scene": self.scene,
            "convention": (
                "T_<a>_<b> maps a point from frame b into frame a; "
                "frames: b=Vehicle/base_link (body, x fwd/y stbd/z down), "
                "c=front_cam_left optical, gyro=imu_filter, dvl=dvl sensor"
            ),
            "truth": "derived by tools/derive_extrinsics.py; do not edit by hand",
            "sensors": self.sensors,
            "transforms": self.transforms,
        }


def _matrix(yaml_matrix: np.ndarray) -> list[list[float]]:
    """Round to a stable, readable precision (1e-9 m / 1e-12 rad)."""
    return [[round(float(value), 9) for value in row] for row in yaml_matrix]


def derive(scn_path: Path, platform: str) -> Extrinsics:
    sensors = parse_scn_sensors(scn_path)
    missing = [name for name in SCENE_SENSOR_NAMES if name not in sensors]
    if missing:
        raise KeyError(f"{scn_path} is missing sensors: {missing}")

    sensor_table: dict[str, dict] = {}
    transforms: dict[str, dict] = {}
    body_to_sensor: dict[str, np.ndarray] = {}

    for name in SCENE_SENSOR_NAMES:
        sensor = sensors[name]
        body_to_sensor[name] = sensor.transform_body_to_sensor
        entry = {
            "type": sensor.kind,
            "link": sensor.link,
            "xyz": [round(value, 9) for value in sensor.xyz],
            "rpy_rad": [round(value, 12) for value in sensor.rpy],
            "rate_hz": sensor.rate_hz,
        }
        if sensor.ros_topics:
            entry["ros_publisher"] = dict(sensor.ros_topics)
        if sensor.specs:
            entry["specs"] = dict(sensor.specs)
        if name == DVL_FRAME:
            entry["beam_angle_deg"] = round(float(np.degrees(sensor.beam_angle_rad)), 9)
            entry["alpha_deg"] = round(float(np.degrees(sensor.alpha_rad)), 9)
            entry["beta_deg"] = list(dvl_beam_azimuths_from_scene(sensor))
        sensor_table[name] = entry
        transforms[f"T_b_{name}"] = {"matrix": _matrix(body_to_sensor[name])}

    camera = body_to_sensor[CAMERA_FRAME]
    camera_inverse = invert_rigid(camera)
    # T_<a>_c for the two sensors AQUA-SLAM cares about.
    for label, frame in (("gyro", GYRO_FRAME), ("dvl", DVL_FRAME)):
        transforms[f"T_{label}_c"] = {
            "matrix": _matrix(invert_rigid(body_to_sensor[frame]) @ camera)
        }
    transforms["T_b_c"] = {"matrix": _matrix(camera)}
    transforms["T_c_b"] = {"matrix": _matrix(camera_inverse)}
    # Sanity: every matrix in the table must be a proper rigid transform.
    for name, entry in transforms.items():
        matrix = np.array(entry["matrix"], dtype=float)
        rotation = matrix[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9):
            raise ValueError(f"{name} rotation is not orthonormal")
        if abs(np.linalg.det(rotation) - 1.0) > 1e-9:
            raise ValueError(f"{name} rotation determinant is not +1")
    return Extrinsics(
        scene=str(scn_path.relative_to(REPO_ROOT))
        if scn_path.is_relative_to(REPO_ROOT)
        else str(scn_path),
        platform=platform,
        sensors=sensor_table,
        transforms=transforms,
    )


def _dump_yaml(payload: dict) -> str:
    """Dump the table with vectors and matrices inline, one row per line.

    The default emitter turns every 3-vector into three lines, which buries
    the matrices that matter in the doc.  Inner lists of at most four scalars
    are emitted in flow style; the outer list of a matrix stays expanded so
    each row is one line.
    """
    import yaml

    class _Dumper(yaml.SafeDumper):
        pass

    def _represent_list(dumper, data):
        flow = bool(data) and len(data) <= 4 and not any(
            isinstance(item, (list, dict)) for item in data
        )
        return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=flow)

    _Dumper.add_representer(list, _represent_list)
    _Dumper.add_representer(
        dict,
        lambda dumper, data: dumper.represent_mapping(
            "tag:yaml.org,2002:map", data, flow_style=False
        ),
    )
    return yaml.dump(
        payload, Dumper=_Dumper, sort_keys=False, allow_unicode=True, width=200
    )


def _extract_yaml_block(text: str) -> str | None:
    start = text.find(FENCE_MARKER)
    if start < 0:
        return None
    start = text.find("```yaml", start)
    if start < 0:
        return None
    start = text.find("\n", start) + 1
    end = text.find("```", start)
    if end < 0:
        return None
    return text[start:end]


def _compare(expected: dict, actual: dict, path: str = "") -> list[str]:
    """Recursively diff two nested dict/list/scalar structures."""
    problems: list[str] = []
    if isinstance(expected, dict):
        if not isinstance(actual, dict):
            return [f"{path or '<root>'}: expected a mapping, found {type(actual).__name__}"]
        for key in expected:
            if key not in actual:
                problems.append(f"{path}.{key}: missing in {actual.get('__file__', 'derived')}")
            else:
                problems.extend(_compare(expected[key], actual[key], f"{path}.{key}"))
        for key in actual:
            if key not in expected:
                problems.append(f"{path}.{key}: unexpected extra key")
        return problems
    if isinstance(expected, list):
        if not isinstance(actual, list) or len(expected) != len(actual):
            return [f"{path}: length mismatch ({len(expected)} vs {len(actual)})"]
        for index, (left, right) in enumerate(zip(expected, actual)):
            problems.extend(_compare(left, right, f"{path}[{index}]"))
        return problems
    if isinstance(expected, float) or isinstance(actual, float):
        try:
            if abs(float(expected) - float(actual)) > 1e-9:
                problems.append(f"{path}: {actual} != {expected}")
        except (TypeError, ValueError):
            problems.append(f"{path}: {actual!r} != {expected!r}")
        return problems
    if expected != actual:
        problems.append(f"{path}: {actual!r} != {expected!r}")
    return problems


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scn", default=str(REPO_ROOT / DEFAULT_SCENE))
    parser.add_argument("--platform", default="xunyun")
    parser.add_argument("--out", default="", help="write the YAML table to this path")
    parser.add_argument(
        "--emit", action="store_true", help="print the YAML table for the doc"
    )
    parser.add_argument(
        "--check", action="store_true", help="diff the derived table against the doc"
    )
    parser.add_argument("--doc", default=str(DOC_PATH))
    args = parser.parse_args(argv)

    scn_path = Path(args.scn)
    if not scn_path.is_file():
        print(f"FAIL  scene not found: {scn_path}")
        return 2
    try:
        derived = derive(scn_path, args.platform)
    except (KeyError, ValueError) as error:
        print(f"FAIL  cannot derive extrinsics: {error}")
        return 2

    payload = derived.as_yaml_dict()
    text = _dump_yaml(payload)

    if args.out:
        out_path = Path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(text, encoding="utf-8")
        print(f"wrote {out_path}")
    if args.emit:
        print(text, end="")

    if not (args.emit or args.out or args.check):
        print(f"scene    : {derived.scene}")
        print(f"platform : {derived.platform}")
        print("\nSensor poses (body frame, <origin> verbatim):")
        print(f"  {'sensor':<16} {'type':<9} {'xyz [m]':<26} {'rpy [rad]':<30} rate")
        for name, entry in derived.sensors.items():
            xyz = "(" + ", ".join(f"{v:+.3f}" for v in entry["xyz"]) + ")"
            rpy = "(" + ", ".join(f"{v:+.6f}" for v in entry["rpy_rad"]) + ")"
            rate = "" if entry["rate_hz"] is None else f"{entry['rate_hz']:g} Hz"
            print(f"  {name:<16} {entry['type']:<9} {xyz:<26} {rpy:<30} {rate}")
        dvl = derived.sensors[DVL_FRAME]
        print(
            f"\nDVL beams : theta={dvl['beam_angle_deg']} deg off vertical"
            f" -> AQUA-SLAM alpha={dvl['alpha_deg']} deg,"
            f" beta={dvl['beta_deg']} deg"
        )
        print("\nTransforms (AQUA-SLAM vocabulary, T_<a>_<b> maps b -> a):")
        for name in ("T_gyro_c", "T_dvl_c", "T_b_c"):
            print(f"\n{name} =")
            for row in derived.transforms[name]["matrix"]:
                print("  [" + ", ".join(f"{value:+.9f}" for value in row) + "]")

        # Round-trip evidence: T_c_b must undo T_b_c.
        b_c = np.array(derived.transforms["T_b_c"]["matrix"], dtype=float)
        c_b = np.array(derived.transforms["T_c_b"]["matrix"], dtype=float)
        residual = float(np.max(np.abs(b_c @ c_b - np.eye(4))))
        print(f"\nT_b_c @ T_c_b - I : max residual {residual:.3e}")

    if args.check:
        doc_path = Path(args.doc)
        if not doc_path.is_file():
            print(f"FAIL  doc not found: {doc_path}")
            return 2
        block = _extract_yaml_block(doc_path.read_text(encoding="utf-8"))
        if block is None:
            print(f"FAIL  no fenced YAML block between the markers in {doc_path}")
            return 2
        documented = yaml.safe_load(block)
        # ``source_scene`` legitimately differs when --scn points elsewhere;
        # everything else must match the documented table exactly.
        problems = [
            problem
            for problem in _compare(documented, payload)
            if not problem.startswith(".source_scene")
        ]
        if problems:
            print(f"scene     : {derived.scene}")
            print(f"documented: {doc_path}")
            for problem in problems[:40]:
                print(f"  MISMATCH {problem}")
            if len(problems) > 40:
                print(f"  ... and {len(problems) - 40} more")
            print("\nFAIL  docs/frames/extrinsics_reference.md drifted from the scene")
            print("      regenerate the block with: derive_extrinsics.py --emit")
            return 1
        print(f"OK    {doc_path.name} matches the scene ({derived.scene})")
        return 0

    if not (args.emit or args.out or args.check):
        print("\nOK    derivation complete (use --check to diff against the doc)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
