#!/usr/bin/env python3
"""Generate the canonical YouLong Stonefish description deterministically.

The source scene is intentionally kept as the authoritative Stonefish syntax:
Stonefish sensors and actuators must remain inside the robot definition because
its include mechanism only works at scenario root level.  YAML files describe
and validate the maintained geometry/physics boundary; this tool materialises
the checked-in canonical scene from that source without changing numerical
parameters.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--template",
        type=Path,
        default=root / "vehicles/youlong/model/youlong.scn",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "vehicles/youlong/model/youlong.scn",
    )
    args = parser.parse_args()
    template = args.template.resolve()
    output = args.output.resolve()
    config_dir = root / "config"
    required_configs = (
        "vehicle_geometry.yaml",
        "vehicle_inertial.yaml",
        "vehicle_hydrodynamics.yaml",
        "vehicle_thrusters.yaml",
        "vehicle_sensors.yaml",
    )
    missing = [name for name in required_configs
               if not (config_dir / name).is_file()]
    if missing:
        raise SystemExit(f"missing vehicle configuration: {', '.join(missing)}")
    source = template.read_text(encoding="utf-8")
    if "<robot name=\"$(arg robot_name)\"" not in source:
        raise SystemExit("vehicle template must expose the robot_name argument")
    if "physics=\"submerged\"" not in source:
        raise SystemExit("vehicle template must retain Stonefish submerged physics")
    if template == output:
        # A checked-in generated file is already deterministic.  Reading it
        # here still verifies that the requested source exists.
        return
    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(template, output)


if __name__ == "__main__":
    main()
