#!/usr/bin/env python3
"""Check that committed vehicle and seeded-world outputs are reproducible."""

from __future__ import annotations

import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


SEED_OUTPUT = "worlds/guoshui_2026/guoshui_2026_cruise_seeded.scn"
GENERATED_OBJECTS = (
    "objects/guoshui_2026/guoshui_2026_seeded_gate_red_pipes.obj",
    "objects/guoshui_2026/guoshui_2026_seeded_gate_white_supports.obj",
    "objects/guoshui_2026/guoshui_2026_seeded_gate_red_sleeves.obj",
    "objects/guoshui_2026/guoshui_2026_seeded_gate_white_sleeves.obj",
)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(command: list[str]) -> None:
    subprocess.run(command, check=True, stdout=subprocess.DEVNULL)


def generate_tree(source: Path, destination: Path, seed: int) -> dict[str, str]:
    shutil.copytree(source, destination)
    tools = destination / "tools"
    worlds = destination / "worlds" / "guoshui_2026"
    run([
        sys.executable,
        str(tools / "generate_guoshui_2026_scene.py"),
        "--seed",
        str(seed),
        "--template",
        str(worlds / "guoshui_2026_cruise.scn"),
        "--output",
        str(worlds / "guoshui_2026_cruise_seeded.scn"),
    ])
    paths = [SEED_OUTPUT, *GENERATED_OBJECTS]
    return {relative: digest(destination / relative) for relative in paths}


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    vehicle_generator = root / "tools" / "generate_vehicle.py"
    vehicle_template = root / "vehicles" / "youlong" / "model" / "youlong.scn"

    with tempfile.TemporaryDirectory(prefix="uv_sim_assets_repro_") as temporary:
        temporary_root = Path(temporary)
        vehicle_output = temporary_root / "youlong.scn"
        run([
            sys.executable,
            str(vehicle_generator),
            "--template",
            str(vehicle_template),
            "--output",
            str(vehicle_output),
        ])
        if vehicle_output.read_bytes() != vehicle_template.read_bytes():
            raise SystemExit("vehicle generator output differs from template")

        first = generate_tree(root, temporary_root / "first", seed=0)
        committed = {
            relative: digest(root / relative)
            for relative in [SEED_OUTPUT, *GENERATED_OBJECTS]
        }
        if first != committed:
            raise SystemExit("seed 0 output differs from committed assets")

        second = generate_tree(root, temporary_root / "second", seed=17)
        third = generate_tree(root, temporary_root / "third", seed=17)
        if second != third:
            raise SystemExit("same scene seed produced different assets")

    print("reproducibility check passed")


if __name__ == "__main__":
    main()
