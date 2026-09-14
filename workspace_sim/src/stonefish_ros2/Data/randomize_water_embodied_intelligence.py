#!/usr/bin/env python3
"""Randomize a Water Embodied Intelligence Stonefish scenario.

The mapping is deliberately constrained to the competition rules: four unique
cells are selected from the 3x3 mapping area, with two round and two square
cones. Each cone receives an independent horizontal position offset of at most
0.05 m in Euclidean distance and an independent yaw. The AprilTag and optical
environment are randomized as well. A seed makes the complete scenario
reproducible.
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


DEFAULT_SCENE = Path(__file__).with_name("water_embodied_intelligence_2026.scn")
TAG_TEXTURES = {
    tag_id: f"aruco_4x4_id{tag_id}.png" for tag_id in range(7)
}

# NED x is north and y is east. The grid is centered at (2,-4), with 2 m sides.
GRID_CELLS = [
    (2.0 + dx, -4.0 + dy)
    for dx in (-2.0 / 3.0, 0.0, 2.0 / 3.0)
    for dy in (-2.0 / 3.0, 0.0, 2.0 / 3.0)
]
CONE_NAMES = (
    "TrafficConeRound00",
    "TrafficConeRound01",
    "TrafficConeSquare00",
    "TrafficConeSquare01",
)
BAND_NAMES = (
    "TrafficConeRoundBand00",
    "TrafficConeRoundBand01",
    "TrafficConeSquareBand00",
    "TrafficConeSquareBand01",
)
MAX_CELL_OFFSET = 0.05

# Keep the randomized environment usable for camera/perception experiments.
# Stonefish expects Jerlov water type in the interval [0, 1].
DEFAULT_SUN_AZIMUTH_RANGE = (0.0, 360.0)
DEFAULT_SUN_ELEVATION_RANGE = (30.0, 75.0)
DEFAULT_SUN_GAIN_RANGE = (0.5, 5.0)
DEFAULT_JERLOV_RANGE = (0.10, 0.25)


def _world_transform(root: ET.Element, name: str) -> ET.Element:
    for element_type in ("static", "dynamic", "animated"):
        element = root.find(f".//{element_type}[@name='{name}']")
        if element is not None:
            transform = element.find("world_transform")
            if transform is None:
                raise ValueError(f"{name} has no world_transform")
            return transform
    raise ValueError(f"scenario object not found: {name}")


def _set_xyz(transform: ET.Element, x: float, y: float, z: float = 2.0) -> None:
    transform.set("xyz", f"{x:.4f} {y:.4f} {z:.4f}")


def _set_yaw(transform: ET.Element, yaw: float) -> None:
    transform.set("rpy", f"3.1416 0.0 {yaw:.5f}")


def _random_cell_offset(rng: random.Random) -> tuple[float, float]:
    """Return a uniform random point inside the 5 cm horizontal offset disk."""
    radius = MAX_CELL_OFFSET * rng.random() ** 0.5
    angle = rng.uniform(-3.1415926, 3.1415926)
    return radius * math.cos(angle), radius * math.sin(angle)


def _set_tag_texture(root: ET.Element, tag_id: int) -> None:
    looks = root.find("looks")
    if looks is None:
        raise ValueError("scenario has no <looks> section")
    tag_look = looks.find("look[@name='aruco_tag']")
    if tag_look is None:
        raise ValueError("scenario has no aruco_tag look")
    tag_look.set("texture", TAG_TEXTURES[tag_id])


def _random_range(rng: random.Random, value_range: tuple[float, float]) -> float:
    return rng.uniform(value_range[0], value_range[1])


def _set_environment(
    root: ET.Element,
    rng: random.Random,
    sun_azimuth_range: tuple[float, float],
    sun_elevation_range: tuple[float, float],
    sun_gain_range: tuple[float, float],
    jerlov_range: tuple[float, float],
) -> dict[str, float]:
    """Randomize the scene's sun and water optical parameters."""
    environment = root.find("environment")
    if environment is None:
        raise ValueError("scenario has no <environment> section")

    atmosphere = environment.find("atmosphere")
    if atmosphere is None:
        raise ValueError("scenario has no <atmosphere> section")
    sun = atmosphere.find("sun")
    if sun is None:
        raise ValueError("scenario has no <sun> element")

    ocean = environment.find("ocean")
    if ocean is None:
        raise ValueError("scenario has no <ocean> section")
    water = ocean.find("water")
    if water is None:
        raise ValueError("scenario has no <water> element")

    values = {
        "sun_azimuth": _random_range(rng, sun_azimuth_range),
        "sun_elevation": _random_range(rng, sun_elevation_range),
        "sun_gain": _random_range(rng, sun_gain_range),
        "jerlov": _random_range(rng, jerlov_range),
    }
    sun.set("azimuth", f"{values['sun_azimuth']:.3f}")
    sun.set("elevation", f"{values['sun_elevation']:.3f}")
    sun.set("gain", f"{values['sun_gain']:.3f}")
    water.set("jerlov", f"{values['jerlov']:.4f}")
    return values


def _validate_range(
    name: str,
    value_range: tuple[float, float],
    minimum: float | None = None,
    maximum: float | None = None,
) -> None:
    low, high = value_range
    if low > high:
        raise ValueError(f"{name} range minimum must not exceed maximum")
    if minimum is not None and low < minimum:
        raise ValueError(f"{name} range must be >= {minimum}")
    if maximum is not None and high > maximum:
        raise ValueError(f"{name} range must be <= {maximum}")


def randomize_scene(
    source: Path,
    destination: Path,
    seed: int | None,
    tag_id: int | None,
    sun_azimuth_range: tuple[float, float] = DEFAULT_SUN_AZIMUTH_RANGE,
    sun_elevation_range: tuple[float, float] = DEFAULT_SUN_ELEVATION_RANGE,
    sun_gain_range: tuple[float, float] = DEFAULT_SUN_GAIN_RANGE,
    jerlov_range: tuple[float, float] = DEFAULT_JERLOV_RANGE,
) -> tuple[int, list[tuple[str, float, float, float]], dict[str, float]]:
    if tag_id is not None and tag_id not in TAG_TEXTURES:
        raise ValueError("tag id must be between 0 and 6")
    _validate_range("sun azimuth", sun_azimuth_range, 0.0, 360.0)
    _validate_range("sun elevation", sun_elevation_range, 0.0, 90.0)
    _validate_range("sun gain", sun_gain_range, 0.0)
    _validate_range("jerlov", jerlov_range, 0.0, 1.0)

    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(source, parser=parser)
    root = tree.getroot()
    rng = random.Random(seed)

    selected_cells = rng.sample(GRID_CELLS, k=len(CONE_NAMES))
    placements: list[tuple[str, float, float, float]] = []
    for cone_name, band_name, (cell_x, cell_y) in zip(CONE_NAMES, BAND_NAMES, selected_cells):
        offset_x, offset_y = _random_cell_offset(rng)
        x = cell_x + offset_x
        y = cell_y + offset_y
        yaw = rng.uniform(-3.1415926, 3.1415926)
        _set_xyz(_world_transform(root, cone_name), x, y)
        _set_xyz(_world_transform(root, band_name), x, y, 1.9825)
        _set_yaw(_world_transform(root, cone_name), yaw)
        _world_transform(root, band_name).set("rpy", f"0.0 0.0 {yaw:.5f}")
        placements.append((cone_name, x, y, yaw))

    selected_tag = tag_id if tag_id is not None else rng.randrange(7)
    _set_tag_texture(root, selected_tag)
    environment = _set_environment(
        root,
        rng,
        sun_azimuth_range,
        sun_elevation_range,
        sun_gain_range,
        jerlov_range,
    )

    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="\t")
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    return selected_tag, placements, environment


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_SCENE, help="source .scn file")
    parser.add_argument("--output", type=Path, help="destination .scn file")
    parser.add_argument("--in-place", action="store_true", help="replace the input file")
    parser.add_argument("--seed", type=int, help="random seed for reproducible scene generation")
    parser.add_argument("--tag-id", type=int, choices=range(7), help="force AprilTag id 0..6")
    parser.add_argument(
        "--sun-azimuth-range", nargs=2, type=float, metavar=("MIN", "MAX"),
        default=DEFAULT_SUN_AZIMUTH_RANGE,
        help="sun azimuth range in degrees (default: 0 360)",
    )
    parser.add_argument(
        "--sun-elevation-range", nargs=2, type=float, metavar=("MIN", "MAX"),
        default=DEFAULT_SUN_ELEVATION_RANGE,
        help="sun elevation range in degrees (default: 30 75)",
    )
    parser.add_argument(
        "--sun-gain-range", nargs=2, type=float, metavar=("MIN", "MAX"),
        default=DEFAULT_SUN_GAIN_RANGE,
        help="sun light gain range (default: 0.5 5.0)",
    )
    parser.add_argument(
        "--jerlov-range", nargs=2, type=float, metavar=("MIN", "MAX"),
        default=DEFAULT_JERLOV_RANGE,
        help="Jerlov water type range in [0, 1] (default: 0.10 0.35)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.in_place and args.output is not None:
        print("--in-place and --output cannot be used together", file=sys.stderr)
        return 2
    if not args.input.is_file():
        print(f"input scene does not exist: {args.input}", file=sys.stderr)
        return 2

    if args.in_place:
        destination = args.input
    elif args.output is not None:
        destination = args.output
    else:
        suffix = f"_seed{args.seed}" if args.seed is not None else "_randomized"
        destination = args.input.with_name(f"{args.input.stem}{suffix}{args.input.suffix}")

    try:
        selected_tag, placements, environment = randomize_scene(
            args.input,
            destination,
            args.seed,
            args.tag_id,
            tuple(args.sun_azimuth_range),
            tuple(args.sun_elevation_range),
            tuple(args.sun_gain_range),
            tuple(args.jerlov_range),
        )
    except (OSError, ET.ParseError, ValueError) as exc:
        print(f"failed to randomize scene: {exc}", file=sys.stderr)
        return 1

    print(f"wrote {destination}")
    print(f"AprilTag id: {selected_tag}")
    print(
        "Environment: "
        f"sun azimuth={environment['sun_azimuth']:.3f}°, "
        f"elevation={environment['sun_elevation']:.3f}°, "
        f"gain={environment['sun_gain']:.3f}, "
        f"jerlov={environment['jerlov']:.4f}"
    )
    for name, x, y, yaw in placements:
        print(f"{name}: N={x:.4f}, E={y:.4f}, yaw={yaw:.4f} rad")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
