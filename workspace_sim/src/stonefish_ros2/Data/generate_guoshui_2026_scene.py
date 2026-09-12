#!/usr/bin/env python3
"""Build a reproducible Guoshui 2026 training scene from one integer seed.

The fixed ``guoshui_2026_cruise.scn`` file remains the editable baseline.  This
tool writes a separate seeded SCN and regenerates the four gate OBJ parts so
Stonefish can load the result without any runtime randomisation.
"""

from __future__ import annotations

import argparse
import math
import random
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Tuple

from generate_guoshui_gate_parts import GATES, write_gate_parts


Vec2 = Tuple[float, float]


def indent_xml(tree: ET.ElementTree, space: str = "  ") -> None:
    """Indent an XML tree on Python 3.8 and newer Python versions."""

    # ElementTree.indent() was added in Python 3.9.
    if hasattr(ET, "indent"):
        ET.indent(tree, space=space)
        return

    def _indent(element: ET.Element, level: int = 0) -> None:
        children = list(element)
        if children:
            newline = "\n" + space * (level + 1)
            if not element.text or not element.text.strip():
                element.text = newline
            for child in children:
                _indent(child, level + 1)
                if not child.tail or not child.tail.strip():
                    child.tail = newline
            if not children[-1].tail or not children[-1].tail.strip():
                children[-1].tail = "\n" + space * level
        elif level and (not element.tail or not element.tail.strip()):
            element.tail = "\n" + space * level

    _indent(tree.getroot())


START_CENTER: Vec2 = (8.95, -1.80)
IMPACT_CENTER: Vec2 = (8.70, 1.55)
RACK_CENTER: Vec2 = (0.95, -0.45)
COLLECTION_CENTER: Vec2 = (1.55, -1.65)
RING_CENTER: Vec2 = (1.00, -0.23)

SEEDED_GATE_MODELS = {
    "Guoshui2026GateRedPipes": "guoshui_2026_seeded_gate_red_pipes.obj",
    "Guoshui2026GateWhiteSupports": "guoshui_2026_seeded_gate_white_supports.obj",
    "Guoshui2026GateRedSleeves": "guoshui_2026_seeded_gate_red_sleeves.obj",
    "Guoshui2026GateWhiteSleeves": "guoshui_2026_seeded_gate_white_sleeves.obj",
}

# Deliberately bounded perturbations.  Gate cross-track motion is the largest
# variation; all other values preserve the rule-sized pool and task order.
GATE_LATERAL_METRES = 0.30
GATE_LONGITUDINAL_METRES = 0.08
GATE_YAW_RADIANS = math.radians(5.0)
RACK_POSITION_METRES = 0.08
RACK_YAW_RADIANS = math.radians(5.0)
COLLECTION_POSITION_METRES = 0.08
COLLECTION_YAW_RADIANS = math.radians(5.0)
TARGET_ITEM_YAW_RADIANS = math.pi
RACK_HALF_EXTENT_METRES = 0.22
TARGET_EDGE_MARGIN_METRES = 0.025
TARGET_GOLF_RADIUS_METRES = 0.02135
TARGET_RING_OUTER_RADIUS_METRES = 0.060
TARGET_MIN_SEPARATION_METRES = 0.115
TARGET_POSITION_RETRIES = 1000
IMPACT_GROUP_POSITION_METRES = 0.06
IMPACT_INDIVIDUAL_POSITION_METRES = 0.025
IMPACT_PAIR_YAW_RADIANS = math.radians(6.0)
GUIDE_POSITION_METRES = 0.015
GUIDE_YAW_RADIANS = math.radians(1.5)

COLLECTION_NET_SOURCE = "guoshui_2026_collection_net_diagonal.obj"
COLLECTION_NET_SIDE = "guoshui_2026_collection_net_side_diagonal.obj"
COLLECTION_NET_BOTTOM = "guoshui_2026_collection_net_bottom_diagonal.obj"
COLLECTION_NET_ROD_VERTICES = 14
COLLECTION_NET_ROD_NORMALS = 8
COLLECTION_NET_ROD_FACES = 24


def add(a: Vec2, b: Vec2) -> Vec2:
    return (a[0] + b[0], a[1] + b[1])


def subtract(a: Vec2, b: Vec2) -> Vec2:
    return (a[0] - b[0], a[1] - b[1])


def scale(a: Vec2, factor: float) -> Vec2:
    return (a[0] * factor, a[1] * factor)


def unit(a: Vec2) -> Vec2:
    length = math.hypot(a[0], a[1])
    if length < 1.0e-12:
        raise ValueError("zero-length planar vector")
    return (a[0] / length, a[1] / length)


def rotate(a: Vec2, yaw: float) -> Vec2:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (cosine * a[0] - sine * a[1], sine * a[0] + cosine * a[1])


def transform_xy(point: Vec2, pivot: Vec2, offset: Vec2, yaw: float) -> Vec2:
    return add(add(pivot, offset), rotate(subtract(point, pivot), yaw))


def parse_vector(value: str) -> list[float]:
    values = [float(item) for item in value.split()]
    if len(values) != 3:
        raise ValueError(f"expected three values, got {value!r}")
    return values


def format_vector(values: list[float]) -> str:
    return " ".join(f"{value:.6f}" for value in values)


def split_collection_net(source: Path, side: Path, bottom: Path) -> None:
    """Split the diagonal collection mesh into side and bottom net assets."""

    lines = source.read_text(encoding="utf-8").splitlines()
    vertices = [line for line in lines if line.startswith("v ")]
    normals = [line for line in lines if line.startswith("vn ")]
    faces = [line for line in lines if line.startswith("f ")]
    if (
        len(vertices) % COLLECTION_NET_ROD_VERTICES
        or len(normals) % COLLECTION_NET_ROD_NORMALS
        or len(faces) % COLLECTION_NET_ROD_FACES
    ):
        raise ValueError(f"unexpected collection-net OBJ layout: {source}")

    rod_count = len(vertices) // COLLECTION_NET_ROD_VERTICES
    if len(normals) // COLLECTION_NET_ROD_NORMALS != rod_count:
        raise ValueError("collection-net vertex/normal rod counts do not match")
    if len(faces) // COLLECTION_NET_ROD_FACES != rod_count:
        raise ValueError("collection-net vertex/face rod counts do not match")

    # The side rods have a substantial Z span; the final horizontal rods are
    # centred at local z=0 and form the separate bottom panel.
    side_rods = []
    bottom_rods = []
    for rod in range(rod_count):
        rod_vertices = vertices[
            rod * COLLECTION_NET_ROD_VERTICES : (rod + 1) * COLLECTION_NET_ROD_VERTICES
        ]
        z_values = [float(line.split()[3]) for line in rod_vertices]
        target = side_rods if max(z_values) - min(z_values) > 0.005 else bottom_rods
        target.append(rod)
    if not side_rods or not bottom_rods:
        raise ValueError("collection-net OBJ did not contain side and bottom rods")
    if side_rods != list(range(side_rods[-1] + 1)):
        raise ValueError("collection-net side rods are not contiguous")
    if bottom_rods != list(range(side_rods[-1] + 1, rod_count)):
        raise ValueError("collection-net bottom rods are not contiguous")

    def write_part(path: Path, start_rod: int, end_rod: int, description: str) -> None:
        vertex_start = start_rod * COLLECTION_NET_ROD_VERTICES
        vertex_end = end_rod * COLLECTION_NET_ROD_VERTICES
        normal_start = start_rod * COLLECTION_NET_ROD_NORMALS
        normal_end = end_rod * COLLECTION_NET_ROD_NORMALS
        face_start = start_rod * COLLECTION_NET_ROD_FACES
        face_end = end_rod * COLLECTION_NET_ROD_FACES

        def renumber(match: re.Match[str]) -> str:
            vertex_index = int(match.group(1)) - vertex_start
            normal_index = int(match.group(2)) - normal_start
            return f"{vertex_index}//{normal_index}"

        output = [
            f"# {description}",
            "# Units: metres. White 3 mm-diameter rods.",
            "# Generated by generate_guoshui_2026_scene.py from the diagonal source mesh.",
            "o Guoshui2026CollectionNetPart",
        ]
        output.extend(vertices[vertex_start:vertex_end])
        output.extend(normals[normal_start:normal_end])
        output.extend(
            re.sub(r"(\d+)//(\d+)", renumber, line)
            for line in faces[face_start:face_end]
        )
        output.append(
            f"# counts: vertices={vertex_end - vertex_start}, "
            f"normals={normal_end - normal_start}, faces={face_end - face_start}"
        )
        path.write_text("\n".join(output) + "\n", encoding="utf-8")

    side.parent.mkdir(parents=True, exist_ok=True)
    write_part(side, side_rods[0], side_rods[-1] + 1, "Guoshui 2026 collection-frame side net.")
    write_part(bottom, bottom_rods[0], bottom_rods[-1] + 1, "Guoshui 2026 collection-frame bottom net.")


def matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[row][index] * b[index][column] for index in range(3)) for column in range(3)]
        for row in range(3)
    ]


def rpy_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    """Stonefish RPY convention: Rz(yaw) * Ry(pitch) * Rx(roll)."""

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def matrix_rpy(matrix: list[list[float]]) -> tuple[float, float, float]:
    pitch = math.asin(max(-1.0, min(1.0, -matrix[2][0])))
    if abs(math.cos(pitch)) > 1.0e-8:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        # The affected target objects are not at this singularity, but this
        # keeps the transform well-defined if the template is extended.
        roll = 0.0
        yaw = math.atan2(-matrix[0][1], matrix[1][1])
    return roll, pitch, yaw


def compose_world_yaw(rpy: list[float], yaw: float) -> list[float]:
    rotation = [
        [math.cos(yaw), -math.sin(yaw), 0.0],
        [math.sin(yaw), math.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ]
    return list(matrix_rpy(matmul(rotation, rpy_matrix(*rpy))))


def transform_pose_element(element: ET.Element, pivot: Vec2, offset: Vec2, yaw: float) -> None:
    """Apply one rigid world-Z transform to an XML pose element."""

    xyz = parse_vector(element.attrib["xyz"])
    xy = transform_xy((xyz[0], xyz[1]), pivot, offset, yaw)
    element.attrib["xyz"] = format_vector([xy[0], xy[1], xyz[2]])
    if "rpy" in element.attrib:
        element.attrib["rpy"] = format_vector(compose_world_yaw(parse_vector(element.attrib["rpy"]), yaw))


def transform_named_world_pose(
    root: ET.Element,
    name_predicate: callable,
    pivot: Vec2,
    offset: Vec2,
    yaw: float,
) -> None:
    for element in root:
        name = element.attrib.get("name", "")
        if name_predicate(name):
            world_transform = element.find("world_transform")
            if world_transform is not None:
                transform_pose_element(world_transform, pivot, offset, yaw)


def find_named(root: ET.Element, name: str) -> ET.Element:
    for element in root:
        if element.attrib.get("name") == name:
            return element
    raise KeyError(f"scene object {name!r} not found")


def set_xy(element: ET.Element, xy: Vec2) -> None:
    pose = element.find("world_transform")
    if pose is None:
        raise ValueError(f"{element.attrib.get('name')} has no world_transform")
    xyz = parse_vector(pose.attrib["xyz"])
    pose.attrib["xyz"] = format_vector([xy[0], xy[1], xyz[2]])


def jitter(rng: random.Random, amount: float, enabled: bool) -> float:
    return rng.uniform(-amount, amount) if enabled else 0.0


def sample_target_positions(rng: random.Random, enabled: bool) -> dict[str, Vec2]:
    """Sample all targets inside the rack with clearance and no fixed stacking order."""

    # The rack perimeter is centred at RACK_CENTER and has a 0.44 m span.
    # Limits account for each target's largest horizontal footprint, then leave
    # an additional visible gap to the white perimeter pipe.
    golf_limit = (
        RACK_HALF_EXTENT_METRES
        - TARGET_GOLF_RADIUS_METRES
        - TARGET_EDGE_MARGIN_METRES
    )
    ring_limit = (
        RACK_HALF_EXTENT_METRES
        - TARGET_RING_OUTER_RADIUS_METRES
        - TARGET_EDGE_MARGIN_METRES
    )

    # Keep seed 0 as a deterministic, safe nominal layout.  Non-zero seeds
    # use the same safe region but no longer preserve the old yellow/pink/ring
    # vertical ordering.
    if not enabled:
        return {
            "yellow": (0.11, -0.11),
            "pink": (-0.11, 0.10),
            "ring": (0.10, 0.10),
        }

    limits = {"yellow": golf_limit, "pink": golf_limit, "ring": ring_limit}
    names = ("yellow", "pink", "ring")
    for _ in range(TARGET_POSITION_RETRIES):
        positions = {
            name: (rng.uniform(-limits[name], limits[name]), rng.uniform(-limits[name], limits[name]))
            for name in names
        }
        if all(
            math.hypot(
                positions[left][0] - positions[right][0],
                positions[left][1] - positions[right][1],
            )
            >= TARGET_MIN_SEPARATION_METRES
            for index, left in enumerate(names)
            for right in names[index + 1 :]
        ):
            return positions

    raise RuntimeError("could not sample three separated targets inside the rack")


def seeded_layout(seed: int) -> dict[str, object]:
    """Return all sampled task centres and orientations for a seed."""

    rng = random.Random(seed)
    enabled = seed != 0

    rack_offset = (
        jitter(rng, RACK_POSITION_METRES, enabled),
        jitter(rng, RACK_POSITION_METRES, enabled),
    )
    rack_yaw = jitter(rng, RACK_YAW_RADIANS, enabled)
    collection_offset = (
        jitter(rng, COLLECTION_POSITION_METRES, enabled),
        jitter(rng, COLLECTION_POSITION_METRES, enabled),
    )
    collection_yaw = jitter(rng, COLLECTION_YAW_RADIANS, enabled)
    rack_center = add(RACK_CENTER, rack_offset)

    base_path = (IMPACT_CENTER,) + tuple(gate[1] for gate in GATES) + (RACK_CENTER,)
    gate_specs: list[tuple[str, Vec2, float, float]] = []
    gate_yaw_offsets: list[float] = []
    for index, (name, base_center, top_z, bottom_z) in enumerate(GATES):
        travel_direction = unit(subtract(base_path[index + 2], base_path[index]))
        lateral_direction = (-travel_direction[1], travel_direction[0])
        longitudinal = jitter(rng, GATE_LONGITUDINAL_METRES, enabled)
        lateral = jitter(rng, GATE_LATERAL_METRES, enabled)
        center = add(
            base_center,
            add(scale(travel_direction, longitudinal), scale(lateral_direction, lateral)),
        )
        gate_specs.append((name, center, top_z, bottom_z))
        gate_yaw_offsets.append(jitter(rng, GATE_YAW_RADIANS, enabled))

    impact_group_offset = (
        jitter(rng, IMPACT_GROUP_POSITION_METRES, enabled),
        jitter(rng, IMPACT_GROUP_POSITION_METRES, enabled),
    )
    impact_pair_yaw = jitter(rng, IMPACT_PAIR_YAW_RADIANS, enabled)
    impact_pair_center = add(IMPACT_CENTER, impact_group_offset)
    impact_blue = add(
        add(impact_pair_center, rotate((0.25, 0.0), impact_pair_yaw)),
        (
            jitter(rng, IMPACT_INDIVIDUAL_POSITION_METRES, enabled),
            jitter(rng, IMPACT_INDIVIDUAL_POSITION_METRES, enabled),
        ),
    )
    impact_red = add(
        add(impact_pair_center, rotate((-0.25, 0.0), impact_pair_yaw)),
        (
            jitter(rng, IMPACT_INDIVIDUAL_POSITION_METRES, enabled),
            jitter(rng, IMPACT_INDIVIDUAL_POSITION_METRES, enabled),
        ),
    )

    return {
        "seed": seed,
        "rack_offset": rack_offset,
        "rack_yaw": rack_yaw,
        "rack_center": rack_center,
        "collection_offset": collection_offset,
        "collection_yaw": collection_yaw,
        "gates": tuple(gate_specs),
        "gate_yaw_offsets": tuple(gate_yaw_offsets),
        "impact_blue": impact_blue,
        "impact_red": impact_red,
        "target_positions": sample_target_positions(rng, enabled),
        "yellow_yaw": jitter(rng, TARGET_ITEM_YAW_RADIANS, enabled),
        "pink_yaw": jitter(rng, TARGET_ITEM_YAW_RADIANS, enabled),
        "ring_yaw": jitter(rng, TARGET_ITEM_YAW_RADIANS, enabled),
        "guide_rng": rng,
        "guide_enabled": enabled,
    }


def update_guides(root: ET.Element, layout: dict[str, object]) -> None:
    """Rebuild the six guide-line poses from the perturbed task centres."""

    gate_centres = tuple(gate[1] for gate in layout["gates"])
    impact_centre = scale(add(layout["impact_blue"], layout["impact_red"]), 0.5)
    route = (START_CENTER, impact_centre) + gate_centres + (layout["rack_center"],)
    names = (
        "GuideStartToImpact",
        "GuideImpactToGate1",
        "GuideGate1ToGate2",
        "GuideGate2ToGate3",
        "GuideGate3ToGate4",
        "GuideGate4ToTargetRack",
    )
    rng: random.Random = layout["guide_rng"]
    enabled: bool = layout["guide_enabled"]
    for name, start, end in zip(names, route, route[1:]):
        direction = unit(subtract(end, start))
        normal = (-direction[1], direction[0])
        midpoint = scale(add(start, end), 0.5)
        position = add(
            midpoint,
            add(
                scale(direction, jitter(rng, GUIDE_POSITION_METRES, enabled)),
                scale(normal, jitter(rng, GUIDE_POSITION_METRES, enabled)),
            ),
        )
        yaw = math.atan2(direction[1], direction[0]) + jitter(rng, GUIDE_YAW_RADIANS, enabled)
        guide = find_named(root, name)
        pose = guide.find("world_transform")
        if pose is None:
            raise ValueError(f"{name} has no world_transform")
        xyz = parse_vector(pose.attrib["xyz"])
        pose.attrib["xyz"] = format_vector([position[0], position[1], xyz[2]])
        pose.attrib["rpy"] = format_vector([0.0, 0.0, yaw])


def generate(seed: int, template: Path, output: Path) -> dict[str, object]:
    if template.resolve() == output.resolve():
        raise ValueError("seeded output must not overwrite the fixed template")

    split_collection_net(
        template.parent / COLLECTION_NET_SOURCE,
        template.parent / COLLECTION_NET_SIDE,
        template.parent / COLLECTION_NET_BOTTOM,
    )

    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(template, parser=parser)
    root = tree.getroot()
    layout = seeded_layout(seed)

    # Gate geometry is generated in world coordinates and uses the same six
    # task centres as the guide lines below.
    path_points = (
        scale(add(layout["impact_blue"], layout["impact_red"]), 0.5),
        *(gate[1] for gate in layout["gates"]),
        layout["rack_center"],
    )
    write_gate_parts(
        output.parent,
        gates=layout["gates"],
        path_points=path_points,
        opening_yaw_offsets=layout["gate_yaw_offsets"],
        names=tuple(SEEDED_GATE_MODELS.values()),
    )
    for scene_name, mesh_name in SEEDED_GATE_MODELS.items():
        mesh = find_named(root, scene_name).find("physical/mesh")
        if mesh is None:
            raise ValueError(f"{scene_name} has no physical mesh")
        mesh.attrib["filename"] = mesh_name

    # Move and rotate every rack component as one rigid assembly.
    transform_named_world_pose(
        root,
        lambda name: name.startswith("TargetRack"),
        RACK_CENTER,
        layout["rack_offset"],
        layout["rack_yaw"],
    )

    # Targets are sampled in the rack's local XY plane, so their full shapes
    # remain inside the perimeter even when the whole rack is perturbed.
    target_positions: dict[str, Vec2] = layout["target_positions"]
    for name, key, target_yaw in (
        ("TargetYellowGolf", "yellow", layout["yellow_yaw"]),
        ("TargetPinkGolf", "pink", layout["pink_yaw"]),
    ):
        target = find_named(root, name)
        pose = target.find("world_transform")
        if pose is None:
            raise ValueError(f"{name} has no world_transform")
        local_xy = target_positions[key]
        pre_rack_xy = add(RACK_CENTER, local_xy)
        set_xy(target, pre_rack_xy)
        transform_pose_element(
            pose,
            pre_rack_xy,
            (0.0, 0.0),
            target_yaw,
        )
        transform_pose_element(
            pose, RACK_CENTER, layout["rack_offset"], layout["rack_yaw"]
        )
    for element in root:
        if element.attrib.get("name", "").startswith("TargetRedRing"):
            pose = element.find("world_transform")
            if pose is not None:
                ring_pre_rack_center = add(RACK_CENTER, target_positions["ring"])
                transform_pose_element(
                    pose,
                    RING_CENTER,
                    subtract(ring_pre_rack_center, RING_CENTER),
                    layout["ring_yaw"],
                )
                transform_pose_element(pose, RACK_CENTER, layout["rack_offset"], layout["rack_yaw"])

    # Collection frame and its rendered nets move as one XY assembly.
    transform_named_world_pose(
        root,
        lambda name: name.startswith("Collection"),
        COLLECTION_CENTER,
        layout["collection_offset"],
        layout["collection_yaw"],
    )
    for collection_net_name in ("CollectionNetSideVisual", "CollectionNetVisual"):
        collection_net = find_named(root, collection_net_name)
        for keypoint in collection_net.findall("trajectory/keypoint"):
            transform_pose_element(
                keypoint,
                COLLECTION_CENTER,
                layout["collection_offset"],
                layout["collection_yaw"],
            )

    # Both visual ropes follow the perturbed ball centres exactly.
    set_xy(find_named(root, "ImpactBallBlue"), layout["impact_blue"])
    set_xy(find_named(root, "ImpactBallRed"), layout["impact_red"])
    set_xy(find_named(root, "ImpactBallBlueVisibleTether"), layout["impact_blue"])
    set_xy(find_named(root, "ImpactBallRedVisibleTether"), layout["impact_red"])
    update_guides(root, layout)

    root.insert(0, ET.Comment(f" Generated from guoshui_2026_cruise.scn with scene_seed={seed}. "))
    indent_xml(tree)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    tree.write(temporary, encoding="utf-8", xml_declaration=True)
    temporary.replace(output)

    print(f"scene_seed={seed}")
    print(f"scenario={output}")
    print("gates=" + "; ".join(f"{name}@({xy[0]:.3f},{xy[1]:.3f})" for name, xy, _, _ in layout["gates"]))
    print(f"rack=({layout['rack_center'][0]:.3f},{layout['rack_center'][1]:.3f})")
    print("targets_local=" + "; ".join(
        f"{name}@({xy[0]:.3f},{xy[1]:.3f})"
        for name, xy in layout["target_positions"].items()
    ))
    print(f"collection=({COLLECTION_CENTER[0] + layout['collection_offset'][0]:.3f},{COLLECTION_CENTER[1] + layout['collection_offset'][1]:.3f})")
    return layout


def main() -> None:
    data_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, required=True, help="integer scene seed; 0 is the unperturbed baseline")
    parser.add_argument(
        "--template",
        type=Path,
        default=data_dir / "guoshui_2026_cruise.scn",
        help="fixed SCN template",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=data_dir / "guoshui_2026_cruise_seeded.scn",
        help="generated SCN output",
    )
    arguments = parser.parse_args()
    generate(arguments.seed, arguments.template, arguments.output)


if __name__ == "__main__":
    main()
