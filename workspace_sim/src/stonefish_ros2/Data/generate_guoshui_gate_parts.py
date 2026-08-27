#!/usr/bin/env python3
"""Generate the four fixed Guoshui 2026 gate parts.

The scene uses one OBJ per visual class because Stonefish applies one look to
each model node.  All coordinates are already in the scene's NED frame, so the
generated models use an identity world transform in the SCN file.
"""

from __future__ import annotations

import math
from pathlib import Path


Vec = tuple[float, float, float]


def add(a: Vec, b: Vec) -> Vec:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vec, b: Vec) -> Vec:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def mul(a: Vec, s: float) -> Vec:
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a: Vec, b: Vec) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vec, b: Vec) -> Vec:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def unit(a: Vec) -> Vec:
    length = math.sqrt(dot(a, a))
    if length < 1.0e-12:
        raise ValueError("zero-length vector")
    return mul(a, 1.0 / length)


def basis(axis: Vec) -> tuple[Vec, Vec]:
    """Return two perpendicular vectors for a cylinder cross-section."""

    axis = unit(axis)
    reference = (0.0, 0.0, 1.0)
    if abs(dot(axis, reference)) > 0.90:
        reference = (0.0, 1.0, 0.0)
    u = unit(cross(axis, reference))
    v = unit(cross(axis, u))
    return u, v


class Mesh:
    def __init__(self) -> None:
        self.vertices: list[Vec] = []
        self.normals: list[Vec] = []
        self.faces: list[tuple[int, int, int, int, int, int]] = []

    def vertex(self, position: Vec, normal: Vec) -> tuple[int, int]:
        self.vertices.append(position)
        self.normals.append(unit(normal))
        return len(self.vertices), len(self.normals)

    def face(self, a: tuple[int, int], b: tuple[int, int], c: tuple[int, int]) -> None:
        self.faces.append((a[0], a[1], b[0], b[1], c[0], c[1]))

    def face_with_normal(self, vertex_indices: tuple[int, int, int], normal: Vec) -> None:
        normal_index = len(self.normals) + 1
        self.normals.append(unit(normal))
        self.faces.append(
            (
                vertex_indices[0], normal_index,
                vertex_indices[1], normal_index,
                vertex_indices[2], normal_index,
            )
        )

    def write(self, path: Path, description: str) -> None:
        lines = [
            f"# {description}",
            "# All faces are triangles; no MTL file is required by Stonefish.",
            "o Guoshui2026GatePart",
        ]
        lines.extend(f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in self.vertices)
        lines.extend(f"vn {x:.6f} {y:.6f} {z:.6f}" for x, y, z in self.normals)
        lines.extend(
            f"f {a}//{an} {b}//{bn} {c}//{cn}"
            for a, an, b, bn, c, cn in self.faces
        )
        lines.append(
            f"# counts: vertices={len(self.vertices)}, normals={len(self.normals)}, "
            f"faces={len(self.faces)}"
        )
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def add_pipe(mesh: Mesh, p0: Vec, p1: Vec, radius: float, sides: int = 16) -> None:
    """Add a closed, triangulated solid cylinder between two world points."""

    axis = unit(sub(p1, p0))
    u, v = basis(axis)
    start: list[tuple[int, int]] = []
    end: list[tuple[int, int]] = []
    for i in range(sides):
        angle = 2.0 * math.pi * i / sides
        radial = unit(add(mul(u, math.cos(angle)), mul(v, math.sin(angle))))
        start.append(mesh.vertex(add(p0, mul(radial, radius)), radial))
        end.append(mesh.vertex(add(p1, mul(radial, radius)), radial))

    for i in range(sides):
        j = (i + 1) % sides
        # The order gives the outside wall an outward-facing winding.
        mesh.face(start[i], start[j], end[j])
        mesh.face(start[i], end[j], end[i])

    start_center = mesh.vertex(p0, mul(axis, -1.0))[0]
    end_center = mesh.vertex(p1, axis)[0]
    for i in range(sides):
        j = (i + 1) % sides
        mesh.face_with_normal((start_center, start[j][0], start[i][0]), mul(axis, -1.0))
        mesh.face_with_normal((end_center, end[i][0], end[j][0]), axis)


def add_elbow(
    mesh: Mesh,
    center: Vec,
    horizontal_direction: Vec,
    vertical_direction: Vec,
    outer_radius: float = 0.028,
    inner_radius: float = 0.0205,
    leg_length: float = 0.055,
    bend_radius: float = 0.025,
    sides: int = 16,
    arc_steps: int = 8,
) -> None:
    """Add one hollow 90-degree PVC elbow with two open ends.

    The centerline starts along the horizontal pipe and turns through a
    quarter-circle into the vertical pipe.  It is one continuous mesh, rather
    than two intersecting capped cylinders, so the corner cannot produce the
    black spike artifacts seen with the previous sleeve construction.
    """

    a = unit(horizontal_direction)
    b = unit(vertical_direction)
    if abs(dot(a, b)) > 1.0e-6:
        raise ValueError("elbow directions must be perpendicular")

    points: list[Vec] = [add(center, mul(a, leg_length)), add(center, mul(a, bend_radius))]
    tangents: list[Vec] = [mul(a, -1.0), mul(a, -1.0)]
    for step in range(1, arc_steps + 1):
        theta = math.pi * 0.5 * step / arc_steps
        arc_center = add(add(center, mul(a, bend_radius)), mul(b, bend_radius))
        point = sub(
            sub(arc_center, mul(b, bend_radius * math.cos(theta))),
            mul(a, bend_radius * math.sin(theta)),
        )
        tangent = unit(add(mul(a, -math.cos(theta)), mul(b, math.sin(theta))))
        points.append(point)
        tangents.append(tangent)
    points.append(add(center, mul(b, leg_length)))
    tangents.append(b)

    rings: list[tuple[list[tuple[int, int]], list[tuple[int, int]]]] = []
    for point, tangent in zip(points, tangents):
        u, v = basis(tangent)
        outer: list[tuple[int, int]] = []
        inner: list[tuple[int, int]] = []
        for i in range(sides):
            angle = 2.0 * math.pi * i / sides
            radial = unit(add(mul(u, math.cos(angle)), mul(v, math.sin(angle))))
            outer.append(mesh.vertex(add(point, mul(radial, outer_radius)), radial))
            inner.append(mesh.vertex(add(point, mul(radial, inner_radius)), mul(radial, -1.0)))
        rings.append((outer, inner))

    for ring_index in range(len(rings) - 1):
        outer0, inner0 = rings[ring_index]
        outer1, inner1 = rings[ring_index + 1]
        for i in range(sides):
            j = (i + 1) % sides
            mesh.face(outer0[i], outer0[j], outer1[j])
            mesh.face(outer0[i], outer1[j], outer1[i])
            mesh.face(inner0[i], inner1[j], inner0[j])
            mesh.face(inner0[i], inner1[i], inner1[j])

    # The two open annuli expose the hollow bore.
    first_outer, first_inner = rings[0]
    last_outer, last_inner = rings[-1]
    start_normal = mul(tangents[0], -1.0)
    end_normal = tangents[-1]
    for i in range(sides):
        j = (i + 1) % sides
        mesh.face_with_normal(
            (first_outer[i][0], first_inner[j][0], first_outer[j][0]),
            start_normal,
        )
        mesh.face_with_normal(
            (first_outer[i][0], first_inner[i][0], first_inner[j][0]),
            start_normal,
        )
        mesh.face_with_normal(
            (last_outer[i][0], last_outer[j][0], last_inner[j][0]),
            end_normal,
        )
        mesh.face_with_normal(
            (last_outer[i][0], last_inner[j][0], last_inner[i][0]),
            end_normal,
        )


GATES = (
    ("GateLow1", (7.30, 0.80), 0.60, 1.10),
    ("GateHigh1", (5.80, -0.40), 0.40, 0.90),
    ("GateLow2", (4.15, 0.85), 0.60, 1.10),
    ("GateHigh2", (2.75, 1.15), 0.40, 0.90),
)


def point(xy: tuple[float, float], z: float) -> Vec:
    return (xy[0], xy[1], z)


def build() -> tuple[Mesh, Mesh, Mesh, Mesh]:
    red_pipes = Mesh()
    white_supports = Mesh()
    red_sleeves = Mesh()
    white_sleeves = Mesh()
    pipe_radius = 0.020
    down = (0.0, 0.0, 1.0)

    # The path enters the first gate from the impact-ball area and leaves the
    # fourth gate toward the target display rack.  A gate's opening normal is
    # the angle bisector of the incoming and outgoing path directions; its
    # horizontal rails are perpendicular to that normal.
    path_points = ((8.70, 1.55),) + tuple(gate[1] for gate in GATES) + ((0.95, -0.45),)

    for index, (_, center_xy, top_z, bottom_z) in enumerate(GATES):
        previous_xy = path_points[index]
        next_xy = path_points[index + 2]
        incoming = unit(
            (center_xy[0] - previous_xy[0], center_xy[1] - previous_xy[1], 0.0)
        )
        outgoing = unit(
            (next_xy[0] - center_xy[0], next_xy[1] - center_xy[1], 0.0)
        )
        opening_normal = unit(add(incoming, outgoing))
        rail_direction = unit((-opening_normal[1], opening_normal[0], 0.0))
        left_xy = (
            center_xy[0] - 0.35 * rail_direction[0],
            center_xy[1] - 0.35 * rail_direction[1],
        )
        right_xy = (
            center_xy[0] + 0.35 * rail_direction[0],
            center_xy[1] + 0.35 * rail_direction[1],
        )
        left_top, right_top = point(left_xy, top_z), point(right_xy, top_z)
        left_bottom, right_bottom = point(left_xy, bottom_z), point(right_xy, bottom_z)

        for lower, upper in ((left_bottom, left_top), (right_bottom, right_top)):
            add_pipe(red_pipes, lower, upper, pipe_radius)
        add_pipe(red_pipes, left_top, right_top, pipe_radius)
        add_pipe(red_pipes, left_bottom, right_bottom, pipe_radius)

        for lower in (left_bottom, right_bottom):
            add_pipe(white_supports, lower, point((lower[0], lower[1]), 1.30), pipe_radius)

        # The top and bottom elbows each connect the rail toward the gate
        # center to the downward vertical pipe.
        add_elbow(red_sleeves, left_top, rail_direction, down)
        add_elbow(red_sleeves, right_top, mul(rail_direction, -1.0), down)
        add_elbow(white_sleeves, left_bottom, rail_direction, down)
        add_elbow(white_sleeves, right_bottom, mul(rail_direction, -1.0), down)

    return red_pipes, white_supports, red_sleeves, white_sleeves


def main() -> None:
    output_dir = Path(__file__).resolve().parent
    meshes = build()
    descriptions = (
        "Guoshui 2026 four gates: red PVC frame pipes.",
        "Guoshui 2026 four gates: white PVC support columns.",
        "Guoshui 2026 four gates: hollow red 90-degree PVC elbows.",
        "Guoshui 2026 four gates: hollow white 90-degree PVC elbows.",
    )
    names = (
        "guoshui_2026_gate_red_pipes.obj",
        "guoshui_2026_gate_white_supports.obj",
        "guoshui_2026_gate_red_sleeves.obj",
        "guoshui_2026_gate_white_sleeves.obj",
    )
    for mesh, name, description in zip(meshes, names, descriptions):
        mesh.write(output_dir / name, description)
        print(f"{name}: {len(mesh.vertices)} vertices, {len(mesh.faces)} triangles")


if __name__ == "__main__":
    main()
