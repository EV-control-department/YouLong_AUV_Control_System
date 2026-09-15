#!/usr/bin/env python3
"""T2: dump the live TF tree and diff it against the documented extrinsics.

The simulation publishes **one** transform -- ``world_ned -> Vehicle/base_link``
(``ROS2SimulationManager.cpp:395``) -- and no per-sensor static transforms, so
the sensor extrinsics live only in ``docs/frames/extrinsics_reference.md`` and
on the real vehicle only in the mounting measurements.  This tool makes that
statement checkable instead of assumed.

    # with a live sim (or the real stack) running
    source /opt/ros/jazzy/setup.bash
    source workspace_auv/install/setup.bash
    source workspace_sim/install/setup.bash
    python3 workspace_sim/tools/dump_tf_tree.py

    # CI / no ROS: exercise the report logic against a synthetic tree
    python3 workspace_sim/tools/dump_tf_tree.py --self-test

The simulator does **not** need a GPU for this: ``stonefish_simulator_nogpu`` runs
the same ``ROS2SimulationManager``, so it publishes TF (if any) without rendering.
It only needs to be a **separate process** from this one.  Strip cameras with
``tools/strip_vision_sensors.py`` and pass ``<rate> 0`` (real-time) if you use the
console build.

What "healthy" means today
--------------------------
``<ros_base_link_transform publish="true"/>`` appears in **none** of the scenes, so
``publishBaseLinkTransform_`` (default ``false``, ``ROS2SimulationManager.h``) keeps
the TF broadcast off and this tool reports "no TF received".  That is the documented
state (``docs/frames/frame_convention.md`` section 3), **not** a failure of the stack:
a run with no TF and no unexpected sensor frames is the expected outcome.  The tool
therefore also reports which sensor frames are present, so the day somebody enables
TF (or adds sensor statics) the diff shows up instead of passing silently.

Exit codes: 0 unexpected free pass, 1 frames disagree with the docs, 2 environment not ready.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

#: Parent frame of the body transform, when the scene enables it.
EXPECTED_ROOT = "world_ned"

#: Frames named by the sensor blocks; if any of these show up in the TF tree
#: somebody added sensor statics (good, but the docs must be updated too).
SENSOR_FRAME_HINTS = (
    "dvl",
    "imu_filter",
    "front_cam",
    "down_cam",
    "pressure",
    "Vehicle",
)

#: Sensor extrinsics live here (derived from the scene by derive_extrinsics.py).
EXTRINSICS_YAML = (
    REPO_ROOT / "workspace_sim" / "src" / "uv_sim" / "config" / "extrinsics" / "xunyun.yaml"
)


def read_scene_expectations(scene: Path | None) -> tuple[str | None, bool]:
    """Return ``(robot_name, publishes_base_link_tf)`` for a vehicle scene.

    ``robot_name`` is the ``<robot name=...>`` of the vehicle scene (the frame
    prefix the simulator would use: ``XUNYUN`` in the canyon world, ``Vehicle``
    in the bare ``xunyun_fixed.scn``).  ``publishes_base_link_tf`` mirrors
    ``ROS2ScenarioParser.cpp``: the broadcast is on only when the scene has a
    ``<ros_base_link_transform publish="true"/>`` child of ``<robot>``.
    """
    if scene is None or not scene.is_file():
        return None, False
    import xml.etree.ElementTree as ET

    try:
        root = ET.parse(str(scene)).getroot()
    except ET.ParseError:
        return None, False
    for robot in root.iter("robot"):
        flag = robot.find("ros_base_link_transform")
        publishes = (
            flag is not None
            and flag.get("publish", "false").strip().lower() in ("true", "1")
        )
        name = robot.get("name")
        if name and name.startswith("$("):  # e.g. $(arg robot_name): caller decides
            name = None
        return name, publishes
    return None, False


def load_documented_frames(path: Path) -> list[str]:
    """Return the sensor names the extrinsics table documents."""
    import yaml

    if not path.is_file():
        return []
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return list((payload.get("sensors") or {}).keys())


def report(
    edges: list[tuple[str, str]],
    documented: list[str],
    scene: Path | None = None,
    scene_declares_tf: bool = False,
) -> tuple[bool, list[str]]:
    """Compare live ``(parent, child)`` edges against what the docs claim.

    ``expected_body`` is taken from the scene's ``<robot name>`` when known, so a
    world scene that renames the vehicle (canyon passes ``XUNYUN``) does not
    produce a bogus mismatch against a hard-coded ``Vehicle``.

    Returns ``(ok, lines)`` where ``lines`` is the printable report.
    """
    lines: list[str] = []
    children = {child for _, child in edges}
    robot_name, publishes_tf = (
        read_scene_expectations(scene) if scene is not None else (None, scene_declares_tf)
    )
    publishes_tf = publishes_tf or scene_declares_tf
    expected_body = f"{robot_name}/base_link" if robot_name else None
    body_frames = {child for child in children if child.endswith("base_link")}

    lines.append(f"live TF edges: {len(edges)}")
    for parent, child in sorted(edges):
        lines.append(f"  {parent} -> {child}")
    if scene is not None:
        lines.append(
            f"scene: {scene.name} -> robot {robot_name or '<unknown>'}, "
            f"<ros_base_link_transform> {'ON' if publishes_tf else 'absent'}"
        )

    problems: list[str] = []
    no_tf_expected = not edges and not publishes_tf
    if not edges and publishes_tf:
        problems.append(
            "no TF received although the scene declares "
            "<ros_base_link_transform publish=\"true\">: is the simulator running?"
        )
    roots = {parent for parent, _ in edges}
    if edges and EXPECTED_ROOT not in roots:
        problems.append(
            f"no edge from {EXPECTED_ROOT!r}; the body transform must be "
            f"{EXPECTED_ROOT} -> <robot>/base_link"
        )
    if edges and not body_frames:
        problems.append("no '<robot>/base_link' child frame found")
    if expected_body and body_frames and expected_body not in body_frames:
        problems.append(
            f"body frame is {sorted(body_frames)}, expected {expected_body!r} "
            f"from the scene <robot name=...>"
        )

    # Sensor frames: documented but not broadcast is EXPECTED today.  Report
    # which side each sensor is on so a future change cannot slip by silently.
    # Live frames may carry a robot prefix (``Vehicle/dvl``) while the
    # extrinsics table uses the bare sensor name, so compare leaf names.
    documented_set = set(documented)
    sensor_frames_live = {
        frame
        for frame in children
        if any(hint in frame for hint in SENSOR_FRAME_HINTS)
        and not frame.endswith("base_link")
    }
    live_leaves = {frame.rsplit("/", 1)[-1] for frame in sensor_frames_live}
    missing = sorted(documented_set - live_leaves)
    extra = sorted(frame for frame in sensor_frames_live if frame.rsplit("/", 1)[-1] not in documented_set)

    lines.append("")
    lines.append(f"documented sensor frames: {len(documented_set)}")
    lines.append(f"  broadcast in TF        : {sorted(sensor_frames_live) or 'none'}")
    if missing:
        lines.append(
            f"  documented, NOT in TF  : {missing}"
            "  <- expected: the sim has no per-sensor static TF"
        )
    if extra:
        lines.append(
            f"  in TF, NOT documented  : {extra}"
            "  <- update docs/frames/extrinsics_reference.md"
        )
        problems.append(f"undocumented sensor TF frames: {extra}")

    lines.append("")
    if problems:
        lines.extend(f"FAIL  {problem}" for problem in problems)
        return False, lines
    if no_tf_expected:
        lines.append(
            "OK    no TF, and the scene does not ask for it (no "
            "<ros_base_link_transform>) -- matches docs/frames/frame_convention.md section 3"
        )
    elif not sensor_frames_live:
        lines.append(
            f"OK    TF tree matches the documented state "
            f"(root {EXPECTED_ROOT}, no sensor statics)"
        )
    else:
        lines.append("OK    TF tree matches the documented state")
    return True, lines


def collect_live_edges(timeout_s: float) -> list[tuple[str, str]]:
    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = Node("dump_tf_tree")
    buffer = Buffer()
    TransformListener(buffer, node)
    edges: list[tuple[str, str]] = []
    deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
    try:
        import time

        while node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            frames = buffer.all_frames_as_yaml()
            if frames:
                import yaml

                data = yaml.safe_load(frames) or {}
                edges = [
                    (frame.removeprefix("/"), info.get("parent", "").removeprefix("/"))
                    for frame, info in data.items()
                    if info.get("parent")
                ]
                if edges:
                    break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return sorted(set(edges))


def _self_test() -> int:
    documented = ["dynamics", "pressure", "dvl", "imu_filter", "front_cam_left"]
    healthy = [("world_ned", "Vehicle/base_link")]
    ok, lines = report(healthy, documented, scene=None, scene_declares_tf=True)
    print("-- synthetic healthy tree --")
    print("\n".join(lines))
    if not ok:
        print("FAIL  self-test expected the healthy tree to pass")
        return 1
    # 1) 有 TF 但根不对、且冒出未文档化的传感器静态帧 -> 必须红
    broken = [("Vehicle/base_link", "Vehicle/dvl")]
    ok, lines = report(broken, documented, scene=None, scene_declares_tf=True)
    print("\n-- synthetic tree with a sensor static but no root --")
    print("\n".join(lines))
    if ok:
        print("FAIL  self-test expected the broken tree to fail")
        return 1

    print("\n-- synthetic tree with no TF and no scene declaration (today's state) --")
    ok, lines = report([], documented, scene=None, scene_declares_tf=False)
    print("\n".join(lines))
    if not ok:
        print("FAIL  self-test expected the no-TF state to pass")
        return 1

    print("\n-- same, but the scene asks for TF --")
    ok, lines = report([], documented, scene=None, scene_declares_tf=True)
    print("\n".join(lines))
    if ok:
        print("FAIL  self-test expected a missing-but-declared TF to fail")
        return 1

    print("\nPASS  self-test")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--timeout", type=float, default=5.0, help="seconds to wait for TF")
    parser.add_argument("--extrinsics", default=str(EXTRINSICS_YAML))
    parser.add_argument(
        "--scene",
        default=str(REPO_ROOT / "workspace_sim" / "src" / "stonefish_ros2" / "Data"
                    / "xunyun_fixed.scn"),
        help="vehicle scene to read the <robot name> and <ros_base_link_transform> from",
    )
    parser.add_argument(
        "--self-test", action="store_true", help="check the report logic without ROS"
    )
    args = parser.parse_args(argv)

    if args.self_test:
        return _self_test()

    documented = load_documented_frames(Path(args.extrinsics))
    if not documented:
        print(f"WARN  no extrinsics table at {args.extrinsics}; sensor diff will be empty")
    scene = Path(args.scene)
    try:
        edges = collect_live_edges(args.timeout)
    except ImportError as error:
        print(f"FAIL  ROS 2 python packages unavailable: {error}")
        return 2
    ok, lines = report(edges, documented, scene=scene)
    print("\n".join(lines))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
