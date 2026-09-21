#!/usr/bin/env python3
"""Validate Stonefish XML scenes, references and physical mesh topology."""

from __future__ import annotations

import argparse
from collections import Counter
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET


def mesh_closed(path: Path) -> bool:
    vertices = []
    faces = []
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if line.startswith("v "):
            vertices.append(tuple(float(value) for value in line.split()[1:4]))
        elif line.startswith("f "):
            # OBJ vertex references are one-based; the validator uses Python
            # zero-based indices below.
            face = [int(part.split("/")[0]) - 1 for part in line.split()[1:]]
            if len(face) >= 3:
                faces.append(face)
    if not vertices or not faces:
        return False
    edges: Counter[tuple[int, int]] = Counter()
    for face in faces:
        if any(index < 0 or index >= len(vertices) for index in face):
            return False
        anchor = vertices[face[0]]
        for first, second in zip(face, face[1:] + face[:1]):
            if first == second:
                return False
            edges[tuple(sorted((first, second)))] += 1
        # Reject zero-area triangles explicitly.  Edge multiplicity alone
        # does not catch three distinct collinear vertices.
        for index in range(1, len(face) - 1):
            second = vertices[face[index]]
            third = vertices[face[index + 1]]
            ab = tuple(second[i] - anchor[i] for i in range(3))
            ac = tuple(third[i] - anchor[i] for i in range(3))
            cross = (
                ab[1] * ac[2] - ab[2] * ac[1],
                ab[2] * ac[0] - ab[0] * ac[2],
                ab[0] * ac[1] - ab[1] * ac[0],
            )
            if math.sqrt(sum(component * component for component in cross)) <= 1.0e-12:
                return False
    return all(count == 2 for count in edges.values())



_CAMERA_SCENE_FRAMES = {
    "front_cam_left": "front_left_camera_optical_frame",
    "front_cam_right": "front_right_camera_optical_frame",
    "down_cam_left": "downward_left_camera_optical_frame",
    "down_cam_right": "downward_right_camera_optical_frame",
}


def _identity_rotation():
    return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _matmul(first, second):
    return tuple(
        tuple(sum(first[row][index] * second[index][column]
                  for index in range(3)) for column in range(3))
        for row in range(3)
    )


def _matvec(matrix, vector):
    return tuple(
        sum(matrix[row][index] * vector[index] for index in range(3))
        for row in range(3))


def _rotation_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _parse_float_vector(value, count, context):
    if value is None:
        raise ValueError(f"{context} is missing")
    values = [float(item) for item in value.split()]
    if len(values) != count:
        raise ValueError(f"{context} must contain {count} values")
    return tuple(values)


def _urdf_camera_transforms(urdf: Path):
    root = ET.parse(urdf).getroot()
    joints = {}
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        xyz = _parse_float_vector(
            origin.get("xyz", "0 0 0") if origin is not None else "0 0 0",
            3, f"{urdf}:{joint.get('name')}.xyz")
        rpy = _parse_float_vector(
            origin.get("rpy", "0 0 0") if origin is not None else "0 0 0",
            3, f"{urdf}:{joint.get('name')}.rpy")
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise ValueError(f"{urdf}:{joint.get('name')} has no parent/child")
        joints[child.get("link")] = (
            parent.get("link"), xyz, _rotation_from_rpy(*rpy))
    cache = {"base_link": ((0.0, 0.0, 0.0), _identity_rotation())}

    def resolve(link, visiting=()):
        if link in cache:
            return cache[link]
        if link in visiting or link not in joints:
            raise ValueError(f"{urdf}: cannot resolve link {link!r}")
        parent, translation, rotation = joints[link]
        parent_translation, parent_rotation = resolve(parent, visiting + (link,))
        offset = _matvec(parent_rotation, translation)
        resolved_translation = tuple(
            parent_translation[index] + offset[index] for index in range(3))
        resolved_rotation = _matmul(parent_rotation, rotation)
        cache[link] = (resolved_translation, resolved_rotation)
        return cache[link]

    return {
        frame: resolve(frame)
        for frame in _CAMERA_SCENE_FRAMES.values()
    }


def _rotation_distance(first, second):
    relative = _matmul(tuple(zip(*first)), second)
    trace = max(-1.0, min(3.0, sum(
        relative[index][index] for index in range(3))))
    return math.acos(max(-1.0, min(1.0, (trace - 1.0) * 0.5)))


def validate_camera_scene_origins(
        urdf: Path, scene: Path, translation_tolerance=1.0e-5,
        rotation_tolerance=2.0e-4) -> list[str]:
    """Ensure Stonefish camera origins match the authoritative simulation URDF."""
    errors = []
    try:
        urdf_transforms = _urdf_camera_transforms(urdf)
        scene_root = ET.parse(scene).getroot()
    except (ET.ParseError, OSError, ValueError) as error:
        return [f"{scene}: camera origin validation failed: {error}"]

    sensors = {
        sensor.get("name"): sensor
        for sensor in scene_root.iter("sensor")
        if sensor.get("type") == "camera"
    }
    for sensor_name, frame in _CAMERA_SCENE_FRAMES.items():
        sensor = sensors.get(sensor_name)
        if sensor is None:
            errors.append(f"{scene}: missing camera sensor {sensor_name}")
            continue
        origin = sensor.find("origin")
        try:
            scene_translation = _parse_float_vector(
                origin.get("xyz") if origin is not None else None,
                3, f"{scene}:{sensor_name}.xyz")
            scene_rpy = _parse_float_vector(
                origin.get("rpy") if origin is not None else None,
                3, f"{scene}:{sensor_name}.rpy")
            scene_rotation = _rotation_from_rpy(*scene_rpy)
        except (AttributeError, ValueError) as error:
            errors.append(str(error))
            continue
        expected_translation, expected_rotation = urdf_transforms[frame]
        delta = math.sqrt(sum(
            (scene_translation[index] - expected_translation[index]) ** 2
            for index in range(3)))
        angle = _rotation_distance(expected_rotation, scene_rotation)
        if delta > translation_tolerance or angle > rotation_tolerance:
            errors.append(
                f"{scene}: {sensor_name} origin disagrees with {urdf}:{frame}; "
                f"translation_error={delta:.6g}, rotation_error={angle:.6g}")
    return errors

def validate(root: Path) -> list[str]:
    errors: list[str] = []
    # ``legacy`` is intentionally preserved as a migration fixture with its
    # original bare resource names. Maintained project worlds and the
    # self-contained examples are checked against the package root.
    scenes = sorted((root / "worlds" / "guoshui_2026").glob("*.scn"))
    scenes += sorted((root / "worlds" / "sauvc_2026").glob("*.scn"))
    scenes += sorted((root / "worlds" / "examples").glob("*.scn"))
    scenes += [root / "vehicles/youlong/model/youlong.scn"]
    for scene in scenes:
        try:
            tree = ET.parse(scene)
        except ET.ParseError as error:
            errors.append(f"{scene}: XML parse error: {error}")
            continue
        root_element = tree.getroot()
        parents = {child: parent for parent in root_element.iter() for child in parent}
        for element in root_element.iter():
            for attribute in ("filename", "texture"):
                value = element.get(attribute)
                if not value or value.startswith(("/", "~", "$")):
                    continue
                target = root / value
                if not target.is_file():
                    errors.append(f"{scene}: missing {attribute} {value}")
            include = element.get("file") if element.tag == "include" else None
            if element.tag == "include" and parents.get(element) is not root_element:
                errors.append(f"{scene}: include must be at scenario root")
            if include and not include.startswith(("/", "~", "$")):
                if not (root / include).is_file():
                    errors.append(f"{scene}: missing include {include}")
                elif include == "vehicles/youlong/model/youlong.scn":
                    arguments = {
                        child.get("name"): child.get("value")
                        for child in element.findall("arg")
                    }
                    if arguments.get("robot_name") != "youlong":
                        errors.append(f"{scene}: YouLong include has invalid robot_name")
                    if not arguments.get("robot_position"):
                        errors.append(f"{scene}: YouLong include has no robot_position")
                    if "robot_orientation" not in arguments:
                        errors.append(f"{scene}: YouLong include has no robot_orientation")
    physical = root / "vehicles/youlong/meshes/physical/youlong_collision_hydro.obj"
    if not mesh_closed(physical):
        errors.append(f"{physical}: physical mesh is not closed")
    vehicle = root / "vehicles/youlong/model/youlong.scn"
    simulation_urdf = root.parent / "uv_sim_description/urdf/auv_sim.urdf"
    if simulation_urdf.is_file():
        errors.extend(validate_camera_scene_origins(
            simulation_urdf, vehicle))
    else:
        errors.append(f"{simulation_urdf}: simulation URDF is missing")
    try:
        robot = ET.parse(vehicle).getroot().find("robot")
    except (ET.ParseError, OSError):
        robot = None
    if robot is not None:
        thrusters = [
            element.get("name")
            for element in robot.findall("actuator")
            if element.get("type") == "thruster"
        ]
        expected_thrusters = [f"Thruster{index}" for index in range(6)]
        if thrusters != expected_thrusters:
            errors.append(
                f"{vehicle}: thruster order {thrusters!r} != "
                f"{expected_thrusters!r}")
        topics = {
            element.get("topic")
            for element in robot.findall("sensor/ros_publisher")
            if element.get("topic")
        }
        required_topics = {
            "/auv/sim/ground_truth/odom",
            "/auv/sensors/pressure",
            "/auv/sim/raw/dvl/velocity",
            "/auv/sensors/imu/data",
        }
        missing_topics = sorted(required_topics - topics)
        if missing_topics:
            errors.append(f"{vehicle}: missing sensor topics {missing_topics}")
        expected_camera_frames = {
            "/auv/sim/raw/camera/front/left": "front_left_camera_optical_frame",
            "/auv/sim/raw/camera/front/right": "front_right_camera_optical_frame",
            "/auv/sim/raw/camera/downward/left": "downward_left_camera_optical_frame",
            "/auv/sim/raw/camera/downward/right": "downward_right_camera_optical_frame",
        }
        camera_publishers = {
            element.get("topic"): element.get("frame_id")
            for element in robot.findall("sensor/ros_publisher")
            if element.get("topic") in expected_camera_frames
        }
        for topic, frame in expected_camera_frames.items():
            if camera_publishers.get(topic) != frame:
                errors.append(
                    f"{vehicle}: {topic} frame_id {camera_publishers.get(topic)!r} "
                    f"!= {frame!r}")
        if robot.get("name") != "$(arg robot_name)":
            errors.append(f"{vehicle}: robot name is not argument-driven")
    return errors


def maintained_worlds(root: Path) -> list[Path]:
    """Return worlds covered by the maintained asset validation gate."""

    scenes = sorted((root / "worlds" / "guoshui_2026").glob("*.scn"))
    scenes += sorted((root / "worlds" / "sauvc_2026").glob("*.scn"))
    return [scene for scene in scenes if scene.is_file()]


def _make_console_fixture(root: Path, destination: Path) -> None:
    """Copy assets and remove only camera sensors for Stonefish console mode.

    Stonefish's ``ROS2ConsoleSimulationApp`` deliberately rejects camera
    sensors because it has no rendering context.  The production description
    must keep those sensors, so the headless parser gate uses this disposable
    fixture.  Geometry, masses, hydrodynamics, thrusters and non-visual
    sensors remain byte-for-byte identical.
    """

    shutil.copytree(root, destination)
    vehicle = destination / "vehicles/youlong/model/youlong.scn"
    tree = ET.parse(vehicle)
    removed = 0
    for robot in tree.getroot().findall("robot"):
        for sensor in list(robot.findall("sensor")):
            if sensor.get("type") == "camera":
                robot.remove(sensor)
                removed += 1
    if removed == 0:
        raise ValueError("canonical vehicle has no camera sensors to sanitize")
    tree.write(vehicle, encoding="utf-8", xml_declaration=True)


def validate_with_stonefish(
    root: Path,
    binary: Path,
    timeout_seconds: float = 20.0,
    simulation_rate: float = 20.0,
) -> list[str]:
    """Parse every maintained world with Stonefish's no-GPU application.

    The application keeps running after parsing, therefore success is detected
    from its parser log and the process is terminated cleanly.  A temporary
    camera-free fixture is used for the console parser; the regular validator
    still checks the full production scenes and camera references.
    """

    binary = binary.resolve()
    if not binary.is_file():
        return [f"Stonefish binary does not exist: {binary}"]

    errors: list[str] = []
    with tempfile.TemporaryDirectory(prefix="uv_sim_assets_stonefish_") as temporary:
        stage = Path(temporary) / "assets"
        try:
            _make_console_fixture(root, stage)
        except (OSError, ET.ParseError, ValueError) as error:
            return [f"could not prepare Stonefish console fixture: {error}"]

        for world in maintained_worlds(root):
            relative_world = world.relative_to(root)
            staged_world = stage / relative_world
            log_dir = Path(temporary) / "ros_log" / relative_world.stem
            log_dir.mkdir(parents=True, exist_ok=True)
            parser_log = log_dir / "stonefish_ros2_parser.log"
            environment = os.environ.copy()
            environment["ROS_LOG_DIR"] = str(log_dir)
            environment.setdefault("ROS_LOCALHOST_ONLY", "1")
            command = [str(binary), str(stage), str(staged_world), str(simulation_rate)]
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                env=environment,
                start_new_session=True,
            )
            deadline = time.monotonic() + timeout_seconds
            output: list[str] = []
            parsed = False
            failed = False
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    break
                if parser_log.is_file():
                    parser_text = parser_log.read_text(encoding="utf-8", errors="replace")
                    if "Parsing finished normally." in parser_text:
                        parsed = True
                        break
                    if any(
                        marker in parser_text
                        for marker in (
                            "Parsing of scenario file",
                            "Pre-processing of included file",
                            "Including files failed",
                            "Robot not properly defined",
                            "Sensor of robot",
                        )
                    ):
                        failed = True
                        break
                time.sleep(0.05)
            if not parsed and parser_log.is_file():
                parser_text = parser_log.read_text(encoding="utf-8", errors="replace")
                parsed = "Parsing finished normally." in parser_text
                failed = failed or any(
                    marker in parser_text
                    for marker in (
                        "Parsing of scenario file",
                        "Pre-processing of included file",
                        "Including files failed",
                        "Robot not properly defined",
                        "Sensor of robot",
                    )
                )
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3.0)
            if process.stdout is not None:
                try:
                    output = process.stdout.read().splitlines()
                except (OSError, ValueError):
                    output = []
            if not parsed:
                detail = "\n".join(output[-12:])
                if parser_log.is_file():
                    detail = parser_log.read_text(encoding="utf-8", errors="replace").strip() or detail
                if failed:
                    reason = "parser reported failure"
                elif process.returncode is not None:
                    reason = f"Stonefish exited with code {process.returncode}"
                else:
                    reason = "parser did not finish before timeout"
                errors.append(f"{relative_world}: {reason}\n{detail}".rstrip())
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path, nargs="?", default=Path(__file__).resolve().parents[1])
    parser.add_argument(
        "--stonefish-binary",
        type=Path,
        help="also start this stonefish_simulator_nogpu binary for every maintained world",
    )
    parser.add_argument("--stonefish-timeout", type=float, default=20.0)
    parser.add_argument("--stonefish-rate", type=float, default=20.0)
    args = parser.parse_args()
    root = args.root.resolve()
    errors = validate(root)
    if args.stonefish_binary:
        errors.extend(
            validate_with_stonefish(
                root,
                args.stonefish_binary,
                timeout_seconds=args.stonefish_timeout,
                simulation_rate=args.stonefish_rate,
            )
        )
    if errors:
        print("\n".join(errors))
        return 1
    print(f"validated {len(list((args.root / 'worlds').rglob('*.scn')))} worlds")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
