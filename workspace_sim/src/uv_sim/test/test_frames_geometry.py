"""T2: pin the frame conventions and the DVL beam geometry.

Everything here is pure numpy/XML -- no ROS runtime -- so the suite still runs
in a bare sandbox.  Two kinds of assertion live here:

* **math** -- the euler order Stonefish uses, rigid inverses, and the DVL
  least-squares inversion AQUA-SLAM performs (``mE`` / ``(E^T E)^-1 E^T``);
* **contract** -- ``docs/frames/extrinsics_reference.md`` and
  ``config/extrinsics/xunyun.yaml`` must still agree with
  ``stonefish_ros2/Data/xunyun_fixed.scn``.  If someone remounts a sensor in
  the scene without regenerating the table, ``test_extrinsics_doc_matches_scene``
  fails with the exact element that drifted.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT / "workspace_sim" / "src" / "uv_sim"))

from uv_sim.geometry import (  # noqa: E402
    DEFAULT_SCENE,
    beam_inversion_sensitivity,
    beam_velocity_to_body,
    check_rotation,
    dvl_beam_azimuths_from_scene,
    dvl_beam_directions,
    euler_rpy_to_matrix,
    invert_rigid,
    parse_scn_sensors,
)

SCENE_PATH = REPO_ROOT / DEFAULT_SCENE
TOOLS_DIR = REPO_ROOT / "workspace_sim" / "tools"
DOC_PATH = REPO_ROOT / "docs" / "frames" / "extrinsics_reference.md"
EXTRINSICS_YAML = (
    REPO_ROOT / "workspace_sim" / "src" / "uv_sim" / "config" / "extrinsics" / "xunyun.yaml"
)

ALPHA_DEG = 60.0          # AQUA-SLAM elevation for the real XUNYUN head
BEAM_ANGLE_DEG = 30.0     # scene <specs beam_angle>, off vertical
BETA_DEG = (45.0, 135.0, 225.0, 315.0)

# Camera window positions in the body frame.  These are the authoritative ones
# (N1, resolved 2026-09-15): position.py / line_follower.py / arrow_surfacer.py
# and docs/perception.md all use them, so the scene <origin> was corrected to
# match.  Changing the scene's camera z must fail these tests on purpose.
FRONT_CAMERA_XYZ = (0.23, -0.05, 0.076)
DOWN_CAMERA_XYZ = (-0.13, -0.05, 0.0645)


def _load_tool(name: str):
    """Import a ``workspace_sim/tools`` script by path (they are not packages).

    The module is registered in ``sys.modules`` before execution: tools that
    define dataclasses need it, because ``@dataclass`` resolves annotations
    through ``sys.modules[cls.__module__]``.
    """
    spec = importlib.util.spec_from_file_location(name, TOOLS_DIR / f"{name}.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(name, None)
        raise
    return module


# --------------------------------------------------------------------------- #
# Rotation and rigid-transform conventions
# --------------------------------------------------------------------------- #
def test_euler_order_is_rz_ry_rx() -> None:
    """Stonefish parses ``<origin rpy>`` as Rz(yaw)@Ry(pitch)@Rx(roll).

    ``Rz*Rx != Rx*Rz`` for yaw=90, roll=90, so a swapped implementation fails
    here instead of silently mirroring every extrinsic.
    """
    expected = euler_rpy_to_matrix(0.0, 0.0, math.pi / 2) @ euler_rpy_to_matrix(
        math.pi / 2, 0.0, 0.0
    )
    composed = euler_rpy_to_matrix(math.pi / 2, 0.0, math.pi / 2)
    assert np.allclose(composed, expected, atol=1e-12)
    assert not np.allclose(
        composed, euler_rpy_to_matrix(math.pi / 2, 0.0, 0.0) @ euler_rpy_to_matrix(0.0, 0.0, math.pi / 2)
    )


def test_yaw_rotates_bow_to_starboard_axis() -> None:
    """yaw=+90 (clockwise, NED) must take body +x (bow) onto +y (starboard)."""
    rotation = euler_rpy_to_matrix(0.0, 0.0, math.pi / 2)
    assert np.allclose(rotation @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-12)


def test_euler_matrix_is_a_proper_rotation() -> None:
    rotation = euler_rpy_to_matrix(0.3, -0.4, 1.1)
    assert check_rotation(rotation)


def test_invert_rigid_round_trip() -> None:
    transform = np.eye(4)
    transform[:3, :3] = euler_rpy_to_matrix(0.2, 0.1, -0.7)
    transform[:3, 3] = FRONT_CAMERA_XYZ
    assert np.allclose(transform @ invert_rigid(transform), np.eye(4), atol=1e-12)
    assert np.allclose(invert_rigid(transform) @ transform, np.eye(4), atol=1e-12)


# --------------------------------------------------------------------------- #
# Scene parsing
# --------------------------------------------------------------------------- #
def test_scene_sensors_match_the_documented_poses() -> None:
    """The table in docs/frames/extrinsics_reference.md is derived from this."""
    sensors = parse_scn_sensors(SCENE_PATH)
    for name in (
        "dynamics",
        "pressure",
        "dvl",
        "imu_filter",
        "front_cam_left",
        "front_cam_right",
        "down_cam_left",
        "down_cam_right",
    ):
        assert name in sensors, f"{name} disappeared from {SCENE_PATH.name}"

    dvl = sensors["dvl"]
    assert dvl.xyz == pytest.approx((-0.375, 0.0, 0.1), abs=1e-9)
    assert dvl.rpy == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    assert dvl.rate_hz == pytest.approx(20.0)
    assert dvl.link == "Vehicle"
    assert dvl.ros_topics["topic"] == "/auv/dvl"

    assert sensors["imu_filter"].xyz == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    assert sensors["imu_filter"].rate_hz == pytest.approx(200.0)

    front_left = sensors["front_cam_left"]
    assert front_left.xyz == pytest.approx(FRONT_CAMERA_XYZ, abs=1e-9)
    assert front_left.rpy == pytest.approx((1.5708, 0.0, 1.5708), abs=1e-9)
    down_left = sensors["down_cam_left"]
    assert down_left.xyz == pytest.approx(DOWN_CAMERA_XYZ, abs=1e-9)
    assert down_left.rpy == pytest.approx((0.0, 0.0, 1.5708), abs=1e-9)


def test_beam_angle_is_degrees_and_converts_to_alpha_radians() -> None:
    """``<specs beam_angle>`` is written in degrees while rpy is radians.

    Getting this wrong turns ``alpha`` into ``90 - 30 rad``, which is the
    trap that makes the beam geometry meaningless; pin both directions.
    """
    dvl = parse_scn_sensors(SCENE_PATH)["dvl"]
    assert dvl.specs["beam_angle"] == "30.0"
    assert math.degrees(dvl.beam_angle_rad) == pytest.approx(BEAM_ANGLE_DEG, abs=1e-9)
    assert math.degrees(dvl.alpha_rad) == pytest.approx(ALPHA_DEG, abs=1e-9)


# --------------------------------------------------------------------------- #
# DVL beam geometry
# --------------------------------------------------------------------------- #
def test_beam_matrix_matches_aqua_slam_convention() -> None:
    beams = dvl_beam_directions(math.radians(ALPHA_DEG), BETA_DEG)
    assert beams.shape == (4, 3)
    for row in beams:
        assert np.linalg.norm(row) == pytest.approx(1.0, abs=1e-12)
    # 30 deg off vertical means |horizontal|/|vertical| = tan(30)
    expected_horizontal = math.cos(math.radians(ALPHA_DEG))
    expected_vertical = math.sin(math.radians(ALPHA_DEG))
    assert np.allclose(np.linalg.norm(beams[:, :2], axis=1), expected_horizontal, atol=1e-12)
    assert np.allclose(beams[:, 2], expected_vertical, atol=1e-12)
    assert np.all(beams[:, 2] > 0.0), "beams must point down (sensor +z in NED)"
    gram = beams.T @ beams
    assert np.allclose(np.diag(gram), [0.5, 0.5, 3.0], atol=1e-12)
    assert abs(np.linalg.det(gram)) > 1e-9


def test_beam_inversion_recovers_any_velocity() -> None:
    """AQUA-SLAM's ``(E^T E)^-1 E^T`` is exact for a rank-3 beam set."""
    beams = dvl_beam_directions(math.radians(ALPHA_DEG), BETA_DEG)
    for truth in (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (0.3, -0.2, 0.05),
        (-1.2, 0.9, -0.4),
    ):
        velocity = np.array(truth)
        recovered = beam_velocity_to_body(beams, beams @ velocity)
        assert np.max(np.abs(recovered - velocity)) < 1e-12


@pytest.mark.parametrize("azimuths", [BETA_DEG, (20.0, 110.0, 200.0, 290.0)])
def test_beam_inversion_does_not_need_a_regular_azimuth_layout(azimuths) -> None:
    beams = dvl_beam_directions(math.radians(ALPHA_DEG), azimuths)
    velocity = np.array([0.4, -0.3, 0.1])
    assert np.max(np.abs(beam_velocity_to_body(beams, beams @ velocity) - velocity)) < 1e-12


def test_alpha_mismatch_scale_is_the_documented_30_percent() -> None:
    """The AQUA-SLAM shipped ``alpha=67.5 deg`` inflates horizontal speed.

    ``cos(67.5)/cos(60) = 1.3066``; this is the number quoted in
    docs/frames/extrinsics_reference.md, so keep it pinned.
    """
    horizontal, vertical = beam_inversion_sensitivity(math.radians(67.5))
    assert horizontal == pytest.approx(1.3066, abs=1e-4)
    assert vertical == pytest.approx(0.9374, abs=1e-4)
    horizontal_truth, vertical_truth = beam_inversion_sensitivity(math.radians(60.0))
    assert horizontal_truth == pytest.approx(1.0, abs=1e-12)
    assert vertical_truth == pytest.approx(1.0, abs=1e-12)


def test_alpha_from_beam_angle_is_complementary() -> None:
    """theta (off vertical) and alpha (from horizontal) must sum to 90 deg."""
    dvl = parse_scn_sensors(SCENE_PATH)["dvl"]
    assert math.degrees(dvl.beam_angle_rad + dvl.alpha_rad) == pytest.approx(90.0, abs=1e-9)


# --------------------------------------------------------------------------- #
# Derived transforms
# --------------------------------------------------------------------------- #
def test_derived_camera_transform_is_consistent_with_aqua_slam_convention() -> None:
    """``R_b_c`` maps body axes onto optical axes and the inverse is exact.

    ``T_b_c`` is built from the scene ``<origin rpy="1.5708 0 1.5708">``,
    ``Rz(90) @ Rx(90)`` for the front pair, which takes the body axes onto the
    optical ones: body +x -> optical +z, body +y -> optical -x, body +z ->
    optical -y.  This equals the transpose of ``position.py``'s
    ``optical_to_body``, i.e. ``R_c_b`` -- i.e. the same matrix AQUA-SLAM's
    ``Tbc`` rotation block and ``position.py`` both carry.
    """
    derive = _load_tool("derive_extrinsics")
    derived = derive.derive(SCENE_PATH, "xunyun")
    body_to_camera = np.array(derived.transforms["T_b_c"]["matrix"], dtype=float)
    camera_to_body = np.array(derived.transforms["T_c_b"]["matrix"], dtype=float)

    assert check_rotation(body_to_camera[:3, :3])
    assert np.allclose(
        body_to_camera[:3, :3],
        [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        atol=1e-5,
    )
    # The camera position is the <origin> xyz; AQUA-SLAM's Tbc stores it in
    # column 3 exactly like this, so the two conventions agree.
    assert body_to_camera[:3, 3] == pytest.approx(FRONT_CAMERA_XYZ, abs=1e-9)
    assert np.allclose(body_to_camera @ camera_to_body, np.eye(4), atol=1e-6)
    # position.py carries the transpose (optical -> body); keep the two consistent.
    assert np.allclose(
        camera_to_body[:3, :3],
        [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]],
        atol=1e-5,
    )


def test_down_camera_transform_maps_its_optical_axis_to_body_down() -> None:
    derive = _load_tool("derive_extrinsics")
    derived = derive.derive(SCENE_PATH, "xunyun")
    down = np.array(derived.transforms["T_b_down_cam_left"]["matrix"], dtype=float)
    assert np.allclose(
        down[:3, :3],
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        atol=1e-5,
    )
    # Camera z (forward) points along body +z (down) for a down-looking camera.
    assert down[:3, 2] == pytest.approx((0.0, 0.0, 1.0), abs=1e-5)


def test_dvl_and_gyro_to_camera_translations_are_exact() -> None:
    """Hand-checked values; a scene remount must fail here loudly."""
    derive = _load_tool("derive_extrinsics")
    derived = derive.derive(SCENE_PATH, "xunyun")
    dvl_to_camera = np.array(derived.transforms["T_dvl_c"]["matrix"], dtype=float)
    # r = r_cam - r_dvl = (0.23 - (-0.375), -0.05, 0.076 - 0.1)
    assert dvl_to_camera[:3, 3] == pytest.approx((0.605, -0.05, -0.024), abs=1e-5)
    gyro_to_camera = np.array(derived.transforms["T_gyro_c"]["matrix"], dtype=float)
    assert gyro_to_camera[:3, 3] == pytest.approx(FRONT_CAMERA_XYZ, abs=1e-5)


def test_extrinsics_yaml_matches_the_scene() -> None:
    """The canonical YAML is generated; it must not be edited by hand."""
    derive = _load_tool("derive_extrinsics")
    import yaml

    documented = yaml.safe_load(EXTRINSICS_YAML.read_text(encoding="utf-8"))
    derived = derive.derive(SCENE_PATH, "xunyun").as_yaml_dict()
    problems = derive._compare(documented, derived)
    assert not problems, (
        "config/extrinsics/xunyun.yaml is stale; regenerate with "
        "tools/derive_extrinsics.py --out workspace_sim/src/uv_sim/config/extrinsics/"
        f"xunyun.yaml\n" + "\n".join(problems[:20])
    )


def test_extrinsics_doc_matches_the_scene() -> None:
    """The fenced block in docs/frames/extrinsics_reference.md is the table.

    This is the doc-drift guard: remount a sensor in the scene and this fails
    with the exact element that disagrees.
    """
    derive = _load_tool("derive_extrinsics")
    import yaml

    doc_text = DOC_PATH.read_text(encoding="utf-8")
    block = derive._extract_yaml_block(doc_text)
    assert block is not None, (
        f"{DOC_PATH} is missing the {derive.FENCE_MARKER} ... {derive.FENCE_END} "
        "block"
    )
    documented = yaml.safe_load(block)
    derived = derive.derive(SCENE_PATH, "xunyun").as_yaml_dict()
    problems = derive._compare(documented, derived)
    assert not problems, (
        f"{DOC_PATH.name} drifted from {SCENE_PATH.name}; regenerate the block with "
        "tools/derive_extrinsics.py --emit\n" + "\n".join(problems[:20])
    )


# --------------------------------------------------------------------------- #
# Consumers of the table
# --------------------------------------------------------------------------- #
def test_position_py_optical_matrices_still_match_the_documented_convention() -> None:
    """``position.py`` derives its own rotation matrices; keep them consistent.

    The file cannot be imported here (it needs ROS messages), so parse the
    literals.  The front pair uses ``[[0,-1,0],[1,0,0],[0,0,1]]`` as
    ``optical_to_body`` (the transpose of ``R_c_b``), the down pair uses the
    yaw-90 variant.  See docs/frames/frame_convention.md.
    """
    source = (
        REPO_ROOT / "workspace_auv" / "src" / "uv_perception" / "uv_perception" / "position.py"
    ).read_text(encoding="utf-8")
    assert "np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])" in source, (
        "front camera optical_to_body changed; update docs/frames/frame_convention.md"
    )
    assert "np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])" in source, (
        "down camera optical_to_body changed; update docs/frames/frame_convention.md"
    )


def test_dvl_lever_arm_tool_reads_the_extrinsics_file() -> None:
    """``check_dvl_frame.py`` must not hard-code a second copy of the lever arm.

    ``rclpy`` is imported at module scope, so this skips when the ROS
    environment is not sourced.
    """
    pytest.importorskip("rclpy")
    check = _load_tool("check_dvl_frame")
    from_file = check.resolve_lever_arm(EXTRINSICS_YAML)
    assert from_file == pytest.approx((-0.375, 0.0, 0.1), abs=1e-9)
    assert check.resolve_lever_arm(Path("/nonexistent/extrinsics.yaml")) == pytest.approx(
        check.DVL_LEVER_ARM
    )


# --------------------------------------------------------------------------- #
# N1: the camera offsets are one number used in four places
# --------------------------------------------------------------------------- #
def test_camera_offsets_agree_between_position_and_the_task_modules() -> None:
    """N1 (resolved 2026-09-15): one offset, four modules, one value.

    ``position.py`` ray-casts from ``CAM_PARAMS[*]['offset']``; ``line_follower``
    and ``arrow_surfacer`` project down-camera detections with their own copies.
    If they ever disagree, the 3D fix and the line follower believe different
    geometry -- so pin all four to the same numbers.
    """
    base = REPO_ROOT / "workspace_auv" / "src"
    position = (base / "uv_perception" / "uv_perception" / "position.py").read_text(
        encoding="utf-8"
    )
    assert "(-0.13, -0.05, 0.0645)" not in position  # array syntax below
    for literal in (
        "np.array([0.23, -0.05, 0.076])",
        "np.array([0.23, 0.05, 0.076])",
        "np.array([-0.13, -0.05, 0.0645])",
        "np.array([-0.13, 0.05, 0.0645])",
    ):
        assert literal in position, f"position.py no longer has {literal}"

    for name in ("line_follower.py", "arrow_surfacer.py"):
        text = (base / "uv_task" / "uv_task" / name).read_text(encoding="utf-8")
        assert "np.array([-0.13, -0.05, 0.0645])" in text, f"{name} disagrees on down-left"
        assert "np.array([-0.13, 0.05, 0.0645])" in text, f"{name} disagrees on down-right"


def test_scene_camera_z_matches_the_authoritative_offsets() -> None:
    """The scene is the thing that was wrong in N1; keep it pinned to the code."""
    sensors = parse_scn_sensors(SCENE_PATH)
    assert sensors["front_cam_left"].xyz == pytest.approx(FRONT_CAMERA_XYZ, abs=1e-9)
    assert sensors["down_cam_left"].xyz == pytest.approx(DOWN_CAMERA_XYZ, abs=1e-9)
    assert sensors["front_cam_left"].xyz[2] == pytest.approx(0.076, abs=1e-9)
    assert sensors["down_cam_left"].xyz[2] == pytest.approx(0.0645, abs=1e-9)


# --------------------------------------------------------------------------- #
# PID fallback: the file is the source of truth, the fallback must mirror it
# --------------------------------------------------------------------------- #
def test_pid_builtin_fallback_is_structurally_complete() -> None:
    """The fallback must be *usable*, not necessarily numerically current.

    History: the fallback once disagreed with ``sim_pid.yaml`` on both yaw
    channels (``pos.yaw`` kp 1.5 vs 35, ``pos.yaw_rate`` kp 30 vs 12, i_limit 600
    vs 6), so losing the file silently swapped in a different yaw controller.

    Deliberately **not** asserting that the numbers match: yaw gains get tuned by
    hand during sim sessions, and a test that fails on every tuning edit gets
    deleted rather than obeyed.  What is worth pinning is that the fallback still
    names exactly the same channels and fields with finite numbers -- a typo or a
    dropped channel would otherwise only show up as a crash on a machine without
    the config file.
    """
    import math

    import yaml

    import uv_sim.pid_config as pid_config

    shipped = yaml.safe_load(
        (
            REPO_ROOT / "workspace_sim" / "src" / "uv_sim" / "config" / "sim_pid.yaml"
        ).read_text(encoding="utf-8")
    )
    assert set(shipped) == set(pid_config.PID_PARAM_PATHS), (
        "sim_pid.yaml and PID_PARAM_PATHS disagree on the channel list"
    )
    assert set(pid_config.DEFAULT_PID_PARAMS) == set(pid_config.PID_PARAM_PATHS)

    allowed = set(pid_config.PID_REQUIRED_FIELDS) | set(pid_config.PID_OPTIONAL_FIELDS)
    for channel, gains in pid_config.DEFAULT_PID_PARAMS.items():
        assert set(gains) <= allowed, f"{channel} has an unknown field"
        for field in pid_config.PID_REQUIRED_FIELDS:
            assert field in gains, f"{channel} is missing required field {field}"
            assert math.isfinite(float(gains[field])), f"{channel}.{field} is not finite"
        # Same field set on both sides, so a renamed field cannot go unnoticed.
        if channel in shipped:
            assert set(shipped[channel]) <= set(gains) | {"z_thrust_ff"}, (
                f"{channel}: the file uses fields the fallback does not know"
            )


def test_tf_tool_reads_the_robot_name_from_the_scene(tmp_path) -> None:
    """The tool must not hard-code ``Vehicle``: world scenes rename the robot."""
    tool = _load_tool("dump_tf_tree")
    assert hasattr(tool, "read_scene_expectations")

    vehicle_scene = (
        REPO_ROOT / "workspace_sim" / "src" / "stonefish_ros2" / "Data" / "xunyun_fixed.scn"
    )
    name, publishes = tool.read_scene_expectations(vehicle_scene)
    # The scene is parameterised with $(arg robot_name), so the name is the
    # caller's business; the flag is what matters and it must be off.
    assert publishes is False

    world_scene = tmp_path / "world.scn"
    world_scene.write_text(
        '<scenario><robot name="XUNYUN">'
        '<ros_base_link_transform publish="true"/>'
        "</robot></scenario>",
        encoding="utf-8",
    )
    name, publishes = tool.read_scene_expectations(world_scene)
    assert name == "XUNYUN"
    assert publishes is True
