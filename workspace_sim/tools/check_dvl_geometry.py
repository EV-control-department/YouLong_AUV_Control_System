#!/usr/bin/env python3
"""T2 check: DVL beam geometry is numerically sound and matches the scene.

Runs **without ROS** -- it only imports ``uv_sim.geometry``::

    python3 workspace_sim/tools/check_dvl_geometry.py
    python3 workspace_sim/tools/check_dvl_geometry.py --scn <other.scn>

What it verifies
----------------
1. ``(E^T E)^-1 E^T (E v) == v`` to machine precision for the configured
   ``alpha``/``beta`` (the inversion AQUA-SLAM uses at
   ``DVLGroPreIntegration.cpp:552``).
2. The scene's ``<specs beam_angle>`` (off **vertical**) converts to
   ``alpha`` (elevation from **horizontal**) as ``alpha = 90 - theta``, and
   ``alpha`` equals the 60 deg the real XUNYUN head has.
3. ``beam_positive_z="true"`` -- the pre-T1 scene had it defaulting to false,
   which published velocity in a frame rotated 45 deg with y/z flipped.
4. Sensitivity table: what a wrong ``alpha`` costs.  The AQUA-SLAM shipped
   default ``alpha = 67.5 deg`` scales horizontal velocity by +30.7%.

Exit codes: 0 pass, 1 assertion failure, 2 environment/scene problem.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "workspace_sim" / "src" / "uv_sim"))

from uv_sim.geometry import (  # noqa: E402
    DEFAULT_BEAM_AZIMUTHS_DEG,
    DEFAULT_SCENE,
    XUNYUN_BEAM_ANGLE_DEG,
    beam_inversion_sensitivity,
    beam_velocity_to_body,
    dvl_beam_azimuths_from_scene,
    dvl_beam_directions,
    parse_scn_sensors,
)

#: The real head tilts its beams 30 deg off vertical; AQUA-SLAM calls that
#: elevation alpha = 60 deg.
EXPECTED_BEAM_ANGLE_DEG = XUNYUN_BEAM_ANGLE_DEG
EXPECTED_ALPHA_DEG = 90.0 - EXPECTED_BEAM_ANGLE_DEG
RECONSTRUCTION_TOLERANCE = 1e-12
ALPHA_TOLERANCE_DEG = 0.05

#: alpha values to tabulate, i.e. the plausible mistakes and the truth.
SENSITIVITY_ALPHA_DEG = (30.0, 45.0, 55.0, 60.0, 65.0, 67.5, 70.0)


def _fail(message: str) -> int:
    print(f"FAIL  {message}")
    return 1


def check_reconstruction(alpha_rad: float, beta_deg) -> float:
    """Worst-case reconstruction error over a set of probe velocities."""
    beams = dvl_beam_directions(alpha_rad, beta_deg)
    worst = 0.0
    probes = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (0.3, -0.2, 0.05),
        (-1.2, 0.9, -0.4),
    )
    for probe in probes:
        truth = np.array(probe, dtype=float)
        recovered = beam_velocity_to_body(beams, beams @ truth)
        worst = max(worst, float(np.max(np.abs(recovered - truth))))
    return worst


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--scn",
        default=str(REPO_ROOT / DEFAULT_SCENE),
        help="scene file to read the DVL block from",
    )
    args = parser.parse_args(argv)

    scn_path = Path(args.scn)
    if not scn_path.is_file():
        print(f"FAIL  scene not found: {scn_path}")
        return 2

    sensors = parse_scn_sensors(scn_path)
    if "dvl" not in sensors:
        print(f"FAIL  no <sensor type=\"dvl\"> in {scn_path}")
        return 2
    dvl = sensors["dvl"]

    status = 0
    print(f"scene : {scn_path}")
    print(f"DVL   : name={dvl.name!r} frame_id={dvl.name!r} link={dvl.link!r}")
    print(f"        xyz={dvl.xyz} rpy={dvl.rpy} rate={dvl.rate_hz} Hz")
    print(f"        topic={dvl.ros_topics.get('topic')}")

    # -- 1. angles ---------------------------------------------------------- #
    beam_angle = dvl.beam_angle_rad
    if beam_angle is None:
        return _fail("<specs beam_angle> missing; cannot check the beam geometry")
    beam_angle_deg = math.degrees(beam_angle)
    alpha_deg = math.degrees(dvl.alpha_rad)
    print("\n[1] angle conventions")
    print(f"    theta (scene, off vertical)   = {beam_angle_deg:8.4f} deg")
    print(f"    alpha (SLAM, from horizontal) = {alpha_deg:8.4f} deg  = 90 - theta")
    if abs(beam_angle_deg - EXPECTED_BEAM_ANGLE_DEG) > ALPHA_TOLERANCE_DEG:
        status |= _fail(
            f"theta={beam_angle_deg:.4f} deg, expected {EXPECTED_BEAM_ANGLE_DEG} deg "
            "(real head is 30 deg off vertical)"
        )
    if abs(alpha_deg - EXPECTED_ALPHA_DEG) > ALPHA_TOLERANCE_DEG:
        status |= _fail(
            f"alpha={alpha_deg:.4f} deg, expected {EXPECTED_ALPHA_DEG} deg; "
            "AQUA-SLAM's settings yaml ships 67.5 deg, which must be overridden"
        )
    else:
        print(f"    OK  alpha == {EXPECTED_ALPHA_DEG} deg (AQUA-SLAM mE expects this)")

    # -- 2. beam_positive_z ------------------------------------------------- #
    positive_z = dvl.specs.get("beam_positive_z", "false").strip().lower()
    print("\n[2] beam direction sanity")
    if positive_z != "true":
        status |= _fail(
            f'beam_positive_z={positive_z!r}; must be "true" or velocity is published '
            "rotated 45 deg with y/z flipped (the pre-T1 defect)"
        )
    else:
        print('    OK  beam_positive_z="true" (beams point along sensor +z = down)')
    print(f"    specs={dvl.specs}")

    # -- 3. reconstruction -------------------------------------------------- #
    beta_deg = dvl_beam_azimuths_from_scene(dvl)
    beams = dvl_beam_directions(dvl.alpha_rad, beta_deg)
    gram = beams.T @ beams
    print("\n[3] beam matrix and least-squares inversion")
    print(f"    beta (azimuths) = {tuple(round(b, 1) for b in beta_deg)} deg")
    for index, row in enumerate(beams):
        print(
            f"    E[{index}] = ({row[0]:+.4f}, {row[1]:+.4f}, {row[2]:+.4f})"
            f"   |E[{index}]| = {np.linalg.norm(row):.6f}"
        )
    print(f"    E^T E diag      = {np.round(np.diag(gram), 6).tolist()}")
    print(f"    det(E^T E)      = {np.linalg.det(gram):.6f}  (rank 3 => invertible)")
    if abs(np.linalg.det(gram)) < 1e-9:
        status |= _fail("E is rank-deficient; the beam set cannot resolve 3D velocity")
    error = check_reconstruction(dvl.alpha_rad, beta_deg)
    print(f"    max |(E^T E)^-1 E^T (E v) - v| = {error:.3e}")
    if error > RECONSTRUCTION_TOLERANCE:
        status |= _fail(f"reconstruction error {error:.3e} > {RECONSTRUCTION_TOLERANCE:.0e}")
    else:
        print("    OK  inversion is exact for the configured alpha/beta")

    # -- 4. sensitivity ----------------------------------------------------- #
    print("\n[4] alpha sensitivity (what a wrong beam angle costs)")
    print("    alpha[deg]  horizontal scale   error     vertical scale   error")
    for alpha in SENSITIVITY_ALPHA_DEG:
        horizontal, vertical = beam_inversion_sensitivity(math.radians(alpha))
        mark = "  <- truth" if abs(alpha - EXPECTED_ALPHA_DEG) < 1e-9 else ""
        if abs(alpha - 67.5) < 1e-9:
            mark = "  <- AQUA-SLAM shipped default"
        print(
            f"    {alpha:9.1f}  {horizontal:14.4f}  {100 * (horizontal - 1):+7.2f}%"
            f"  {vertical:14.4f}  {100 * (vertical - 1):+7.2f}%{mark}"
        )
    horizontal_67, _ = beam_inversion_sensitivity(math.radians(67.5))
    if abs(horizontal_67 - 1.3066) > 1e-3:
        status |= _fail(
            f"alpha=67.5 deg horizontal scale is {horizontal_67:.4f}, expected 1.3066"
        )
    else:
        print("    OK  alpha=67.5 deg reproduces the documented +30.7% scale error")

    print()
    if status == 0:
        print("PASS  DVL beam geometry is self-consistent and matches the real head")
    else:
        print("FAIL  see the messages above")
    return status


if __name__ == "__main__":
    sys.exit(main())
