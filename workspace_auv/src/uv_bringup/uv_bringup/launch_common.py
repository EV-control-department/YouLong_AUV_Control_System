"""Orchestration helpers shared by the system-level launch files.

**Every node is declared by the package that owns it** (``uv_control/launch``,
``uv_hm/launch``, ...), and a system preset such as ``real_bringup.py`` only
*orchestrates* those entry points.  These two helpers are the whole orchestration
vocabulary, so adding a node never means editing a bringup file -- the bringup
files stay as short as the list of subsystems they start.

``uv_sim_bringup`` (workspace_sim) imports this module too: the simulation preset
builds on the same vocabulary, which is the correct dependency direction
(overlay -> underlay).
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def arg(name: str, default_value: str, description: str) -> DeclareLaunchArgument:
    """Declare a launch argument (thin wrapper, kept for symmetry with include)."""
    return DeclareLaunchArgument(
        name, default_value=default_value, description=description
    )


def include(
    package: str,
    filename: str,
    arguments: dict | None = None,
    when: str | None = None,
) -> IncludeLaunchDescription:
    """Include ``<package>/launch/<filename>``, optionally gated by a flag.

    ``when`` is the *name of a boolean launch argument*; the whole include is
    skipped when it is false.  The gate deliberately lives here, in the preset,
    rather than as an extra ``enable_*`` argument inside the included file: a
    node's "should I run" decision must have exactly one author, otherwise the
    preset and the package launch can disagree about whether a node starts.

    Raises ``PackageNotFoundError`` when the package is not on the ament prefix
    path -- that is intentionally loud, because a silently skipped include would
    look like a working (but incomplete) stack.
    """
    source = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package), 'launch', filename)
    )
    return IncludeLaunchDescription(
        source,
        launch_arguments=(arguments or {}).items(),
        condition=IfCondition(LaunchConfiguration(when)) if when else None,
    )
