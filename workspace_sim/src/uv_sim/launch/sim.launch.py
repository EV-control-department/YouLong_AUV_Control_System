"""Public YouLong simulation entry point.

``uv_sim_bringup`` remains the implementation launch file.  This wrapper owns
the stable world/vehicle interface and rejects ambiguous legacy invocation.
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from uv_sim.scenarios import resolve_world
from uv_sim.scenarios import PROFILE_ALIASES
from uv_sim_bringup.launch_common import (
    declare_feature_arguments,
    declare_mission_file,
    declare_observability_arguments,
    declare_simulation_arguments,
)


def generate_launch_description():
    world = LaunchConfiguration("world")
    vehicle = LaunchConfiguration("vehicle")
    profile = LaunchConfiguration("profile")
    scenario_desc = LaunchConfiguration("scenario_desc")

    def _include(context):
        world_value = world.perform(context).strip()
        legacy_value = scenario_desc.perform(context).strip()
        vehicle_value = vehicle.perform(context).strip()
        profile_value = profile.perform(context).strip()
        try:
            child_profile, profile_world = PROFILE_ALIASES[profile_value]
        except KeyError as error:
            raise RuntimeError(
                f"unsupported profile {profile_value!r}; choose one of "
                f"{', '.join(PROFILE_ALIASES)}") from error
        if vehicle_value != "youlong":
            raise RuntimeError(
                f"unsupported vehicle {vehicle_value!r}; only 'youlong' is "
                "available in the canonical asset package")
        if world_value and legacy_value:
            raise RuntimeError(
                "world:= and scenario_desc:= cannot be used together; "
                "use world:= for the canonical interface")

        if world_value:
            assets_root = Path(
                get_package_share_directory("uv_sim_assets"))
            scenario = resolve_world(assets_root, world_value)
        elif legacy_value:
            scenario = legacy_value
        elif profile_world:
            assets_root = Path(get_package_share_directory("uv_sim_assets"))
            scenario = resolve_world(assets_root, profile_world)
        else:
            scenario = (
                "worlds/guoshui_2026/guoshui_2026_cruise_seeded.scn")

        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("uv_sim_bringup"), "launch", "sim.launch.py",
            ])),
            launch_arguments={
                "profile": child_profile,
                "scenario_desc": str(scenario),
            }.items(),
        )]

    return LaunchDescription([
        DeclareLaunchArgument(
            "profile", default_value="sim_dev",
            choices=list(PROFILE_ALIASES),
            description="Simulator profile or competition world preset",
        ),
        declare_mission_file(),
        *declare_feature_arguments(),
        *declare_observability_arguments(),
        *declare_simulation_arguments(
            scenario_default="",
            window_width_default="960",
            window_height_default="540",
            render_quality_default="low",
            camera_stitch_fps_default="5.0",
        ),
        # Public scene selection.  ``scenario_desc`` is retained above as a
        # deprecated compatibility argument for old scripts.
        DeclareLaunchArgument(
            "world", default_value="",
            description="World name, for example guoshui_2026/cruise_seeded"),
        DeclareLaunchArgument(
            "vehicle", default_value="youlong",
            description="Canonical vehicle name"),
        OpaqueFunction(function=_include),
    ])
