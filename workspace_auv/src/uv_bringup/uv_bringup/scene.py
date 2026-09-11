"""Runtime preparation of isolated Stonefish scene directories."""

from __future__ import annotations

import shutil
import sys
import tempfile
from pathlib import Path

from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


DEFAULT_GENERATED_SCENARIO = "guoshui_2026_cruise_seeded.scn"


def find_stonefish_data_dir(launch_file):
    """Locate the source Data directory without assuming a user home path."""
    from ament_index_python.packages import get_package_share_directory

    stonefish_share = Path(get_package_share_directory("stonefish_ros2")).resolve()
    roots = (Path(launch_file).resolve(), stonefish_share)
    candidates = []
    for root in roots:
        for base in (root, *root.parents):
            candidates.extend([
                base / "workspace_sim" / "src" / "stonefish_ros2" / "Data",
                base / "src" / "stonefish_ros2" / "Data",
            ])
    for candidate in dict.fromkeys(candidates):
        if candidate.is_dir() and any(candidate.glob("*.scn")):
            return candidate
    raise RuntimeError("无法定位 stonefish_ros2 的 Data 源码目录")


def prepare_scene(
    *,
    scenario_desc,
    scene_seed,
    launch_file,
    start_actions,
):
    """Return an action that prepares a scene and then starts ``start_actions``.

    The generated Guoshui scene is built in an isolated copy of the Stonefish
    Data tree.  This prevents the launch process from overwriting the tracked
    source tree and allows multiple seeded simulations to run concurrently.
    """

    def _prepare(context):
        source_data = find_stonefish_data_dir(launch_file)
        scenario_value = scenario_desc.perform(context).strip()
        seed_text = scene_seed.perform(context).strip()
        try:
            seed = int(seed_text)
        except ValueError as error:
            raise RuntimeError(
                f"scene_seed must be an integer, got {seed_text!r}") from error

        scenario_path = Path(scenario_value).expanduser()
        if not scenario_path.is_absolute():
            scenario_path = source_data / scenario_path

        # Preserve the current default: the Guoshui seeded file is generated
        # for every seed, including seed 0.  Other explicitly selected scenes
        # are treated as immutable existing files.
        should_generate = (
            Path(scenario_value).name == DEFAULT_GENERATED_SCENARIO
            and scenario_path.parent == source_data
        )

        if not should_generate:
            if not scenario_path.is_file():
                raise RuntimeError(f"Stonefish scenario does not exist: {scenario_path}")
            context.launch_configurations["resolved_simulation_data"] = str(
                source_data)
            context.launch_configurations["resolved_scenario"] = str(
                scenario_path)
            return [
                LogInfo(msg=["Stonefish Data: ", str(source_data)]),
                LogInfo(msg=["Stonefish scenario: ", str(scenario_path)]),
                *start_actions,
            ]

        run_root = Path(tempfile.mkdtemp(prefix="uv_bringup_scene_"))
        run_data = run_root / "Data"
        shutil.copytree(source_data, run_data, symlinks=True)
        generator = run_data / "generate_guoshui_2026_scene.py"
        template = run_data / "guoshui_2026_cruise.scn"
        output = run_data / DEFAULT_GENERATED_SCENARIO
        context.launch_configurations["resolved_simulation_data"] = str(run_data)
        context.launch_configurations["resolved_scenario"] = str(output)

        generator_process = ExecuteProcess(
            cmd=[
                sys.executable, str(generator),
                "--seed", str(seed),
                "--template", str(template),
                "--output", str(output),
            ],
            name="uv_scene_prepare",
            output="both",
        )

        def _after_generation(event, callback_context):
            if callback_context.is_shutdown:
                return []
            if event.returncode != 0:
                return [
                    LogInfo(msg=["Scene generation failed; Stonefish not started: ", str(output)]),
                    EmitEvent(event=Shutdown(reason="Stonefish scene preparation failed")),
                ]
            return [
                LogInfo(msg=["Generated isolated Stonefish scene with seed ", str(seed), ": ", str(output)]),
                *start_actions,
            ]

        return [
            LogInfo(msg=["Preparing isolated Stonefish Data directory: ", str(run_data)]),
            generator_process,
            RegisterEventHandler(OnProcessExit(
                target_action=generator_process,
                on_exit=_after_generation,
            )),
        ]

    return OpaqueFunction(function=_prepare)
