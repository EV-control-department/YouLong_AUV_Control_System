"""Runtime preparation of isolated Stonefish scene directories."""

from __future__ import annotations

from pathlib import Path
import shutil
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


DEFAULT_GENERATED_SCENARIO = (
    'worlds/guoshui_2026/guoshui_2026_cruise_seeded.scn'
)

LEGACY_SCENARIO_ALIASES = {
    'underwater_xunyun.scn': 'underwater_xunyun.scn',
    'underwater_test.scn': 'underwater_test.scn',
    'wuurc_murc_2026_auv.scn': 'wuurc_murc_2026_auv.scn',
}

SCENARIO_ALIASES = {
    'guoshui_2026_cruise.scn': 'worlds/guoshui_2026/guoshui_2026_cruise.scn',
    'guoshui_2026_cruise_seeded.scn': (
        'worlds/guoshui_2026/guoshui_2026_cruise_seeded.scn'
    ),
    'console_test.scn': 'worlds/examples/console_test.scn',
    'simple.scn': 'worlds/examples/simple.scn',
    'girona500auv_console.scn': 'worlds/examples/girona500auv_console.scn',
    'girona500auv_full.scn': 'worlds/examples/girona500auv_full.scn',
    'girona500auv_full copy.scn': 'worlds/examples/girona500auv_full copy.scn',
    'sauvc_2026_finals.scn': 'worlds/sauvc_2026/sauvc_2026_finals.scn',
    'sauvc_2026_finals_with girona.scn': (
        'worlds/sauvc_2026/sauvc_2026_finals.scn'
    ),
    'sauvc_2026_qualification.scn': (
        'worlds/sauvc_2026/sauvc_2026_qualification.scn'
    ),
    'sauvc_pool.scn': 'worlds/sauvc_2026/sauvc_pool.scn',
}


def find_sim_assets_dir(launch_file):
    """Locate the installed/source ``uv_sim_assets`` package."""
    try:
        assets_share = Path(
            get_package_share_directory('uv_sim_assets')).resolve()
    except Exception:  # source-tree tests may run before colcon install
        assets_share = None
    roots = (Path(launch_file).resolve(), assets_share) if assets_share else (
        Path(launch_file).resolve(),)
    candidates = [assets_share] if assets_share else []
    for root in roots:
        for base in (root, *root.parents):
            candidates.extend([
                base / 'workspace_sim' / 'src' / 'uv_sim_assets',
                base / 'src' / 'uv_sim_assets',
            ])
    for candidate in dict.fromkeys(candidates):
        if (candidate / 'vehicles').is_dir() and (candidate / 'worlds').is_dir():
            return candidate
    raise RuntimeError('cannot locate uv_sim_assets package directory')


def prepare_scene(
    *,
    scenario_desc,
    scene_seed,
    launch_file,
    start_actions,
):
    """Prepare an isolated scene and then start the simulator actions."""

    def _prepare(context):
        source_data = find_sim_assets_dir(launch_file)
        scenario_value = scenario_desc.perform(context).strip()
        seed_text = scene_seed.perform(context).strip()
        try:
            seed = int(seed_text)
        except ValueError as error:
            raise RuntimeError(
                f'scene_seed must be an integer, got {seed_text!r}') from error

        scenario_path = Path(scenario_value).expanduser()
        if not scenario_path.is_absolute():
            legacy_name = LEGACY_SCENARIO_ALIASES.get(scenario_value)
            canonical_name = SCENARIO_ALIASES.get(scenario_value)
            if legacy_name:
                scenario_path = source_data / 'legacy_data' / legacy_name
            elif canonical_name:
                scenario_path = source_data / canonical_name
            else:
                scenario_path = source_data / scenario_path

        simulation_data = source_data
        if scenario_path.parent == source_data / 'legacy_data':
            # The legacy fixtures retain their original bare resource names.
            # Point Stonefish at that directory only for the compatibility
            # alias; maintained worlds always use the package root.
            simulation_data = source_data / 'legacy_data'

        should_generate = (
            scenario_path == source_data / DEFAULT_GENERATED_SCENARIO
        )

        if not should_generate:
            if not scenario_path.is_file():
                raise RuntimeError(
                    f'Stonefish scenario does not exist: {scenario_path}')
            context.launch_configurations['resolved_simulation_data'] = str(
                simulation_data)
            context.launch_configurations['resolved_scenario'] = str(
                scenario_path)
            return [
                LogInfo(msg=['Stonefish assets: ', str(simulation_data)]),
                LogInfo(msg=['Stonefish scenario: ', str(scenario_path)]),
                *start_actions,
            ]

        run_root = Path(tempfile.mkdtemp(prefix='uv_sim_scene_'))
        run_data = run_root / 'assets'
        shutil.copytree(source_data, run_data, symlinks=True)
        generator = run_data / 'tools' / 'generate_guoshui_2026_scene.py'
        template = run_data / 'worlds' / 'guoshui_2026' / 'guoshui_2026_cruise.scn'
        output = run_data / DEFAULT_GENERATED_SCENARIO
        context.launch_configurations['resolved_simulation_data'] = str(run_data)
        context.launch_configurations['resolved_scenario'] = str(output)

        generator_process = ExecuteProcess(
            cmd=[
                sys.executable, str(generator), '--seed', str(seed),
                '--template', str(template), '--output', str(output),
            ],
            name='uv_scene_prepare',
            output='both',
        )

        def _after_generation(event, callback_context):
            if callback_context.is_shutdown:
                return []
            if event.returncode != 0:
                return [
                    LogInfo(msg=[
                        'Scene generation failed; Stonefish not started: ',
                        str(output),
                    ]),
                    EmitEvent(event=Shutdown(
                        reason='Stonefish scene preparation failed')),
                ]
            return [
                LogInfo(msg=[
                    'Generated isolated Stonefish scene with seed ',
                    str(seed), ': ', str(output),
                ]),
                *start_actions,
            ]

        return [
            LogInfo(msg=[
                'Preparing isolated Stonefish assets directory: ', str(run_data),
            ]),
            generator_process,
            RegisterEventHandler(OnProcessExit(
                target_action=generator_process,
                on_exit=_after_generation,
            )),
        ]

    return OpaqueFunction(function=_prepare)
