"""Run one reproducible SIL experiment with recording and evaluation."""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


DEGRADATION_PROFILES = {
    'nominal': {
        'enable_degradation': 'false',
        'dvl_dropout_probability': '0.0',
        'visual_dropout_probability': '0.0',
    },
    'dvl_loss': {
        'enable_degradation': 'true',
        'dvl_dropout_probability': '0.5',
        'visual_dropout_probability': '0.0',
    },
    'visual_loss': {
        'enable_degradation': 'true',
        'dvl_dropout_probability': '0.0',
        'visual_dropout_probability': '0.5',
    },
    'dvl_visual': {
        'enable_degradation': 'true',
        'dvl_dropout_probability': '0.5',
        'visual_dropout_probability': '0.5',
    },
}


def _sim_include(arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('uv_sim_bringup'), 'launch', 'sim.launch.py',
        ])),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    world = LaunchConfiguration('world')
    seed = LaunchConfiguration('seed')
    degradation = LaunchConfiguration('degradation')
    estimator = LaunchConfiguration('estimator')
    results_dir = LaunchConfiguration('results_dir')
    run_id = LaunchConfiguration('run_id')
    duration = LaunchConfiguration('duration')
    record_bag = LaunchConfiguration('record_bag')
    gpu = LaunchConfiguration('gpu')

    def _actions(context):
        mode = degradation.perform(context).strip().lower()
        if mode not in DEGRADATION_PROFILES:
            raise RuntimeError(
                f'unknown degradation profile {mode!r}; choose one of '
                f'{", ".join(DEGRADATION_PROFILES)}')
        profile = DEGRADATION_PROFILES[mode]
        results_path = Path(results_dir.perform(context)).expanduser()
        results_path.mkdir(parents=True, exist_ok=True)
        sim_arguments = {
            'scenario_desc': world,
            'scene_seed': seed,
            'degradation_seed': seed,
            'gpu': gpu,
            'enable_ai': 'false',
            'enable_motion': 'true',
            'enable_nav': 'false',
            'enable_task': 'false',
            'enable_preview': 'false',
            'stream_annotated': 'false',
            'record_session': 'false',
            'enable_evaluation': 'true',
            'estimator': estimator,
            'evaluation_output_dir': results_dir,
            'evaluation_run_id': run_id,
            'enable_degradation': profile['enable_degradation'],
            'dvl_dropout_probability': profile['dvl_dropout_probability'],
            'visual_dropout_probability': profile[
                'visual_dropout_probability'],
        }
        # ``create_run`` pre-creates the required ``rosbag/`` directory.  The
        # rosbag2 writer expects its ``-o`` target not to exist, so write the
        # bag into a fresh child directory instead of passing the pre-created
        # layout directory itself.
        bag_path = PathJoinSubstitution([results_dir, 'rosbag', 'data'])
        topics = [
            '/auv/state/odom',
            '/auv/state/health',
            '/auv/sensors/imu/data',
            '/auv/sensors/dvl/velocity',
            '/auv/sensors/dvl/altitude',
            '/auv/sensors/pressure',
            '/auv/sensors/usbl/measurement',
            '/auv/sim/ground_truth/odom',
            '/auv/perception/observations',
            '/auv/evaluation/metrics',
            '/auv/evaluation/events',
            '/auv/sim/degradation/events',
            '/auv/sim/performance',
            '/auv/sim/control_performance',
        ]
        bag = ExecuteProcess(
            cmd=[
                FindExecutable(name='ros2'), 'bag', 'record', '-o', bag_path,
                *topics,
            ],
            name='experiment_rosbag',
            output='log',
            condition=IfCondition(record_bag),
        )
        stop = TimerAction(
            period=duration,
            actions=[EmitEvent(event=Shutdown(
                reason='experiment duration reached'))],
        )
        return [
            LogInfo(msg=[
                'Experiment estimator=', estimator,
                ' degradation=', degradation,
                ' seed=', seed,
            ]),
            _sim_include(sim_arguments),
            bag,
            stop,
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=(
                'worlds/guoshui_2026/guoshui_2026_cruise_seeded.scn')),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('degradation', default_value='nominal'),
        DeclareLaunchArgument('estimator', default_value='bootstrap'),
        DeclareLaunchArgument('results_dir', default_value='results/current'),
        DeclareLaunchArgument('run_id', default_value=''),
        DeclareLaunchArgument('duration', default_value='60.0'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('gpu', default_value='false'),
        OpaqueFunction(function=_actions),
    ])
