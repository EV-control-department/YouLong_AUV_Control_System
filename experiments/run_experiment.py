"""Create a reproducible experiment directory and optionally run a command."""

from __future__ import annotations

import argparse
import datetime as dt
import json
from pathlib import Path
import shlex
import subprocess


def create_run(
    output_root: Path,
    *,
    name: str,
    seed: int,
    config_text: str = '',
    metadata: dict | None = None,
) -> Path:
    timestamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output_root.mkdir(parents=True, exist_ok=True)
    run = output_root / f'{timestamp}_{name}'
    try:
        run.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        # Keep parallel/repeated invocations recoverable even on filesystems
        # whose timestamp resolution is coarser than this process clock.
        suffix = 1
        while True:
            candidate = output_root / f'{timestamp}_{name}_{suffix}'
            try:
                candidate.mkdir(parents=True, exist_ok=False)
                run = candidate
                break
            except FileExistsError:
                suffix += 1
    for directory in ('rosbag', 'log'):
        (run / directory).mkdir()
    (run / 'trajectory.csv').write_text(
        'stamp,estimate_x,estimate_y,estimate_z,estimate_yaw_rad,'
        'truth_x,truth_y,truth_z,truth_yaw_rad\n', encoding='utf-8')
    (run / 'metrics.json').write_text('{}\n', encoding='utf-8')
    (run / 'config.yaml').write_text(
        config_text or f'name: {name}\nseed: {seed}\n', encoding='utf-8')
    run_metadata = {
        'name': name, 'seed': seed, 'run_id': run.name,
        'created_utc': dt.datetime.now(dt.timezone.utc).isoformat(),
    }
    if metadata:
        run_metadata.update(metadata)
    (run / 'metadata.json').write_text(
        json.dumps(run_metadata, indent=2) + '\n', encoding='utf-8')
    return run


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', default='nominal')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--output-root', default='results')
    parser.add_argument('--config-file')
    parser.add_argument('--command', help='Optional shell-like command to execute')
    parser.add_argument(
        '--ros-launch', action='store_true',
        help='Run uv_sim_bringup experiment.launch.py in the new directory')
    parser.add_argument('--world', default='girona500auv_console.scn')
    parser.add_argument('--degradation', default='nominal')
    parser.add_argument('--estimator', default='bootstrap')
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--record-bag', dest='record_bag', action='store_true')
    parser.add_argument(
        '--no-record-bag', dest='record_bag', action='store_false')
    parser.set_defaults(record_bag=True)
    args = parser.parse_args(argv)
    if args.command and args.ros_launch:
        parser.error('--command and --ros-launch are mutually exclusive')
    config_text = ''
    if args.config_file:
        config_text = Path(args.config_file).read_text(encoding='utf-8')
    run = create_run(
        Path(args.output_root), name=args.name, seed=args.seed,
        config_text=config_text or (
            f'name: {args.name}\nseed: {args.seed}\n'
            f'world: {args.world}\ndegradation: {args.degradation}\n'
            f'estimator: {args.estimator}\nduration: {args.duration}\n'),
        metadata={
            'world': args.world,
            'degradation': args.degradation,
            'estimator': args.estimator,
            'duration_s': args.duration,
            'record_bag': args.record_bag,
        },
    )
    command = args.command
    if args.ros_launch:
        command_parts = [
            'ros2', 'launch', 'uv_sim_bringup', 'experiment.launch.py',
            f'world:={args.world}', f'seed:={args.seed}',
            f'degradation:={args.degradation}',
            f'estimator:={args.estimator}', f'duration:={args.duration}',
            f'record_bag:={str(args.record_bag).lower()}',
            f'results_dir:={run}', f'run_id:={run.name}',
        ]
        command = shlex.join(command_parts)
    if command:
        log_path = run / 'log' / 'experiment.log'
        with log_path.open('w', encoding='utf-8') as stream:
            stream.write(f'$ {command}\n')
            stream.flush()
            completed = subprocess.run(
                shlex.split(command), cwd=run.parent,
                env=None, stdout=stream, stderr=subprocess.STDOUT,
                check=False)
        if completed.returncode:
            print(run)
            raise SystemExit(completed.returncode)
    print(run)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
