from pathlib import Path

from run_experiment import create_run, main


def test_create_run_writes_standard_result_layout(tmp_path: Path):
    run = create_run(tmp_path, name='dvl_loss', seed=42)
    assert (run / 'config.yaml').is_file()
    assert (run / 'metadata.json').is_file()
    assert (run / 'rosbag').is_dir()
    assert (run / 'trajectory.csv').is_file()
    assert (run / 'metrics.json').is_file()
    assert (run / 'log').is_dir()
    assert (run / 'trajectory.csv').read_text(encoding='utf-8').splitlines()[0] == (
        'stamp,estimate_x,estimate_y,estimate_z,estimate_yaw_rad,'
        'truth_x,truth_y,truth_z,truth_yaw_rad')


def test_ros_launch_command_targets_the_created_run(tmp_path, monkeypatch):
    calls = []

    class Completed:
        returncode = 0

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return Completed()

    monkeypatch.setattr('run_experiment.subprocess.run', fake_run)
    assert main([
        '--name', 'nominal', '--seed', '42', '--output-root', str(tmp_path),
        '--ros-launch', '--duration', '3', '--no-record-bag',
    ]) == 0

    run = next(tmp_path.iterdir())
    command, kwargs = calls[0]
    assert f'results_dir:={run}' in command
    assert f'run_id:={run.name}' in command
    assert kwargs['cwd'] == run.parent
    assert (run / 'log' / 'experiment.log').read_text(
        encoding='utf-8').startswith('$ ros2 launch')
