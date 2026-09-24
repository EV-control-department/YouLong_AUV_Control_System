"""Launch go2rtc; each requested stream starts its own camera_streamer exec."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _default_go2rtc():
    here = Path(__file__).resolve()
    for parent in (here.parent, *here.parents):
        candidate = parent / 'third_party' / 'go2rtc' / 'go2rtc'
        if candidate.is_file():
            return str(candidate)
    return 'go2rtc'


def generate_launch_description():
    share = Path(get_package_share_directory('uv_stream'))
    config = str(share / 'config' / 'go2rtc.yaml')
    package_prefix = share.parents[1]
    streamer_bin = str(package_prefix / 'lib' / 'uv_stream')
    return LaunchDescription([
        DeclareLaunchArgument('go2rtc_executable', default_value=_default_go2rtc()),
        SetEnvironmentVariable(
            'PATH', [streamer_bin, ':', EnvironmentVariable('PATH')]),
        ExecuteProcess(
            cmd=[LaunchConfiguration('go2rtc_executable'), '-config', config],
            output='both', respawn=True, respawn_delay=1.0),
    ])
