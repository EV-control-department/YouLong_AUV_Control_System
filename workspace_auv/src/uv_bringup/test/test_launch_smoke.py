"""Launch-testing smoke for an observability preset with all work disabled."""

import os
from pathlib import Path

os.environ.setdefault('ROS_LOG_DIR', '/tmp/uv_bringup_launch_testing')

import launch
import launch_testing
import pytest
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


LAUNCH_ROOT = Path(__file__).parents[1] / 'launch'


@pytest.mark.launch_test
def generate_test_description():
    observability = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(LAUNCH_ROOT / 'observability.launch.py')),
        launch_arguments={
            'enable_ai': 'false',
            'enable_preview': 'false',
            'record_session': 'false',
        }.items(),
    )
    return launch.LaunchDescription([
        observability,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestObservabilityLaunch:
    def test_launch_reaches_ready(self):
        """The split observability launch is constructible without GUI or ROS nodes."""
        assert True
