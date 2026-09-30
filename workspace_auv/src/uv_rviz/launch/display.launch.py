"""Launch the RViz adapter and the packaged AUV display configuration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    ray_length = LaunchConfiguration('ray_length')
    config_path = (
        Path(get_package_share_directory('uv_rviz')) / 'config' / 'auv.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use ROS simulation time when all source topics use /clock',
        ),
        DeclareLaunchArgument(
            'ray_length',
            default_value='3.0',
            description='Displayed length in metres for perception bearing rays',
        ),
        Node(
            package='uv_rviz',
            executable='visualization_adapter',
            name='uv_rviz_adapter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'ray_length': ray_length,
            }],
        ),
        Node(
            package='uv_rviz',
            executable='robot_description_adapter',
            name='uv_rviz_robot_description',
            output='screen',
        ),
        Node(
            package='uv_rviz',
            executable='tf_bridge',
            name='uv_rviz_tf_bridge',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='uv_rviz',
            output='screen',
            arguments=['-d', str(config_path)],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
