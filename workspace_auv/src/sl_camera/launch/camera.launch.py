"""Launch the independent structured-light camera using a ROS parameter YAML."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=get_package_share_directory('sl_camera')
            + '/config/default.yaml'),
        Node(package='sl_camera', executable='sl_camera', name='sl_camera',
             parameters=[LaunchConfiguration('config')], output='both'),
    ])
