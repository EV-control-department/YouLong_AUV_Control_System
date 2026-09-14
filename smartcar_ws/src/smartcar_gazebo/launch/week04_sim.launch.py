"""Launch the Week 04 Gazebo world and ROS 2 bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    world = PathJoinSubstitution(
        [FindPackageShare("smartcar_gazebo"), "worlds", "lesson04_diff_drive.sdf"]
    )
    gz_sim_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_bridge",
                default_value="true",
                description="Start the ROS 2 to Gazebo bridge",
            ),
            SetEnvironmentVariable("GZ_PARTITION", "smartcar_week04"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={"gz_args": [world, " -r"]}.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="smartcar_ros_gz_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_bridge")),
                arguments=[
                    "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                    "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                ],
            ),
        ]
    )
