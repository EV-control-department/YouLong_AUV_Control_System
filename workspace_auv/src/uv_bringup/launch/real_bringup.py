"""Real hardware bringup launch file.

Launches micro-ROS agent + hw_manager + all control/perception/nav/task nodes.

Note on navigator: it takes its pose from /basic_motion/pose_info, so basic_motion
must be running and must have received a START goal before /nav/navigate_to works.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _warn_if_nav_enabled(context):
    """Emit a heads-up when the navigator is enabled."""
    if context.launch_configurations.get('enable_nav', 'false').lower() == 'true':
        return [LogInfo(msg=(
            '[real_bringup] navigator enabled: its pose source is '
            '/basic_motion/pose_info — make sure basic_motion is up and a START '
            'goal has been sent, otherwise /nav/navigate_to will refuse to plan.'
        ))]
    return []


def generate_launch_description():
    declare_enable_ai = DeclareLaunchArgument(
        'enable_ai', default_value='true',
        description='Enable AI perception nodes'
    )
    declare_enable_nav = DeclareLaunchArgument(
        'enable_nav', default_value='false',
        description='Enable navigation node (needs basic_motion + START for pose)'
    )
    declare_enable_task = DeclareLaunchArgument(
        'enable_task', default_value='false',
        description='Enable task runner'
    )
    enable_ai = LaunchConfiguration('enable_ai')
    enable_nav = LaunchConfiguration('enable_nav')
    enable_task = LaunchConfiguration('enable_task')

    # Hardware manager (replaces sim_bridge on real hardware)
    hw_manager = Node(
        package='uv_hm',
        executable='hw_manager',
        name='hw_manager',
        output='screen',
    )

    basic_motion = Node(
        package='uv_control',
        executable='basic_motion',
        name='basic_motion',
        output='screen',
    )

    vision = Node(
        package='uv_perception',
        executable='vision',
        name='vision',
        output='screen',
        parameters=[{'sim_mode': False}],
        condition=IfCondition(enable_ai),
    )

    position = Node(
        package='uv_perception',
        executable='position',
        name='position',
        output='screen',
        condition=IfCondition(enable_ai),
    )

    navigator = Node(
        package='uv_nav',
        executable='navigator',
        name='navigator',
        output='screen',
        condition=IfCondition(enable_nav),
    )

    task_runner = Node(
        package='uv_task',
        executable='task_runner',
        name='task_runner',
        output='screen',
        condition=IfCondition(enable_task),
    )

    return LaunchDescription([
        declare_enable_ai,
        declare_enable_nav,
        declare_enable_task,
        OpaqueFunction(function=_warn_if_nav_enabled),
        hw_manager,
        basic_motion,
        vision,
        position,
        navigator,
        task_runner,
    ])
