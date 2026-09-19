from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    simulation_data = LaunchConfiguration('simulation_data')
    scenario_desc = LaunchConfiguration('scenario_desc')
    simulation_rate = LaunchConfiguration('simulation_rate')
    
    simulation_data_arg = DeclareLaunchArgument(
        'simulation_data',
        default_value = ''
    )

    scenario_desc_arg = DeclareLaunchArgument(
        'scenario_desc',
        default_value = ''
    )

    simulation_rate_arg = DeclareLaunchArgument(
        'simulation_rate',
        default_value = '100.0'
    )
    stonefish_simulator_nogpu_node = Node(
            package='stonefish_ros2',
            executable='stonefish_simulator_nogpu',
            namespace='stonefish_ros2',
            name='stonefish_simulator_nogpu',
            exec_name='stonefish_simulator_nogpu',
            arguments=[simulation_data, scenario_desc, simulation_rate],
            output='screen',
    )

    def _stonefish_exit(event, context):
        """Stop the complete SIL graph when the simulator is gone."""
        if context.is_shutdown:
            return []
        return [
            LogInfo(msg=[
                'Stonefish simulator exited (returncode=',
                str(event.returncode), '); shutting down simulation',
            ]),
            EmitEvent(event=Shutdown(reason='Stonefish simulator exited')),
        ]

    return LaunchDescription([
        simulation_data_arg,
        scenario_desc_arg,
        simulation_rate_arg,
        stonefish_simulator_nogpu_node,
        RegisterEventHandler(OnProcessExit(
            target_action=stonefish_simulator_nogpu_node,
            on_exit=_stonefish_exit,
        )),
    ])
