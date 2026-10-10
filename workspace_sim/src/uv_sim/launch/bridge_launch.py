"""uv_sim entry point: the simulation-side nodes that stand in for hardware.

Eight nodes, one responsibility each (T5's seven, plus ``laser_sim`` for the
line laser in the vehicle scene):

=====================================  ==========================================
``odom_sim`` / ``imu_sim``             ``/auv/*`` -> ``/sim/sensor/*`` relays
``dvl_sim`` / ``pressure_sim``         (the DVL one is AQUA-SLAM's adapter)
``nav_sim``                            ``/sim/sensor/*`` -> ``/zit6/state/{pos,vel}``
                                       (SIL) or ``/zit6/sim/nav`` (HIL)
``mcu_sim``                            setpoints -> PID -> mixing -> thrusters,
                                       plus ``/zit6/state/{status,thr,zithbt}``
``camera_relay``                       images -> ``/auv/*_cam/*`` (+ stitching)
``laser_sim``                          line-laser ``LaserScan`` -> stripe points
=====================================  ==========================================

``hil_mode`` is the **single** knob for the SIL/HIL switch (there used to be one
``sim_bridge`` with an internal branch; now the mode is composed from the nodes):

* ``hil_mode:=false`` (default, the acceptance path) starts all eight.
* ``hil_mode:=true`` starts six: ``dvl_sim``/``pressure_sim`` are skipped because
  the real MCU subscribes ``/auv/*`` directly, and ``nav_sim`` is started with
  ``publish_sim_nav:=true, publish_state:=false`` so the MCU keeps ownership of
  ``/zit6/state/*`` (invariant I2).  ``mcu_sim`` becomes a thrust relay.
  ``laser_sim`` runs in both modes -- like ``camera_relay`` it adapts a
  *simulated* sensor, not part of the MCU contract, and nothing on the real
  vehicle owns it.

``nav_sim``'s two roles are derived from ``hil_mode`` here rather than exposed as
extra launch arguments: two knobs that must agree is exactly the failure mode I2
is about.  The node also refuses to start if both are true (direct ``ros2 run``).

Lives here rather than in the preset so that the node set stays owned by the
package; the presets only decide whether to include this file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

HIL = LaunchConfiguration('hil_mode')


def generate_launch_description():
    # launch_ros coerces a substitution's text to the declared parameter type, so
    # 'True'/'False' become booleans (the same mechanism the pre-T5 single-node
    # launch relied on for hil_mode).
    publish_state = PythonExpression(["'", HIL, "' == 'false'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            'hil_mode', default_value='false',
            description=(
                'true: a real MCU owns PID, mixing and /zit6/state/* -- only the '
                'sensor relays needed for the sim/nav injection, the thrust relay '
                'and the camera relay start; false: simulate the MCU'
            )),
        DeclareLaunchArgument(
            'pid_config_file', default_value='',
            description=(
                'YAML file with the PID gains for the simulated MCU; empty means '
                'the packaged uv_sim/config/sim_pid.yaml'
            )),

        Node(
            package='uv_sim',
            executable='odom_sim',
            name='odom_sim',
            output='screen',
        ),
        Node(
            package='uv_sim',
            executable='imu_sim',
            name='imu_sim',
            output='screen',
        ),
        Node(
            package='uv_sim',
            executable='dvl_sim',
            name='dvl_sim',
            output='screen',
            condition=UnlessCondition(HIL),
        ),
        Node(
            package='uv_sim',
            executable='pressure_sim',
            name='pressure_sim',
            output='screen',
            condition=UnlessCondition(HIL),
        ),
        Node(
            package='uv_sim',
            executable='nav_sim',
            name='nav_sim',
            output='screen',
            parameters=[{
                'publish_state': publish_state,
                'publish_sim_nav': HIL,
            }],
        ),
        Node(
            package='uv_sim',
            executable='mcu_sim',
            name='mcu_sim',
            output='screen',
            parameters=[{
                'hil_mode': HIL,
                'pid_config_file': LaunchConfiguration('pid_config_file'),
            }],
        ),
        Node(
            package='uv_sim',
            executable='camera_relay',
            name='camera_relay',
            output='screen',
        ),
        Node(
            package='uv_sim',
            executable='laser_sim',
            name='laser_sim',
            output='screen',
        ),
    ])
