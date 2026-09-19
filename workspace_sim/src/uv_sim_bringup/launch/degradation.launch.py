"""Optional deterministic sensor-degradation adapters for SIL experiments."""

from __future__ import annotations

from auv_protocol.topics import (
    DOWN_STITCHED,
    DVL_VELOCITY,
    FRONT_STITCHED,
    IMU,
    SIM_DEGRADED_DOWN_STITCHED,
    SIM_DEGRADED_DVL_VELOCITY,
    SIM_DEGRADED_FRONT_STITCHED,
    SIM_DEGRADED_IMU,
    SIM_DEGRADED_USBL,
    USBL_MEASUREMENT,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(package, executable, name, parameters):
    return Node(
        package=package,
        executable=executable,
        name=name,
        exec_name=name,
        output='both',
        parameters=[parameters],
    )


def generate_launch_description():
    enabled = LaunchConfiguration('enable_degradation')
    seed = LaunchConfiguration('degradation_seed')
    dvl_dropout = LaunchConfiguration('dvl_dropout_probability')
    dvl_beam_loss = LaunchConfiguration('dvl_beam_loss_probability')
    dvl_noise = LaunchConfiguration('dvl_noise_stddev')
    dvl_altitude_noise = LaunchConfiguration('dvl_altitude_noise_stddev')
    dvl_bottom_lock_loss = LaunchConfiguration(
        'dvl_bottom_lock_loss_probability')
    imu_dropout = LaunchConfiguration('imu_dropout_probability')
    imu_accel_noise = LaunchConfiguration('imu_accelerometer_noise_stddev')
    imu_gyro_noise = LaunchConfiguration('imu_gyroscope_noise_stddev')
    imu_accel_bias = LaunchConfiguration('imu_accelerometer_bias')
    imu_gyro_bias = LaunchConfiguration('imu_gyroscope_bias')
    imu_random_walk = LaunchConfiguration('imu_random_walk_stddev')
    visual_dropout = LaunchConfiguration('visual_dropout_probability')
    visual_brightness = LaunchConfiguration('visual_brightness_scale')
    visual_noise = LaunchConfiguration('visual_noise_stddev')
    visual_blur = LaunchConfiguration('visual_blur_kernel')
    dvl_delay = LaunchConfiguration('dvl_delay_ms')
    imu_delay = LaunchConfiguration('imu_delay_ms')
    visual_delay = LaunchConfiguration('visual_delay_ms')
    usbl_dropout = LaunchConfiguration('usbl_dropout_probability')
    usbl_noise = LaunchConfiguration('usbl_position_noise_stddev')
    usbl_outlier = LaunchConfiguration('usbl_outlier_probability')
    usbl_outlier_stddev = LaunchConfiguration('usbl_outlier_stddev')
    usbl_delay = LaunchConfiguration('usbl_delay_ms')

    return LaunchDescription([
        DeclareLaunchArgument('enable_degradation', default_value='false'),
        DeclareLaunchArgument('degradation_seed', default_value='0'),
        DeclareLaunchArgument('dvl_dropout_probability', default_value='0.0'),
        DeclareLaunchArgument('dvl_beam_loss_probability', default_value='0.0'),
        DeclareLaunchArgument('dvl_noise_stddev', default_value='0.0'),
        DeclareLaunchArgument(
            'dvl_altitude_noise_stddev', default_value='0.0'),
        DeclareLaunchArgument(
            'dvl_bottom_lock_loss_probability', default_value='0.0'),
        DeclareLaunchArgument('imu_dropout_probability', default_value='0.0'),
        DeclareLaunchArgument(
            'imu_accelerometer_noise_stddev', default_value='0.0'),
        DeclareLaunchArgument(
            'imu_gyroscope_noise_stddev', default_value='0.0'),
        DeclareLaunchArgument(
            'imu_accelerometer_bias', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument(
            'imu_gyroscope_bias', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('imu_random_walk_stddev', default_value='0.0'),
        DeclareLaunchArgument('visual_dropout_probability', default_value='0.0'),
        DeclareLaunchArgument('visual_brightness_scale', default_value='1.0'),
        DeclareLaunchArgument('visual_noise_stddev', default_value='0.0'),
        DeclareLaunchArgument('visual_blur_kernel', default_value='0'),
        DeclareLaunchArgument('dvl_delay_ms', default_value='0.0'),
        DeclareLaunchArgument('imu_delay_ms', default_value='0.0'),
        DeclareLaunchArgument('visual_delay_ms', default_value='0.0'),
        DeclareLaunchArgument('usbl_dropout_probability', default_value='0.0'),
        DeclareLaunchArgument('usbl_position_noise_stddev', default_value='0.0'),
        DeclareLaunchArgument('usbl_outlier_probability', default_value='0.0'),
        DeclareLaunchArgument('usbl_outlier_stddev', default_value='0.0'),
        DeclareLaunchArgument('usbl_delay_ms', default_value='0.0'),
        GroupAction([
            _node('uv_sim_degradation', 'dvl_degradation', 'dvl_degradation', {
                'input_topic': DVL_VELOCITY,
                'output_topic': SIM_DEGRADED_DVL_VELOCITY,
                'dropout_probability': dvl_dropout,
                'beam_loss_probability': dvl_beam_loss,
                'noise_stddev': dvl_noise,
                'altitude_noise_stddev': dvl_altitude_noise,
                'bottom_lock_loss_probability': dvl_bottom_lock_loss,
                'delay_ms': dvl_delay,
                'seed': seed,
            }),
            _node('uv_sim_degradation', 'imu_degradation', 'imu_degradation', {
                'input_topic': IMU,
                'output_topic': SIM_DEGRADED_IMU,
                'dropout_probability': imu_dropout,
                'accelerometer_noise_stddev': imu_accel_noise,
                'gyroscope_noise_stddev': imu_gyro_noise,
                'accelerometer_bias': imu_accel_bias,
                'gyroscope_bias': imu_gyro_bias,
                'random_walk_stddev': imu_random_walk,
                'delay_ms': imu_delay,
                'seed': seed,
            }),
            _node('uv_sim_degradation', 'camera_degradation',
                  'front_camera_degradation', {
                      'camera': 'front', 'input_topic': FRONT_STITCHED,
                      'output_topic': SIM_DEGRADED_FRONT_STITCHED,
                      'dropout_probability': visual_dropout,
                      'brightness_scale': visual_brightness,
                      'gaussian_noise_stddev': visual_noise,
                      'blur_kernel': visual_blur,
                      'delay_ms': visual_delay, 'seed': seed,
                  }),
            _node('uv_sim_degradation', 'camera_degradation',
                  'down_camera_degradation', {
                      'camera': 'downward', 'input_topic': DOWN_STITCHED,
                      'output_topic': SIM_DEGRADED_DOWN_STITCHED,
                      'dropout_probability': visual_dropout,
                      'brightness_scale': visual_brightness,
                      'gaussian_noise_stddev': visual_noise,
                      'blur_kernel': visual_blur,
                      'delay_ms': visual_delay,
                      'seed': seed,
                  }),
            _node('uv_sim_degradation', 'usbl_degradation', 'usbl_degradation', {
                'input_topic': USBL_MEASUREMENT,
                'output_topic': SIM_DEGRADED_USBL,
                'dropout_probability': usbl_dropout,
                'position_noise_stddev': usbl_noise,
                'outlier_probability': usbl_outlier,
                'outlier_stddev': usbl_outlier_stddev,
                'delay_ms': usbl_delay,
                'seed': seed,
            }),
        ], condition=IfCondition(enabled)),
    ])
