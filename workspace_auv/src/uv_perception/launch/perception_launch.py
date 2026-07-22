from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_publish_annotated = DeclareLaunchArgument(
        'publish_annotated', default_value='false',
        description='Publish YOLO annotated images with detection boxes'
    )
    declare_segment_model_path = DeclareLaunchArgument(
        'segment_model_path', default_value='',
        description='Path to YOLO-Seg model for pipe line following'
    )
    publish_annotated = LaunchConfiguration('publish_annotated')
    segment_model_path = LaunchConfiguration('segment_model_path')

    return LaunchDescription([
        declare_publish_annotated,
        declare_segment_model_path,
        Node(
            package='uv_perception',
            executable='vision',
            name='vision',
            output='screen',
            parameters=[{
                'publish_annotated': publish_annotated,
                'segment_model_path': segment_model_path,
            }],
        ),
        Node(
            package='uv_perception',
            executable='position',
            name='position',
            output='screen',
        ),
    ])
