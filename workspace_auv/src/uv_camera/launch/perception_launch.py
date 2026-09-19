"""Launch the camera AI and object-localization component."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from auv_protocol.topics import DOWN_STITCHED, FRONT_STITCHED


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _workspace_python():
    """Use the interpreter where setup_workspace_python.sh installs AI deps."""
    launch_path = Path(__file__).resolve()
    for parent in launch_path.parents:
        for candidate in (
                parent / '.venv' / 'bin' / 'python',
                parent / 'workspace_auv' / '.venv' / 'bin' / 'python'):
            if candidate.is_file():
                return str(candidate)
    return None


def generate_launch_description():
    enable_ai = LaunchConfiguration("enable_ai")
    sim_mode = LaunchConfiguration("sim_mode")
    inference_fps = LaunchConfiguration("inference_fps")
    dataset_fps = LaunchConfiguration("dataset_fps")
    dataset_debug = LaunchConfiguration("dataset_debug")
    dataset_debug_period_sec = LaunchConfiguration("dataset_debug_period_sec")
    dataset_submit_timeout_sec = LaunchConfiguration("dataset_submit_timeout_sec")
    dataset_writer_workers = LaunchConfiguration("dataset_writer_workers")
    dataset_webp_method = LaunchConfiguration("dataset_webp_method")
    dataset_fsync_each_file = LaunchConfiguration("dataset_fsync_each_file")
    camera_startup_timeout_sec = LaunchConfiguration("camera_startup_timeout_sec")
    inference_threads = LaunchConfiguration("inference_threads")
    confidence = LaunchConfiguration("confidence")
    gate_feature_mode = LaunchConfiguration("gate_feature_mode")
    enable_gortc = LaunchConfiguration("enable_gortc")
    stream_annotated = LaunchConfiguration("stream_annotated")
    stream_pose_overlay = LaunchConfiguration("stream_pose_overlay")
    mjpeg_port = LaunchConfiguration("mjpeg_port")
    annotated_max_width = LaunchConfiguration("annotated_max_width")
    save_dataset = LaunchConfiguration("save_dataset")
    dataset_dir = LaunchConfiguration("dataset_dir")
    dataset_queue_size = LaunchConfiguration("dataset_queue_size")
    dataset_png_compression = LaunchConfiguration("dataset_png_compression")
    dataset_format = LaunchConfiguration("dataset_format")
    camera_config_profile = LaunchConfiguration("camera_config_profile")
    camera_config_dir = LaunchConfiguration("camera_config_dir")
    profile_params = LaunchConfiguration("profile_params")
    object_localizer_params = LaunchConfiguration("object_localizer_params")
    detection_report_period = LaunchConfiguration("detection_report_period_sec")
    front_image_topic = LaunchConfiguration("front_image_topic")
    down_image_topic = LaunchConfiguration("down_image_topic")

    def _nodes(context):
        ai_enabled = _as_bool(enable_ai.perform(context))
        recording_enabled = _as_bool(save_dataset.perform(context))
        profile = profile_params.perform(context).strip()
        localizer_config = object_localizer_params.perform(context).strip()
        requested_camera_profile = camera_config_profile.perform(context).strip()
        requested_camera_dir = camera_config_dir.perform(context).strip()

        # An explicit launch argument overrides a profile file.  Empty/auto
        # values deliberately leave the profile file (or sim_mode selection)
        # in control.
        camera_overrides = {}
        if requested_camera_profile and requested_camera_profile.lower() != "auto":
            camera_overrides["camera_config_profile"] = camera_config_profile
        if requested_camera_dir:
            camera_overrides["camera_config_dir"] = camera_config_dir

        vision_parameters = []
        if profile:
            vision_parameters.append(profile)
        vision_parameters.append({
            "enable_ai": enable_ai,
            "sim_mode": sim_mode,
            "inference_fps": inference_fps,
            "dataset_fps": dataset_fps,
            "dataset_debug": dataset_debug,
            "dataset_debug_period_sec": dataset_debug_period_sec,
            "dataset_submit_timeout_sec": dataset_submit_timeout_sec,
            "dataset_writer_workers": dataset_writer_workers,
            "dataset_webp_method": dataset_webp_method,
            "dataset_fsync_each_file": dataset_fsync_each_file,
            "camera_startup_timeout_sec": camera_startup_timeout_sec,
            "inference_threads": inference_threads,
            "confidence": confidence,
            "gate_feature_mode": gate_feature_mode,
            "enable_gortc": enable_gortc,
            "stream_annotated": stream_annotated,
            "stream_pose_overlay": stream_pose_overlay,
            "mjpeg_port": mjpeg_port,
            "annotated_max_width": annotated_max_width,
            "save_dataset": save_dataset,
            "dataset_dir": dataset_dir,
            "dataset_queue_size": dataset_queue_size,
            "dataset_png_compression": dataset_png_compression,
            "dataset_format": dataset_format,
        })
        if camera_overrides:
            vision_parameters.append(camera_overrides)

        localizer_parameters = []
        if profile:
            localizer_parameters.append(profile)
        if localizer_config:
            localizer_parameters.append(localizer_config)
        localizer_parameters.append({
            "sim_mode": sim_mode,
            "detection_report_period_sec": detection_report_period,
        })
        if camera_overrides:
            localizer_parameters.append(camera_overrides)

        nodes = []
        workspace_python = _workspace_python()
        # Recording consumes the sensor stream before the YOLO FrameGate, so
        # it must be possible to run uv_camera without enabling detection.
        if ai_enabled or recording_enabled:
            nodes.append(Node(
                package="uv_camera",
                executable="uv_camera",
                name="uv_camera",
                exec_name="uv_camera",
                prefix=workspace_python,
                output="both",
                parameters=vision_parameters,
                remappings=[
                    ('/tf', '/auv/tf'), ('/tf_static', '/auv/tf_static'),
                    (FRONT_STITCHED, front_image_topic),
                    (DOWN_STITCHED, down_image_topic),
                ],
                respawn=True,
                respawn_delay=1.0,
            ))
        if ai_enabled:
            nodes.append(Node(
                package="uv_camera",
                executable="object_localizer",
                name="object_localizer",
                exec_name="object_localizer",
                prefix=workspace_python,
                output="both",
                parameters=localizer_parameters,
                remappings=[
                    ('/tf', '/auv/tf'), ('/tf_static', '/auv/tf_static'),
                    (FRONT_STITCHED, front_image_topic),
                    (DOWN_STITCHED, down_image_topic),
                ],
                respawn=True,
                respawn_delay=1.0,
            ))
        return nodes

    return LaunchDescription([
        DeclareLaunchArgument("enable_ai", default_value="false"),
        DeclareLaunchArgument("sim_mode", default_value="false"),
        DeclareLaunchArgument("inference_fps", default_value="5.0"),
        DeclareLaunchArgument("dataset_fps", default_value="5.0"),
        DeclareLaunchArgument("dataset_debug", default_value="true"),
        DeclareLaunchArgument("dataset_debug_period_sec", default_value="1.0"),
        DeclareLaunchArgument("dataset_submit_timeout_sec", default_value="1.0"),
        DeclareLaunchArgument("dataset_writer_workers", default_value="4"),
        DeclareLaunchArgument("dataset_webp_method", default_value="0"),
        DeclareLaunchArgument("dataset_fsync_each_file", default_value="false"),
        DeclareLaunchArgument("camera_startup_timeout_sec", default_value="5.0"),
        DeclareLaunchArgument("inference_threads", default_value="2"),
        DeclareLaunchArgument("confidence", default_value="0.8"),
        DeclareLaunchArgument("gate_feature_mode", default_value="auto"),
        DeclareLaunchArgument("enable_gortc", default_value="true"),
        DeclareLaunchArgument("stream_annotated", default_value="true"),
        DeclareLaunchArgument("stream_pose_overlay", default_value="false"),
        DeclareLaunchArgument("mjpeg_port", default_value="8090"),
        DeclareLaunchArgument("annotated_max_width", default_value="0"),
        DeclareLaunchArgument("save_dataset", default_value="false"),
        DeclareLaunchArgument("dataset_dir", default_value="records/datasets"),
        DeclareLaunchArgument("dataset_queue_size", default_value="32"),
        DeclareLaunchArgument("dataset_png_compression", default_value="1"),
        DeclareLaunchArgument("dataset_format", default_value="webp_lossless"),
        DeclareLaunchArgument("camera_config_profile", default_value="auto"),
        DeclareLaunchArgument("camera_config_dir", default_value=""),
        DeclareLaunchArgument("profile_params", default_value=""),
        DeclareLaunchArgument("object_localizer_params", default_value=""),
        DeclareLaunchArgument("detection_report_period_sec", default_value="5.0"),
        DeclareLaunchArgument("front_image_topic", default_value=FRONT_STITCHED),
        DeclareLaunchArgument("down_image_topic", default_value=DOWN_STITCHED),
        OpaqueFunction(function=_nodes),
    ])
