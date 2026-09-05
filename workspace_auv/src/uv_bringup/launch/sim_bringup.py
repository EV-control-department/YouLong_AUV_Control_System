"""Simulation bringup launch file.

Launches Stonefish simulator + all control/perception/nav/task nodes.
"""

import os
import re
import shutil
import subprocess
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node


def _workspace_python_runtime():
    """Find the repository-local Python runtime, if it was bootstrapped."""
    from pathlib import Path

    candidates = []
    launch_file = Path(__file__).resolve()
    for parent in (launch_file.parent, *launch_file.parents):
        candidates.append(parent / '.venv' / 'bin' / 'python')

    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    return None


def _nvidia_available() -> bool:
    """Return whether an NVIDIA GPU is visible to the installed driver."""
    nvidia_smi = shutil.which('nvidia-smi')
    if nvidia_smi is None:
        return False
    try:
        result = subprocess.run(
            [nvidia_smi, '--query-gpu=name', '--format=csv,noheader'],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        return bool(result.stdout.strip())
    except (OSError, subprocess.SubprocessError):
        return False


def _x11_active_window_center():
    """Return the center of the currently active X11 window, if available."""
    try:
        active = subprocess.run(
            ['xprop', '-root', '_NET_ACTIVE_WINDOW'],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        window_match = re.search(r'0x[0-9a-fA-F]+', active.stdout)
        if not window_match:
            return None
        geometry = subprocess.run(
            ['xwininfo', '-id', window_match.group(0)],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        x_match = re.search(r'Absolute upper-left X:\s+(-?\d+)', geometry.stdout)
        y_match = re.search(r'Absolute upper-left Y:\s+(-?\d+)', geometry.stdout)
        w_match = re.search(r'Width:\s+(\d+)', geometry.stdout)
        h_match = re.search(r'Height:\s+(\d+)', geometry.stdout)
        if all((x_match, y_match, w_match, h_match)):
            return (
                int(x_match.group(1)) + int(w_match.group(1)) / 2.0,
                int(y_match.group(1)) + int(h_match.group(1)) / 2.0,
            )
    except (OSError, subprocess.SubprocessError):
        pass
    return None


def _focused_monitor():
    """Return ``(x, y, width, height)`` for the active monitor."""
    monitors = []
    try:
        result = subprocess.run(
            ['xrandr', '--query'],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        for line in result.stdout.splitlines():
            if ' connected' not in line:
                continue
            match = re.search(r'(\d+)x(\d+)\+(-?\d+)\+(-?\d+)', line)
            if match:
                monitors.append({
                    'x': int(match.group(3)),
                    'y': int(match.group(4)),
                    'width': int(match.group(1)),
                    'height': int(match.group(2)),
                    'primary': ' connected primary ' in f' {line} ',
                })
    except (OSError, subprocess.SubprocessError):
        pass

    if not monitors:
        return 0, 0, 1920, 1080

    center = _x11_active_window_center()
    if center is not None:
        for monitor in monitors:
            if (
                monitor['x'] <= center[0] < monitor['x'] + monitor['width']
                and monitor['y'] <= center[1] < monitor['y'] + monitor['height']
            ):
                return tuple(monitor[key] for key in ('x', 'y', 'width', 'height'))

    monitor = next((item for item in monitors if item['primary']), monitors[0])
    return tuple(monitor[key] for key in ('x', 'y', 'width', 'height'))


def generate_launch_description():
    """Build the full simulation launch description."""
    from uv_log.session import default_output_root

    monitor_x, monitor_y, display_width, display_height = _focused_monitor()
    nvidia_available = _nvidia_available()
    # Keep the default simulation window modest.  A large Stonefish window
    # makes the renderer consume a disproportionate amount of GPU/CPU even
    # when nobody is looking at the preview.  All values remain overridable.
    default_sim_width = min(1280, max(800, int(display_width * 0.50)))
    default_sim_height = min(
        720, max(300, min(int(default_sim_width * 9 / 16), display_height)))
    # The preview width is the whole column left on the right-hand side.  The
    # helper later constrains each stream by its real image ratio and half of
    # the monitor height, so this is a maximum rather than a forced ratio.
    default_preview_width = max(160, display_width - default_sim_width)
    default_preview_height = max(90, display_height // 2)
    # NVIDIA availability is still used for the OpenGL environment below,
    # but it is not a reason to enable expensive defaults on a laptop.
    default_render_quality = 'low'
    default_ai_inference_fps = '3.0'

    # Arguments
    declare_enable_ai = DeclareLaunchArgument(
        'enable_ai', default_value='true',
        description='Enable AI perception nodes'
    )
    declare_ai_inference_fps = DeclareLaunchArgument(
        'ai_inference_fps', default_value=default_ai_inference_fps,
        description=(
            'Maximum AI inference rate per camera; lower values reduce CPU load'
        )
    )
    declare_inference_threads = DeclareLaunchArgument(
        'inference_threads', default_value='2',
        description=(
            'Maximum PyTorch CPU threads used by AI; lower values keep the '
            'simulator responsive'
        )
    )
    declare_camera_stitch_fps = DeclareLaunchArgument(
        'camera_stitch_fps', default_value='5.0',
        description=(
            'Maximum rate of the stitched stereo image topics; 5 Hz is the '
            'lightweight default'
        )
    )
    declare_publish_raw_camera_topics = DeclareLaunchArgument(
        'publish_raw_camera_topics', default_value='false',
        description=(
            'Republish four individual camera images in addition to stitched '
            'images; disabled by default because no perception node consumes them'
        )
    )
    declare_enable_nav = DeclareLaunchArgument(
        'enable_nav', default_value='false',
        description='Enable navigation node'
    )
    declare_enable_task = DeclareLaunchArgument(
        'enable_task', default_value='false',
        description='Enable task runner'
    )
    declare_scenario = DeclareLaunchArgument(
        'scenario_desc', default_value='guoshui_2026_cruise_seeded.scn',
        description='Stonefish scenario file name (in Data/ directory); generated from scene_seed by default'
    )
    declare_scene_seed = DeclareLaunchArgument(
        'scene_seed', default_value='0',
        description='Deterministic Guoshui scene seed; 0 keeps the fixed baseline layout'
    )
    declare_target_id = DeclareLaunchArgument(
        'target_id', default_value='yellow_golf',
        description='Competition target metadata: yellow_golf, pink_golf, or red_ring'
    )
    declare_open_annotated_windows = DeclareLaunchArgument(
        'open_annotated_windows', default_value='true',
        description='Open front/down annotated preview windows by default'
    )
    declare_stream_annotated = DeclareLaunchArgument(
        'stream_annotated', default_value='true',
        description=(
            'Generate annotated MJPEG streams for the front/down windows; '
            'set false for raw-only operation to reduce CPU and JPEG work'
        )
    )
    declare_sim_window_width = DeclareLaunchArgument(
        'sim_window_width', default_value=str(default_sim_width),
        description='Stonefish window width in pixels'
    )
    declare_sim_window_height = DeclareLaunchArgument(
        'sim_window_height', default_value=str(default_sim_height),
        description='Stonefish window height in pixels'
    )
    declare_render_quality = DeclareLaunchArgument(
        'render_quality', default_value=default_render_quality,
        description='Stonefish rendering quality: low, medium, or high'
    )
    declare_preview_width = DeclareLaunchArgument(
        'preview_width', default_value=str(default_preview_width),
        description='Width of each annotated preview window in pixels'
    )
    declare_preview_height = DeclareLaunchArgument(
        'preview_height', default_value=str(default_preview_height),
        description='Maximum height of each annotated preview window in pixels'
    )
    declare_preview_port = DeclareLaunchArgument(
        'preview_port', default_value='8090',
        description='uv_camera MJPEG port used by the annotated previews'
    )
    declare_preview_wait_timeout = DeclareLaunchArgument(
        'preview_wait_timeout', default_value='60.0',
        description='Seconds to wait for the annotated MJPEG service'
    )
    declare_record_session = DeclareLaunchArgument(
        'record_session', default_value='false',
        description='Record a crash-resilient ROS/video/log session'
    )
    # The annotated windows need the MJPEG endpoint.  Keep preview enabled by
    # default for interactive simulation; enable_preview:=false still disables
    # both the windows and video capture when a headless run is desired.
    declare_enable_preview = DeclareLaunchArgument(
        'enable_preview',
        default_value='true',
        description=(
            'Enable MJPEG/go2rtc preview and annotated windows; set false '
            'to disable preview and video capture'
        ),
    )
    declare_record_root = DeclareLaunchArgument(
        'record_root', default_value=str(default_output_root()),
        description='Directory under which recording sessions are created'
    )
    declare_record_raw_video = DeclareLaunchArgument(
        'record_raw_video', default_value='false',
        description=(
            'Deprecated compatibility option; use record_video_mode instead'
        )
    )
    declare_record_video_mode = DeclareLaunchArgument(
        'record_video_mode', default_value='raw',
        description=(
            'Video streams to record: raw, annotated, or both; raw keeps the '
            'video at the camera topic rate'
        )
    )
    declare_record_video_format = DeclareLaunchArgument(
        'record_video_format', default_value='jpeg',
        description=(
            'Video archive format: jpeg stores source JPEG frames and '
            'decodes during playback; ts keeps legacy H.264 transcoding'))
    declare_record_video_fps = DeclareLaunchArgument(
        'record_video_fps',
        default_value=LaunchConfiguration('camera_stitch_fps'),
        description=(
            'Recorded video FPS; defaults to camera_stitch_fps so the video '
            'timebase follows the stitched image topic'
        )
    )
    declare_record_video_codec = DeclareLaunchArgument(
        'record_video_codec', default_value='libx264',
        description=(
            'FFmpeg video codec; libx264 is portable, h264_nvenc can reduce '
            'CPU usage when supported by the machine'
        )
    )
    declare_record_image_topics = DeclareLaunchArgument(
        'record_image_topics', default_value='false',
        description=(
            'Deprecated compatibility option; camera image message types are '
            'always excluded from rosbag'
        )
    )
    declare_video_segment_seconds = DeclareLaunchArgument(
        'video_segment_seconds', default_value='2.0',
        description='Length of each crash-recoverable video archive chunk'
    )
    declare_bag_segment_seconds = DeclareLaunchArgument(
        'bag_segment_seconds', default_value='10.0',
        description='Length of each rosbag recording part'
    )
    declare_record_use_sim_time = DeclareLaunchArgument(
        'record_use_sim_time', default_value='false',
        description=(
            'Use /clock for rosbag timestamps; requires a /clock publisher'
        )
    )
    enable_ai = LaunchConfiguration('enable_ai')
    ai_inference_fps = LaunchConfiguration('ai_inference_fps')
    inference_threads = LaunchConfiguration('inference_threads')
    camera_stitch_fps = LaunchConfiguration('camera_stitch_fps')
    publish_raw_camera_topics = LaunchConfiguration('publish_raw_camera_topics')
    enable_nav = LaunchConfiguration('enable_nav')
    enable_task = LaunchConfiguration('enable_task')
    scenario_desc = LaunchConfiguration('scenario_desc')
    scene_seed = LaunchConfiguration('scene_seed')
    target_id = LaunchConfiguration('target_id')
    open_annotated_windows = LaunchConfiguration('open_annotated_windows')
    stream_annotated = LaunchConfiguration('stream_annotated')
    sim_window_width = LaunchConfiguration('sim_window_width')
    sim_window_height = LaunchConfiguration('sim_window_height')
    render_quality = LaunchConfiguration('render_quality')
    preview_width = LaunchConfiguration('preview_width')
    preview_height = LaunchConfiguration('preview_height')
    preview_port = LaunchConfiguration('preview_port')
    preview_wait_timeout = LaunchConfiguration('preview_wait_timeout')
    record_session = LaunchConfiguration('record_session')
    enable_preview = LaunchConfiguration('enable_preview')
    record_root = LaunchConfiguration('record_root')
    record_raw_video = LaunchConfiguration('record_raw_video')
    record_video_mode = LaunchConfiguration('record_video_mode')
    record_video_format = LaunchConfiguration('record_video_format')
    record_video_fps = LaunchConfiguration('record_video_fps')
    record_video_codec = LaunchConfiguration('record_video_codec')
    record_image_topics = LaunchConfiguration('record_image_topics')
    video_segment_seconds = LaunchConfiguration('video_segment_seconds')
    bag_segment_seconds = LaunchConfiguration('bag_segment_seconds')
    record_use_sim_time = LaunchConfiguration('record_use_sim_time')

    # Python ROS nodes that touch images must use the workspace-local NumPy
    # runtime. Otherwise a user-site NumPy 2.x can be selected before ROS 2
    # Jazzy's NumPy 1.x-built cv_bridge and crash sim_bridge.
    workspace_python = _workspace_python_runtime()
    python_node_kwargs = {'prefix': workspace_python} if workspace_python else {}

    def _recording_actions(context):
        """Create the session before nodes so ROS_LOG_DIR covers all nodes."""
        if record_session.perform(context).strip().lower() not in (
                '1', 'true', 'yes', 'on'):
            return []

        from uv_log.session import create_session

        paths = create_session(record_root.perform(context))
        recorder = Node(
            package='uv_log',
            executable='record',
            name='uv_log_recorder',
            output='both',
            **python_node_kwargs,
            arguments=[
                '--session-dir', str(paths.root),
                '--segment-duration', video_segment_seconds.perform(context),
                '--bag-duration', bag_segment_seconds.perform(context),
                '--record-raw', record_raw_video.perform(context),
                '--video-mode', record_video_mode.perform(context),
                '--video-format', record_video_format.perform(context),
                '--video-fps', record_video_fps.perform(context),
                '--video-codec', record_video_codec.perform(context),
                '--record-image-topics', record_image_topics.perform(context),
                '--use-sim-time', record_use_sim_time.perform(context),
            ],
        )
        return [
            SetEnvironmentVariable('ROS_LOG_DIR', str(paths.logs / 'ros')),
            LogInfo(msg=['uv_log session: ', str(paths.root)]),
            recorder,
        ]

    # Stonefish simulator paths
    # Use source directory path for Data (simulator needs direct filesystem access)
    from ament_index_python.packages import get_package_share_directory
    stonefish_share = get_package_share_directory('stonefish_ros2')

    # 定位 stonefish_ros2 源码的 Data/ 目录。仓库是"真机workspace_auv + 仿真workspace_sim"
    # 两层布局；不把具体用户目录写死，改为从启动文件和已安装包路径向上查找。
    from pathlib import Path as _Path

    def _find_stonefish_data_dir() -> str:
        candidates = []
        roots = (_Path(__file__).resolve(), _Path(stonefish_share).resolve())
        for root in roots:
            for base in (root, *root.parents):
                candidates.append(base / 'workspace_sim' / 'src' / 'stonefish_ros2' / 'Data')
                candidates.append(base / 'src' / 'stonefish_ros2' / 'Data')

        # 去重，同时保持从当前源码/安装位置向外查找的顺序。
        candidates = list(dict.fromkeys(candidates))
        for c in candidates:
            if c.is_dir() and any(c.glob('*.scn')):
                return str(c)
        raise RuntimeError("无法定位 stonefish_ros2 的 Data 源码目录")

    simulation_data_dir = _find_stonefish_data_dir()
    stonefish_source_dir = str(_Path(simulation_data_dir).parent)

    def _generate_seeded_scene(context):
        seed_text = scene_seed.perform(context)
        try:
            int(seed_text)
        except ValueError as error:
            raise RuntimeError(f'scene_seed must be an integer, got {seed_text!r}') from error

        generator = _Path(simulation_data_dir) / 'generate_guoshui_2026_scene.py'
        template = _Path(simulation_data_dir) / 'guoshui_2026_cruise.scn'
        output = _Path(simulation_data_dir) / 'guoshui_2026_cruise_seeded.scn'
        subprocess.run(
            [
                sys.executable, str(generator),
                '--seed', seed_text,
                '--template', str(template),
                '--output', str(output),
            ],
            check=True,
        )
        return [LogInfo(msg=[
            'Generated Guoshui scene with seed ', seed_text,
            ': ', str(output),
        ])]

    # Build the stonefish simulator node (GPU version — with rendering window)
    # The simulator expects: simulation_data, scenario_desc, rate, res_x, res_y, quality
    stonefish_sim = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        namespace='stonefish_ros2',
        name='stonefish_simulator',
        arguments=[
            simulation_data_dir,
            PathJoinSubstitution([simulation_data_dir, scenario_desc]),
            '100.0',
            sim_window_width,
            sim_window_height,
            render_quality,
        ],
        output='both',
    )

    # Core nodes
    sim_bridge = Node(
        package='uv_sim',
        executable='sim_bridge',
        name='sim_bridge',
        output='both',
        parameters=[{
            'camera_stitch_fps': camera_stitch_fps,
            'publish_raw_camera_topics': publish_raw_camera_topics,
        }],
        **python_node_kwargs,
    )

    basic_motion = Node(
        package='uv_control',
        executable='basic_motion',
        name='basic_motion',
        output='both',
        **python_node_kwargs,
    )

    # Perception: uv_camera node (uv_sensor + uv_ai, same process) + object_localizer
    vision = Node(
        package='uv_camera',
        executable='uv_camera',
        name='uv_camera',
        output='both',
        **python_node_kwargs,
        parameters=[{
            'sim_mode': True,
            'inference_fps': ai_inference_fps,
            'inference_threads': inference_threads,
            'enable_gortc': enable_preview,
            'stream_annotated': stream_annotated,
        }],
        condition=IfCondition(enable_ai),
    )

    object_localizer = Node(
        package='uv_camera',
        executable='object_localizer',
        name='object_localizer',
        output='both',
        respawn=True,
        respawn_delay=1.0,
        **python_node_kwargs,
        # Build simulated profiles from Stonefish CameraInfo and these camera
        # origins, so the P2 baseline sign follows the simulator optical axes.
        parameters=[{
            'calibration_source': 'sim_camera_info',
            'front_left_camera_info_topic':
                '/sim/front_cam/left/camera_info',
            'front_right_camera_info_topic':
                '/sim/front_cam/right/camera_info',
            'down_left_camera_info_topic':
                '/sim/down_cam/left/camera_info',
            'down_right_camera_info_topic':
                '/sim/down_cam/right/camera_info',
            'front_left_translation': [0.23, -0.05, 0.276],
            'front_right_translation': [0.23, 0.05, 0.276],
            # Stonefish ColorCamera: local +Z is forward, +X is image-right,
            # +Y is image-down.  The front sensor rpy is 1.5708,0,1.5708.
            'front_left_rotation': [0.0, 0.0, 1.0,
                                    1.0, 0.0, 0.0,
                                    0.0, 1.0, 0.0],
            'front_right_rotation': [0.0, 0.0, 1.0,
                                     1.0, 0.0, 0.0,
                                     0.0, 1.0, 0.0],
            'down_left_translation': [-0.13, -0.05, 0.2645],
            'down_right_translation': [-0.13, 0.05, 0.2645],
            # Down localization uses the known target height.  Stereo is no
            # longer needed for the position estimate; the right camera only
            # provides an independent consistency check.
            'down_geometry_mode': 'known_height',
            # /zit6/state/pos and PoseInfo.robot_z are relative to the
            # Stonefish spawn.  The scenario spawns XUNYUN at scene depth
            # 0.12 m, so convert target scene depths into this local frame.
            'down_scene_origin_z_m': 0.12,
            'down_default_target_z_m': 1.294,
            'down_target_z_sigma_m': 0.01,
            'down_target_z_json':
                '{"guide_line": 1.294, '
                '"target_rack": 1.00, '
                '"collection_frame": 0.94, '
                '"yellow_golf": 0.964, '
                '"pink_golf": 0.964, '
                '"red_ring": 0.925}',
            # Down-view gate detections are disabled until their geometry is
            # reliable; front-view gate localization remains enabled.
            'down_ignored_classes': ['gate'],
            'down_min_plane_incidence': 0.15,
            'use_rejected_front_pairs_for_multiview': True,
            # Front stereo is more reliable mainly from 0.5 to 2.5 m in this
            # simulator.  Keep all finite 3-D pairs, but inflate uncertainty
            # outside that band; multi-view intersections have no range gate.
            'front_stereo_noise_scale': 1.8,
            'front_stereo_trusted_min_range_m': 0.5,
            'front_stereo_trusted_max_range_m': 2.5,
            'front_stereo_out_of_range_noise_scale': 6.0,
            'front_stereo_trusted_range_only': False,
            'front_stereo_out_of_range_as_ray': False,
            'front_observation_pool_size': 300,
            'front_direct_queue_size': 50,
            'publish_period_sec': 0.2,
            'observation_history_size': 100,
            'front_duplicate_merge_distance_m': 0.25,
            'front_gate_min_cluster_observations': 3,
            'front_min_publish_confidence': 0.15,
            'down_observation_pool_size': 300,
            'down_direct_queue_size': 50,
            'guide_line_min_spacing_m': 0.5,
            'down_duplicate_merge_distance_m': 0.25,
        }],
        condition=IfCondition(enable_ai),
    )

    # Navigation node (optional)
    navigator = Node(
        package='uv_nav',
        executable='navigator',
        name='navigator',
        output='both',
        **python_node_kwargs,
        condition=IfCondition(enable_nav),
    )

    # Task runner (optional)
    task_runner = Node(
        package='uv_task',
        executable='task_runner',
        name='task_runner',
        output='both',
        **python_node_kwargs,
        parameters=[{'target_id': target_id}],
        condition=IfCondition(enable_task),
    )

    # The helper waits for uv_camera's MJPEG endpoint, then opens two native
    # OpenCV windows.  Start it slightly after the perception node so the
    # camera server has time to bind its port.
    annotated_preview = Node(
        package='uv_bringup',
        executable='annotated_preview',
        name='annotated_preview',
        output='both',
        **python_node_kwargs,
        arguments=[
            '--port', preview_port,
            '--width', preview_width,
            '--height', preview_height,
            '--sim-width', sim_window_width,
            '--monitor-x', str(monitor_x),
            '--monitor-y', str(monitor_y),
            '--monitor-width', str(display_width),
            '--monitor-height', str(display_height),
            '--sim-title', 'Stonefish Simulator',
            '--wait-timeout', preview_wait_timeout,
        ],
        condition=IfCondition(PythonExpression([
            "'", enable_ai, "'.lower() == 'true' and '",
            enable_preview, "'.lower() == 'true' and '",
            open_annotated_windows, "'.lower() == 'true' and '",
            stream_annotated, "'.lower() == 'true'",
        ])),
    )

    render_environment = []
    if nvidia_available:
        render_environment = [
            SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
            SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        ]

    return LaunchDescription([
        declare_enable_ai,
        declare_ai_inference_fps,
        declare_inference_threads,
        declare_camera_stitch_fps,
        declare_publish_raw_camera_topics,
        declare_enable_nav,
        declare_enable_task,
        declare_scenario,
        declare_scene_seed,
        declare_target_id,
        declare_open_annotated_windows,
        declare_stream_annotated,
        declare_sim_window_width,
        declare_sim_window_height,
        declare_render_quality,
        declare_preview_width,
        declare_preview_height,
        declare_preview_port,
        declare_preview_wait_timeout,
        declare_record_session,
        declare_enable_preview,
        declare_record_root,
        declare_record_raw_video,
        declare_record_video_mode,
        declare_record_video_format,
        declare_record_video_fps,
        declare_record_video_codec,
        declare_record_image_topics,
        declare_video_segment_seconds,
        declare_bag_segment_seconds,
        declare_record_use_sim_time,
        # 仅在检测到可用 NVIDIA GPU 时启用 NVIDIA OpenGL/PRIME 渲染。
        *render_environment,
        # Stonefish uses SDL2. SDL_VIDEO_WINDOW_POS is consumed by SDL when
        # its graphical window is created, so this also works before the
        # simulator process has a discoverable X11 window id.
        SetEnvironmentVariable(
            'SDL_VIDEO_WINDOW_POS', f'{monitor_x},{monitor_y}'),
        OpaqueFunction(function=_recording_actions),
        OpaqueFunction(function=_generate_seeded_scene),
        LogInfo(msg=['Simulation data: ', simulation_data_dir]),
        LogInfo(msg=[
            'Scenario: ',
            PathJoinSubstitution([simulation_data_dir, scenario_desc]),
        ]),
        stonefish_sim,
        sim_bridge,
        basic_motion,
        vision,
        object_localizer,
        navigator,
        task_runner,
        TimerAction(period=2.0, actions=[annotated_preview]),
    ])
