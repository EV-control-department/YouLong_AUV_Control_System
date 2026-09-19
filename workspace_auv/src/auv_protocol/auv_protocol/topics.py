"""The single source of truth for the public ``/auv`` ROS 2 namespace.

The constants in this module deliberately contain absolute names.  Nodes may
still apply a ROS namespace when a multi-vehicle deployment is introduced,
but a normal single-AUV launch must never silently fall back to an unscoped
topic.
"""

ROOT = '/auv'

# Canonical sensor topics.
FRONT_LEFT_IMAGE = f'{ROOT}/sensors/camera/front/left/image_raw'
FRONT_RIGHT_IMAGE = f'{ROOT}/sensors/camera/front/right/image_raw'
DOWN_LEFT_IMAGE = f'{ROOT}/sensors/camera/downward/left/image_raw'
DOWN_RIGHT_IMAGE = f'{ROOT}/sensors/camera/downward/right/image_raw'
FRONT_LEFT_INFO = f'{ROOT}/sensors/camera/front/left/camera_info'
FRONT_RIGHT_INFO = f'{ROOT}/sensors/camera/front/right/camera_info'
DOWN_LEFT_INFO = f'{ROOT}/sensors/camera/downward/left/camera_info'
DOWN_RIGHT_INFO = f'{ROOT}/sensors/camera/downward/right/camera_info'
FRONT_STITCHED = f'{ROOT}/sensors/camera/front/image_stitched'
DOWN_STITCHED = f'{ROOT}/sensors/camera/downward/image_stitched'
FRONT_STEREO_INFO = f'{ROOT}/sensors/camera/front/stereo_info'
DOWN_STEREO_INFO = f'{ROOT}/sensors/camera/downward/stereo_info'
IMU = f'{ROOT}/sensors/imu/data'
DVL_VELOCITY = f'{ROOT}/sensors/dvl/velocity'
DVL_ALTITUDE = f'{ROOT}/sensors/dvl/altitude'
PRESSURE = f'{ROOT}/sensors/pressure'
USBL_MEASUREMENT = f'{ROOT}/sensors/usbl/measurement'

# Perception.
_CAMERA_CHANNEL_PATHS = {
    'front_left': 'front/left',
    'front_right': 'front/right',
    'down_left': 'downward/left',
    'down_right': 'downward/right',
}


def _camera_channel_path(camera_channel):
    key = str(camera_channel).strip().lower()
    return _CAMERA_CHANNEL_PATHS.get(key, key)


def DETECTIONS(camera_channel):
    return f'{ROOT}/perception/detections/{_camera_channel_path(camera_channel)}'


def LINES(camera_channel):
    return f'{ROOT}/perception/lines/{_camera_channel_path(camera_channel)}'


ARUCO_IDS = f'{ROOT}/perception/aruco/ids'
OBJECTS = f'{ROOT}/perception/observations'
TARGETS = f'{ROOT}/perception/targets'
TARGET_OBSERVATIONS = f'{ROOT}/perception/target_observations'

# Mapping and planning contracts reserved for the research stack.
MAPPING_LANDMARKS = f'{ROOT}/mapping/landmarks'
MAPPING_KEYFRAMES = f'{ROOT}/mapping/keyframes'
MAPPING_LOCALIZATION_OPPORTUNITY = f'{ROOT}/mapping/localization_opportunity'
PLANNING_PATH = f'{ROOT}/planning/path'
PLANNING_TRAJECTORY = f'{ROOT}/planning/trajectory'
PLANNING_STATUS = f'{ROOT}/planning/status'

# State and control.
STATE_ODOM = f'{ROOT}/state/odom'
STATE_TWIST = f'{ROOT}/state/twist'
STATE_HEALTH = f'{ROOT}/state/health'
STATE_RESET = f'{ROOT}/state/reset'
TF = f'{ROOT}/tf'
TF_STATIC = f'{ROOT}/tf_static'
MOTION_COMMAND = f'{ROOT}/control/motion_command'
TRAJECTORY = f'{ROOT}/control/trajectory'
CONTROL_STATUS = f'{ROOT}/control/status'
BASIC_MOTION = f'{ROOT}/basic_motion'
CONTROL_POSE_INFO = f'{ROOT}/control/pose_info'

# Mission services/actions.
MISSION_RUN = f'{ROOT}/mission/run'
MISSION_STOP = f'{ROOT}/mission/stop'
MISSION_STATUS = f'{ROOT}/mission/status'
MISSION_EXECUTE = f'{ROOT}/mission/execute'

# Hardware adapter.
ZIT6_SETPOINT = f'{ROOT}/hardware/zit6/cmd/setpoint'
ZIT6_SERVO = f'{ROOT}/hardware/zit6/cmd/servo'
ZIT6_LIGHT = f'{ROOT}/hardware/zit6/cmd/light'
ZIT6_HEARTBEAT = f'{ROOT}/hardware/zit6/cmd/heartbeat'
ZIT6_INS = f'{ROOT}/hardware/zit6/cmd/ins'
ZIT6_STATUS = f'{ROOT}/hardware/zit6/state/status'
ZIT6_POSITION = f'{ROOT}/hardware/zit6/state/position'
ZIT6_VELOCITY = f'{ROOT}/hardware/zit6/state/velocity'
ZIT6_THRUSTER = f'{ROOT}/hardware/zit6/state/thruster'
ZIT6_HEARTBEAT_STATE = f'{ROOT}/hardware/zit6/state/heartbeat'
ZIT6_GET_PARAMS = f'{ROOT}/hardware/zit6/get_params'
ZIT6_UPDATE_PARAMS = f'{ROOT}/hardware/zit6/update_params'

# Simulation and evaluation.  These are still under /auv so they cannot be
# confused with a second, global protocol namespace.
SIM_GT_ODOM = f'{ROOT}/sim/ground_truth/odom'
SIM_GT_TWIST = f'{ROOT}/sim/ground_truth/twist'
SIM_RAW_DVL_VELOCITY = f'{ROOT}/sim/raw/dvl/velocity'
SIM_RAW_DVL_ALTITUDE = f'{ROOT}/sim/raw/dvl/altitude'
SIM_RAW_FRONT_LEFT_IMAGE = f'{ROOT}/sim/raw/camera/front/left/image_raw'
SIM_RAW_FRONT_RIGHT_IMAGE = f'{ROOT}/sim/raw/camera/front/right/image_raw'
SIM_RAW_DOWN_LEFT_IMAGE = f'{ROOT}/sim/raw/camera/downward/left/image_raw'
SIM_RAW_DOWN_RIGHT_IMAGE = f'{ROOT}/sim/raw/camera/downward/right/image_raw'
SIM_RAW_FRONT_LEFT_INFO = f'{ROOT}/sim/raw/camera/front/left/camera_info'
SIM_RAW_FRONT_RIGHT_INFO = f'{ROOT}/sim/raw/camera/front/right/camera_info'
SIM_RAW_DOWN_LEFT_INFO = f'{ROOT}/sim/raw/camera/downward/left/camera_info'
SIM_RAW_DOWN_RIGHT_INFO = f'{ROOT}/sim/raw/camera/downward/right/camera_info'
SIM_THRUSTER_COMMAND = f'{ROOT}/sim/actuators/thruster_command'
SIM_THRUSTER_STATE = f'{ROOT}/sim/actuators/thruster_state'
SIM_PERFORMANCE = f'{ROOT}/sim/performance'
SIM_CONTROL_PERFORMANCE = f'{ROOT}/sim/control_performance'
ZIT6_SIM_NAV = f'{ROOT}/hardware/zit6/sim/nav'
EVALUATION_METRICS = f'{ROOT}/evaluation/metrics'
EVALUATION_EVENTS = f'{ROOT}/evaluation/events'
SIM_DEGRADATION_EVENTS = f'{ROOT}/sim/degradation/events'
SIM_DEGRADED_DVL_VELOCITY = f'{ROOT}/sim/degraded/dvl/velocity'
SIM_DEGRADED_DVL_ALTITUDE = f'{ROOT}/sim/degraded/dvl/altitude'
SIM_DEGRADED_IMU = f'{ROOT}/sim/degraded/imu/data'
SIM_DEGRADED_USBL = f'{ROOT}/sim/degraded/usbl/measurement'
SIM_DEGRADED_FRONT_STITCHED = f'{ROOT}/sim/degraded/camera/front/image_stitched'
SIM_DEGRADED_DOWN_STITCHED = f'{ROOT}/sim/degraded/camera/downward/image_stitched'

# Temporary compatibility endpoints.  New nodes must not use these as their
# primary interface; they are intentionally kept in one place for bridges.
LEGACY_BASIC_MOTION = '/basic_motion'
LEGACY_POSE_INFO = '/basic_motion/pose_info'
LEGACY_ZIT6_SETPOINT = '/zit6/cmd/setpoint'
LEGACY_ZIT6_HEARTBEAT = '/zit6/cmd/heartbeat'
LEGACY_ZIT6_POSITION = '/zit6/state/pos'
LEGACY_ZIT6_VELOCITY = '/zit6/state/vel'
LEGACY_ZIT6_THRUSTER = '/zit6/state/thr'
LEGACY_ZIT6_HEARTBEAT_STATE = '/zit6/state/zithbt'
LEGACY_ZIT6_STATUS = '/zit6/state/status'
LEGACY_ZIT6_USBL = '/zit6/state/USBL'
LEGACY_TASK_RUN = '/task/run'
LEGACY_TASK_STOP = '/task/stop'
LEGACY_TASK_STATUS = '/task/status'
LEGACY_TASK_EXECUTE = '/task/exec'
