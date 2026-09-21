"""九宫格静态目标建图任务。

任务目标
--------
让 AUV 自主遍历池底九宫格（3x3）的九个格点，识别其中的方形/圆形锥桶，
并把每个锥桶的位置与类别发布成一张可供其他节点消费的地图。

感知方案
--------
YOLO 只负责给出分割掩膜和类别，不提供距离；距离由左右相机图像的 SGBM
视差得到。掩膜内的深度采用直方图主峰，随后用静态目标卡尔曼滤波器融合
多次测量。

执行框架（运动串行 + 感知并行）
-------------------------------
* 运动串行：主线程通过 BasicMotion 的 WTRAVEL 一次只去一个目标点，保证
  控制链路安全、可随时被 /task/stop 中断。
* 感知并行：``mapping-perception`` 后台线程持续消费图像 / 检测 / 位姿，
  因此 WTRAVEL 移动期间产生的视觉数据也能进入滤波器，不会因为「到达
  格点才开始处理」而丢掉 YOLO 推理延迟窗口内的检测结果。

坐标系
------
* 输入位姿来自 ``/basic_motion/pose_info``（PoseInfo），位于 basic_motion
  的 odom 系：START 时以当时艇位为原点，NED 约定（x=北, y=东, z=下，
  yaw 顺时针为正且单位为度）。
* 本任务所有输出统一换算到 ``mapping_odom`` 系，即 PoseInfo 所在的 odom
  系；地图里的目标位置与九宫格理论中心在同一个系里比较。

对外接口
--------
* 订阅：``/basic_motion/pose_info``、下视拼接图像、左右目检测结果、
  左右相机 CameraInfo。
* 发布：``/task/mapping/map``（低频地图快照，TRANSIENT_LOCAL，后加入的
  订阅者也能立刻拿到最新地图）、``/task/mapping/events``（逐事件调试流）。
"""

from collections import deque
from itertools import combinations
import json
import math
import threading
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from uv_camera.object_localizer import StereoCalibration
from uv_msgs.action import BasicMotion
from uv_msgs.msg import DetectionArray, PoseInfo


def _rpy_matrix(roll, pitch, yaw):
    """把角度制的 RPY 转成旋转矩阵（ZYX 顺序，即 R = Rz(yaw)Ry(pitch)Rx(roll)）。

    输入单位是度。该矩阵用于机体系 -> 世界系的向量旋转，
    例如把机体系下的相机坐标换算到 odom 系。
    """
    roll, pitch, yaw = (math.radians(float(value))
                        for value in (roll, pitch, yaw))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)


def _stamp(message):
    """统一提取消息时间戳并转成秒（float）。

    不同消息类型的时间戳位置不同（Image/PoseInfo 在 header 里，
    DetectionArray 直接是 stamp 字段），这里做兼容处理。
    """
    stamp = message.header.stamp if hasattr(message, 'header') else message.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class StaticPositionFilter:
    """静态物体模型：位置不变，过程噪声只吸收艇位和模型误差。

    三维位置卡尔曼滤波器，状态 x = [X, Y, Z]，没有速度项。
    因为目标静止，状态转移矩阵 F = I，所以预测步骤只是让协方差随时间
    膨胀（膨胀速度由 process_noise 控制），用来对付艇位估计漂移。
    状态更新使用 Joseph 形式，数值上更稳定。
    """

    def __init__(self, position, covariance, timestamp):
        self.position = np.asarray(position, dtype=float)   # 当前融合位置
        self.covariance = np.asarray(covariance, dtype=float)  # 位置协方差 P
        self.timestamp = float(timestamp)                   # 最近一次更新时刻
        self.observations = 1   # 收到的观测总数（含被拒绝的）
        self.accepted = 1       # 真正通过门限并融合的观测数

    def update(self, position, covariance, timestamp, process_noise, gate):
        """用一次新观测更新滤波器。

        参数：
            position:    观测位置（odom 系，3 维）
            covariance:  观测协方差 R（3x3）
            timestamp:   观测时间戳（秒）
            process_noise: 过程噪声谱密度，dt 越大协方差膨胀越多
            gate:        马氏距离平方门限，超过则判为离群点

        返回：
            (是否接受, 原因字符串)
        """
        timestamp = float(timestamp)
        # 时间倒退的观测直接丢弃，避免 dt 为负破坏协方差
        if timestamp <= self.timestamp:
            return False, 'out_of_order'
        # dt 上限 2s：长时间没观测时不让过程噪声把协方差放得过大
        dt = min(timestamp - self.timestamp, 2.0)
        # 预测：F = I（目标静止），只膨胀协方差
        prediction_covariance = self.covariance + np.eye(3) * process_noise * dt
        # 残差（新息）与新息协方差 S = P_pred + R
        residual = np.asarray(position, dtype=float) - self.position
        innovation = prediction_covariance + np.asarray(covariance, dtype=float)
        try:
            # 马氏距离平方 d^2 = r^T S^-1 r，用于离群点门限
            # （3 自由度卡方分布，11.345 约对应 99% 分位）
            mahalanobis = float(residual @ np.linalg.solve(innovation, residual))
        except np.linalg.LinAlgError:
            # 协方差奇异（例如观测协方差退化）时拒绝该观测
            return False, 'singular_covariance'
        if not math.isfinite(mahalanobis) or mahalanobis > gate:
            return False, f'outlier:{mahalanobis:.3f}'
        # 卡尔曼增益 K = P_pred * S^-1
        gain = prediction_covariance @ np.linalg.inv(innovation)
        # 状态更新：x = x + K * 残差
        self.position += gain @ residual
        # Joseph 形式协方差更新，对任意 K 都保持对称半正定，避免数值发散
        remainder = np.eye(3) - gain
        self.covariance = (remainder @ prediction_covariance @ remainder.T
                           + gain @ covariance @ gain.T)
        # 再强制对称化，消除浮点误差带来的微小不对称
        self.covariance = (self.covariance + self.covariance.T) / 2.0
        self.timestamp = timestamp
        self.observations += 1
        self.accepted += 1
        return True, 'accepted'


class MappingTask:
    """访问已知九宫格并输出方形/圆形锥桶的地图。

    主要参数（来自 ``config/tasks/mapping_grid.json``）：
        timeout / move_timeout / observe_seconds
            任务总时限、单次 WTRAVEL 超时、每个格点的观察时长。
        min_observations / min_confidence / class_vote_ratio / expected_cones
            确认目标所需的观测次数、检测置信度下限、类别投票占比、期望目标数。
        grid_center_x/y、grid_side_m、grid_yaw_deg、floor_z
            九宫格中心、边长、航向和池底深度，用于推算九个格点的理论中心。
        survey_z、survey_yaw_deg
            巡检高度与巡检航向（下视相机贴得越近视野越小，故抬高艇体）。
        tag_x/tag_y、tag_id、tag_dictionary、tag_timeout
            池底标记的理论位置、期望 ID（-1 表示接受任意 ID）、字典和超时。
        visit_order
            九个格点的访问顺序（默认蛇形，减少往返路程）。
        image_topic / left_detection_topic / right_detection_topic / *_info_topic
            下视拼接图、左右目检测结果和相机内参话题。
        pose_slop_s / image_slop_s / detection_slop_s
            位姿、图像、检测左右目的时间同步阈值。
        left_translation / right_translation / camera_rotation
            下视双目相机相对机体系的外参。
        sgbm_* / min_disparity / min_depth_m / max_depth_m / depth_bin_m /
        depth_peak_ratio / min_depth_points
            SGBM 与掩膜深度主峰的参数。
        measurement_sigma_m / process_noise / mahalanobis_gate / cell_gate_m
            观测噪声、过程噪声、离群点门限和目标-格点关联距离门限。
    """

    def __init__(self, node, params):
        self.node = node
        self.params = params
        # 后台感知线程与 ROS 回调会并发访问滤波器/计数，统一用可重入锁保护
        self.lock = threading.RLock()
        self.bridge = CvBridge()
        self.subscriptions = []
        self.pose_history = deque(maxlen=300)        # 最近位姿，用于按时间戳查找
        self.image_history = deque(maxlen=12)        # 最近拼接图（已切成左右目）
        self.left_detections = deque(maxlen=24)      # 左目检测结果队列
        self.right_detections = deque(maxlen=24)     # 右目检测结果队列
        self.camera_info = {}                        # {'left': CameraInfo, 'right': ...}
        self.calibration = None                      # 双目标定（含基线）
        self.rectify_maps = None                     # 预计算去畸变+校正映射
        self.filters = {}                            # 格点索引 -> StaticPositionFilter
        self.class_votes = {}                        # 格点索引 -> [方形票, 圆形票]
        self.visit_order = []                        # 实际访问过的格点顺序
        self.traversal_order = []                    # 建图后成功遍历的锥桶（独立于巡检）
        self.observation_failures = []               # 没拿到足够观测的格点
        self.cell_observations = {}                  # 格点索引 -> 观测计数统计
        self.current_cell = None                     # 当前正在观察的格点
        self.state = 'initializing'                  # 任务状态机，写入事件流
        self.last_detection_stamp = -1.0             # 已消费的最新检测时间戳
        self.perception_stop = threading.Event()     # 通知后台线程退出
        self.perception_thread = None
        self._next_perception_error_log = 0.0
        self.perception_stats = {                    # 感知统计，便于调参诊断
            'detection_pairs': 0,
            'image_unavailable': 0,
            'pose_unavailable': 0,
            'processed_pairs': 0,
        }
        self.tag_scan_stats = {'frames': 0, 'markers': 0, 'wrong_id': 0,
                               'no_depth': 0}
        self.last_tag_reason = 'not_scanned'         # 最近一次标记识别失败原因
        # 任务总时限：所有等待循环都会用它作为硬边界
        self.deadline = time.monotonic() + float(params['timeout'])

        # 九宫格几何：中心 + 航向旋转，再据此推算九个理论格点中心
        self.grid_center = np.array([
            float(params['grid_center_x']), float(params['grid_center_y']),
            float(params['floor_z']),
        ])
        self.grid_rotation = _rpy_matrix(0, 0, params['grid_yaw_deg'])
        self.grid_centers = self._make_grid_centers()
        # Keep raw world-space measurements for DDS visualization and offline
        # parameter tuning. The filtered track remains task-authoritative.
        # 保留原始世界系观测点：仅用于 DDS 可视化和离线调参，
        # 任务判定始终以滤波后的轨迹为准。
        self.measurement_points = {
            index: deque(maxlen=120) for index in self.grid_centers
        }
        # 下视相机外参：相机在机体系下的平移 + 光学系到机体系的旋转
        self.camera_translation = np.asarray(params['left_translation'], dtype=float)
        self.right_translation = np.asarray(params['right_translation'], dtype=float)
        self.camera_rotation = np.asarray(
            params['camera_rotation'], dtype=float).reshape(3, 3)

        # 地图用 TRANSIENT_LOCAL（latched），后启动的可视化工具也能拿到最新一帧
        qos_map = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = node.create_publisher(String, '/task/mapping/map', qos_map)
        self.event_pub = node.create_publisher(String, '/task/mapping/events', 100)
        self._subscribe(PoseInfo, '/basic_motion/pose_info', self._pose_cb)
        self._subscribe(Image, params['image_topic'], self._image_cb)
        self._subscribe(DetectionArray, params['left_detection_topic'],
                        self._left_detection_cb)
        self._subscribe(DetectionArray, params['right_detection_topic'],
                        self._right_detection_cb)
        self._subscribe(CameraInfo, params['left_info_topic'],
                        lambda message: self._info_cb('left', message))
        self._subscribe(CameraInfo, params['right_info_topic'],
                        lambda message: self._info_cb('right', message))
        # 1Hz 定时发布地图快照，保证外部始终能看到进展
        self.publish_timer = node.create_timer(1.0, self.publish_map)

        # ArUco/AprilTag 检测器：先做环境自检，再兼容 OpenCV 新旧两套 API
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('OpenCV ArUco module is required for the tag trigger')
        dictionary_id = getattr(cv2.aruco, params['tag_dictionary'], None)
        if dictionary_id is None:
            raise ValueError(f'unknown ArUco dictionary: {params["tag_dictionary"]}')
        self.tag_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        # OpenCV 4.7+ 把标记检测迁移到 ArucoDetector；不同 Ubuntu/ROS 镜像
        # 可能只有其中一套 API，这里两套都支持且不改变对外接口。
        self.tag_detector = None
        detector_type = getattr(cv2.aruco, 'ArucoDetector', None)
        if detector_type is not None:
            parameters_type = getattr(cv2.aruco, 'DetectorParameters', None)
            parameters = parameters_type() if parameters_type is not None else None
            self.tag_detector = (detector_type(self.tag_dictionary, parameters)
                                 if parameters is not None else
                                 detector_type(self.tag_dictionary))
        # (字典名, 检测器) 列表，便于后续扩展多字典并行尝试
        self.tag_detectors = [(params['tag_dictionary'], self.tag_detector)]

    def _subscribe(self, message_type, topic, callback):
        """创建订阅并登记，destroy() 时统一释放。"""
        self.subscriptions.append(self.node.create_subscription(
            message_type, topic, callback, qos_profile_sensor_data))

    def _make_grid_centers(self):
        """推算九个格点的理论中心。

        格点索引 index = row * 3 + column，row/column 都以九宫格中心为原点：
        格间距 = 边长 / 3，row 0 在最北（x 最大），column 0 在最西（y 最小）。
        结果在世界系（odom），是目标关联时的基准点。
        """
        spacing = float(self.params['grid_side_m']) / 3.0
        return {
            row * 3 + column: self.grid_center + self.grid_rotation @ np.array([
                (1 - row) * spacing, (column - 1) * spacing, 0.0])
            for row in range(3) for column in range(3)
        }

    def _pose_cb(self, message):
        """缓存位姿，供后台线程按图像时间戳就近查找。"""
        with self.lock:
            self.pose_history.append(message)

    def _image_cb(self, message):
        """接收下视「左右并排拼接图」，在中间切成左目和右目两幅图后缓存。"""
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            width = image.shape[1] // 2
            if width < 2:
                return
            with self.lock:
                # 复制一份：cv_bridge 返回的数组底层缓冲可能被复用
                self.image_history.append((_stamp(message),
                                           image[:, :width].copy(),
                                           image[:, width:].copy()))
        except (cv2.error, ValueError) as error:
            self.node.get_logger().warning(f'mapping image rejected: {error}')

    def _left_detection_cb(self, message):
        """缓存左目检测结果。"""
        with self.lock:
            self.left_detections.append(message)

    def _right_detection_cb(self, message):
        """缓存右目检测结果。"""
        with self.lock:
            self.right_detections.append(message)

    def _info_cb(self, side, message):
        """缓存相机内参，左右都到齐后才能建立双目标定。"""
        with self.lock:
            self.camera_info[side] = message

    def _emit(self, event, **values):
        """发布一条结构化事件（JSON），用于可视化与问题定位。

        事件带 schema_version/state/cell/stamp 公共字段，其余字段由调用方补充。
        对「帧被拒绝 / 测量被拒绝」这类高频事件做 3 秒限流日志，避免刷屏。
        """
        payload = {
            'schema_version': 1,
            'event': event,
            'state': self.state,
            'cell': self.current_cell,
            'stamp': self.node.get_clock().now().nanoseconds * 1e-9,
            **values,
        }
        self.event_pub.publish(String(data=json.dumps(payload, allow_nan=False)))
        if event in ('frame_rejected', 'measurement_rejected'):
            now = time.monotonic()
            if now >= getattr(self, '_next_rejection_log', 0.0):
                self.node.get_logger().warning(
                    f'建图数据被拒绝：格点={self.current_cell}，原因={values.get("reason")}')
                self._next_rejection_log = now + 3.0

    def publish_map(self):
        """组装并发布完整地图快照（由 1Hz 定时器和关键节点主动调用）。

        每个格点输出：理论中心、是否访问过、类别标签（按投票占比判定）、
        滤波位置与协方差、观测次数、原始观测点、以及滤波位置相对理论中心的
        残差（残差过大说明格点关联可能有误，便于人工核对）。
        """
        with self.lock:
            cells = []
            for index, center in self.grid_centers.items():
                track = self.filters.get(index)
                votes = self.class_votes.get(index, [0, 0])
                total = sum(votes)
                label = None
                # 只有某一类票数占比达到 class_vote_ratio 才给出类别，
                # 否则保持未知（label=None），避免误报。
                if total and max(votes) / total >= float(self.params['class_vote_ratio']):
                    label = 'square_cone' if votes[0] >= votes[1] else 'round_cone'
                if hasattr(self, 'final_assignment'):
                    chosen = self.final_assignment.get(index)
                    label = ('square_cone' if chosen == 0 else
                             'round_cone' if chosen == 1 else None)
                cells.append({
                    'id': index,
                    'row': index // 3,
                    'column': index % 3,
                    'center': center.tolist(),          # 理论格点中心
                    'visited': index in self.visit_order,
                    'label': label,
                    'class_id': (getattr(self, 'final_assignment', {}).get(index)
                                 if hasattr(self, 'final_assignment') else
                                 int(np.argmax(votes)) if label else None),
                    'position': track.position.tolist() if track else None,
                    'covariance': track.covariance.tolist() if track else None,
                    'observations': track.observations if track else 0,
                    'accepted_observations': track.accepted if track else 0,
                    'votes': votes,
                    'observation': self.cell_observations.get(index),
                    'residual': (track.position - center).tolist() if track else None,
                    'measurements': list(self.measurement_points.get(index, ())),
                })
            payload = {
                'schema_version': 2,
                'final_assignment': getattr(self, 'final_assignment', None),
                'frame_id': 'mapping_odom',
                'state': self.state,
                'stamp': self.node.get_clock().now().nanoseconds * 1e-9,
                'grid': {
                    'center': self.grid_center.tolist(),
                    'side_m': float(self.params['grid_side_m']),
                    'yaw_deg': float(self.params['grid_yaw_deg']),
                    'floor_z': float(self.params['floor_z']),
                },
                'tag': self._tag_json(),
                'cells': cells,
                'visit_order': list(self.visit_order),
                'traversal_order': list(self.traversal_order),
                'traversal_path': getattr(self, 'traversal_path', []),
                'observation_failures': list(self.observation_failures),
                'all_cells_visited': len(self.visit_order) == len(self.grid_centers),
                'measurement_count': sum(
                    len(points) for points in self.measurement_points.values()),
            }
        self.map_pub.publish(String(data=json.dumps(payload, allow_nan=False)))

    def _tag_json(self):
        """把池底标记的融合结果序列化成地图里的一段。"""
        if not hasattr(self, 'tag_filter') or self.tag_filter is None:
            return None
        return {
            'id': self.tag_id,
            'position': self.tag_filter.position.tolist(),
            'covariance': self.tag_filter.covariance.tolist(),
            'observations': self.tag_filter.observations,
        }

    def _ready(self):
        """任务是否应继续：ROS 正常 + 未被停止 + 未超过总时限。"""
        return rclpy.ok() and not self.node.stopped and time.monotonic() < self.deadline

    def _pose_for(self, timestamp):
        """取与给定（图像）时间戳最接近的位姿。

        时间门限分两级：严格阈值 pose_slop_s；在 reading_tag/observe_cell
        阶段艇是静止的，允许放宽到 5s（仿真渲染/发布频率低时时间戳常常对
        不齐），其他阶段放宽到 1s。超过放宽后的门限直接抛错，由调用方把
        该帧记为拒绝，而不是用错误的位姿去算世界坐标。
        """
        with self.lock:
            poses = list(self.pose_history)
        if not poses:
            raise ValueError('pose is unavailable')
        pose = min(poses, key=lambda item: abs(_stamp(item) - timestamp))
        delta = abs(_stamp(pose) - timestamp)
        strict_slop = float(self.params['pose_slop_s'])
        fallback_slop = max(
            strict_slop,
            5.0 if self.state in ('reading_tag', 'observe_cell') else 1.0)
        if delta > fallback_slop:
            raise ValueError('no pose close enough to image timestamp')
        # 使用了放宽门限时给出限流告警，提醒现场时间同步可能有问题
        if (delta > strict_slop
                and self.state in ('reading_tag', 'observe_cell')):
            now = time.monotonic()
            if now >= getattr(self, '_next_pose_fallback_log', 0.0):
                self.node.get_logger().warning(
                    f'位姿时间戳未严格同步，使用最近位姿：时间差={delta:.3f}s，'
                    f'严格阈值={strict_slop:.3f}s')
                self._next_pose_fallback_log = now + 3.0
        return pose

    def _prepare_calibration(self):
        """用左右相机内参和外参建立双目标定，并预计算校正映射与 SGBM。

        只需要做一次：之后每帧图像仅需 remap 即可得到校正图。
        返回 True 表示标定就绪。
        """
        with self.lock:
            infos = dict(self.camera_info)
        if set(infos) != {'left', 'right'}:
            return False
        # 由 CameraInfo + 机体系外参构造标定（同时算出基线 baseline_m）
        self.calibration = StereoCalibration.from_camera_info(
            'down_mapping', infos['left'], infos['right'],
            self.camera_translation, self.camera_rotation,
            self.right_translation, self.camera_rotation)
        size = (int(infos['left'].width), int(infos['left'].height))
        # 去畸变 + 立体校正的查找表，左右各一份（remap 时复用）
        left_map = cv2.initUndistortRectifyMap(
            self.calibration.camera_matrix_left, self.calibration.dist_left,
            self.calibration.rectification_left,
            self.calibration.projection_left[:, :3], size, cv2.CV_32FC1)
        right_map = cv2.initUndistortRectifyMap(
            self.calibration.camera_matrix_right, self.calibration.dist_right,
            self.calibration.rectification_right,
            self.calibration.projection_right[:, :3], size, cv2.CV_32FC1)
        self.rectify_maps = (left_map, right_map)
        # SGBM 半全局匹配：P1/P2 取经典的 8*bs^2 / 32*bs^2 经验值，
        # SGBM_3WAY 模式质量更好；speckle 过滤用于去掉水面反光造成的斑点噪声。
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=int(self.params['sgbm_min_disparity']),
            numDisparities=int(self.params['sgbm_num_disparities']),
            blockSize=int(self.params['sgbm_block_size']),
            P1=8 * 1 * int(self.params['sgbm_block_size']) ** 2,
            P2=32 * 1 * int(self.params['sgbm_block_size']) ** 2,
            disp12MaxDiff=1, uniquenessRatio=8,
            speckleWindowSize=80, speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        return True

    def _image_for(self, timestamp):
        """取与给定时间戳最接近的拼接帧，并返回校正后的左、右目图像。

        时间门限与 _pose_for 一致（格点观察/读标记阶段艇静止，可放宽到 5s）。
        超出容忍范围返回 None，调用方据此拒绝该帧而不是用错配图像。
        """
        with self.lock:
            if not self.image_history:
                return None
            candidate = min(self.image_history,
                            key=lambda item: abs(item[0] - timestamp))
        delta = abs(candidate[0] - timestamp)
        strict_slop = float(self.params['image_slop_s'])
        # Detection and stitched-image messages are produced by different
        # callbacks.  At a low simulation/render rate, the nearest image can
        # arrive later than the strict synchronization window.  The vehicle
        # is stationary during cell observation, so a bounded fallback is
        # valid and prevents a detection from being discarded unnecessarily.
        # 检测消息和拼接图来自不同回调：低渲染频率下最近帧可能晚于严格同步
        # 窗口才到达。格点观察期间艇是静止的，因此有限度地放宽是合理的，
        # 可避免真实检测被无谓丢弃。
        fallback_slop = max(
            strict_slop,
            5.0 if self.state in ('reading_tag', 'observe_cell') else 1.0)
        if delta > fallback_slop:
            return None
        if delta > strict_slop:
            now = time.monotonic()
            if now >= getattr(self, '_next_image_fallback_log', 0.0):
                self.node.get_logger().warning(
                    f'图像时间戳未严格同步，使用最近帧：时间差={delta:.3f}s，'
                    f'严格阈值={strict_slop:.3f}s')
                self._next_image_fallback_log = now + 3.0
        left, right = candidate[1], candidate[2]
        # 校正后的图像像素行对齐，SGBM 才能正确匹配
        left = cv2.remap(left, *self.rectify_maps[0], cv2.INTER_LINEAR)
        right = cv2.remap(right, *self.rectify_maps[1], cv2.INTER_LINEAR)
        return left, right

    def _detection_pair(self):
        """找出最新的一对可用左右目检测结果。

        只处理比 last_detection_stamp 更新的左目消息，避免同一帧被反复消费。
        左右关联优先用 stereo_pair_id（仿真里左右目 header 可能差约 100ms，
        pair id 才是权威关联）；老发布者没有该字段时退回时间戳差门限。
        返回 (左目消息, 右目消息, 时间戳) 或 None。
        """
        with self.lock:
            left_messages = list(self.left_detections)
            right_messages = list(self.right_detections)
        for left_message in reversed(left_messages):
            timestamp = _stamp(left_message)
            if timestamp <= self.last_detection_stamp:
                continue
            candidates = []
            for right_message in right_messages:
                left_id = int(getattr(left_message, 'stereo_pair_id', 0) or 0)
                right_id = int(getattr(right_message, 'stereo_pair_id', 0) or 0)
                # The simulator carries the same pair ID even when the two
                # camera headers differ by roughly 100 ms.  Pair ID is the
                # authoritative association; timestamp is the fallback for
                # legacy publishers that do not provide it.
                # 仿真器左右目 header 时间可能差约 100ms，但 pair id 相同；
                # 因此 pair id 是权威关联，时间戳只是无 pair id 时的兜底。
                if left_id and right_id:
                    if left_id != right_id:
                        continue
                elif (abs(_stamp(right_message) - timestamp)
                      > float(self.params['detection_slop_s'])):
                    continue
                candidates.append(right_message)
            if candidates:
                # 多个候选时取时间最接近的一个
                right_message = min(candidates,
                                    key=lambda item: abs(_stamp(item) - timestamp))
                return left_message, right_message, timestamp
        return None

    def _record_observation_frame(self):
        """记录当前格点收到了一帧可用于定位的同步感知数据。

        只统计「停留在格点观察阶段」的帧，用于判断该格点是否真的获得过
        可用感知输入；移动期间的有效测量仍会进入滤波器，但不算作格点观测帧。
        """
        if self.state != 'observe_cell' or self.current_cell is None:
            return
        with self.lock:
            observation = self.cell_observations.setdefault(
                self.current_cell,
                {'synchronized_frames': 0, 'valid_measurements': 0})
            observation['synchronized_frames'] += 1

    def _record_observation_measurement(self, cell):
        """把本次有效深度测量记入其理论格点。"""
        with self.lock:
            observation = self.cell_observations.setdefault(
                int(cell),
                {'synchronized_frames': 0, 'valid_measurements': 0})
            observation['valid_measurements'] += 1

    def _depth_map(self, left, right):
        """对校正后的左右目图像做 SGBM，返回 (视差图, 深度图)。

        SGBM 输出的是 16 倍定点视差，需除以 16 还原为像素视差；
        深度按 Z = fx * B / d 计算（fx 取左目投影矩阵，B 为标定基线）。
        视差过小（<= min_disparity）视为无效，填 NaN 以便后续过滤。
        """
        left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        disparity = self.sgbm.compute(left_gray, right_gray).astype(np.float32) / 16.0
        depth = np.full(disparity.shape, np.nan, dtype=np.float32)
        valid = disparity > float(self.params['min_disparity'])
        depth[valid] = (self.calibration.projection_left[0, 0]
                        * self.calibration.baseline_m / disparity[valid])
        return disparity, depth

    def _mask_depth_mode(self, detection, depth):
        """在分割掩膜内取深度直方图主峰，得到该目标的一次距离测量。

        步骤：
          1. 用检测消息里的 mask_x/mask_y 多边形填充出掩膜；
          2. 取掩膜内深度，过滤 NaN 与 [min_depth_m, max_depth_m] 之外的值；
          3. 有效点太少 -> 放弃（返回 None）；
          4. 按 depth_bin_m 分桶做直方图，取最高峰所在桶；
          5. 主峰点数需同时满足 min_depth_points 和 depth_peak_ratio 占比，
             否则说明掩膜内深度不成峰（多半混入了背景/水面），放弃；
          6. 返回 (主峰深度中值, 掩膜像素质心 (x, y), 参与统计的点数)。

        取中值而非均值，可以进一步抑制桶内残留的少量离群点。
        """
        xs = np.asarray(getattr(detection, 'mask_x', []), dtype=float)
        ys = np.asarray(getattr(detection, 'mask_y', []), dtype=float)
        if len(xs) < 3 or len(xs) != len(ys):
            return None
        polygon = np.column_stack((xs, ys)).astype(np.int32)
        mask = np.zeros(depth.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [polygon], 1)
        values = depth[mask.astype(bool)]
        values = values[np.isfinite(values)]
        values = values[(values >= float(self.params['min_depth_m']))
                        & (values <= float(self.params['max_depth_m']))]
        if len(values) < int(self.params['min_depth_points']):
            return None
        bin_size = float(self.params['depth_bin_m'])
        bins = np.arange(float(self.params['min_depth_m']),
                         float(self.params['max_depth_m']) + bin_size,
                         bin_size)
        counts, edges = np.histogram(values, bins=bins)
        peak = int(np.argmax(counts))
        selected = values[(values >= edges[peak]) & (values < edges[peak + 1])]
        if len(selected) < max(int(self.params['min_depth_points']),
                               int(len(values) * self.params['depth_peak_ratio'])):
            return None
        # 像素质心：np.where 返回 (行, 列)，交换后得到常见的 (x, y)
        center = np.median(np.column_stack(np.where(mask > 0)), axis=0)
        return float(np.median(selected)), (float(center[1]), float(center[0])), int(len(selected))

    def _world_measurement(self, pixel, depth, pose, rectified=False):
        """把「像素 + 深度」换算成 odom 系下的三维点。

        坐标链：像素 -> 校正后相机坐标 -> 原始左目光学坐标 -> 机体系 -> odom 系。
          * 像素反投影：X=(u-cx)Z/fx, Y=(v-cy)Z/fy, Z=depth（Z 即深度）；
          * 乘 R1^T 把校正后坐标还原回原始左相机光学系；
          * 用下视相机外参把光学系换算到机体系；
          * 用 PoseInfo 的 RPY 构造机体系->世界旋转，再加艇位得到 odom 点。

        rectified=True 表示传入像素已经来自校正图（AprilTag 分支），
        无需再做一次 undistortPoints。
        """
        rectified_pixel = (np.asarray(pixel, dtype=float) if rectified else
                           self.calibration.rectified_pixel(
                               'left', np.asarray(pixel, dtype=float)))
        fx = self.calibration.projection_left[0, 0]
        fy = self.calibration.projection_left[1, 1]
        cx = self.calibration.projection_left[0, 2]
        cy = self.calibration.projection_left[1, 2]
        point_rectified = np.array([
            (rectified_pixel[0] - cx) * depth / fx,
            (rectified_pixel[1] - cy) * depth / fy,
            depth,
        ])
        point_optical = self.calibration.rectification_left.T @ point_rectified
        # 机体系->世界（odom）旋转换矩阵
        body_rotation = _rpy_matrix(pose.robot_roll, pose.robot_pitch, pose.robot_yaw)
        body_position = np.array([pose.robot_x, pose.robot_y, pose.robot_z])
        return body_position + body_rotation @ (
            self.camera_translation + self.camera_rotation @ point_optical)

    def _process_cone_pair(self, left_message, right_message, timestamp,
                           image_pair, pose):
        """处理一对已经完成时间配对的左右检测。

        对每个左目目标：
          1. 整幅 SGBM 深度图只算一次，左右目共用；
          2. 类别只接受 0/1，且要求右目也存在同类检测（左右一致性初筛）；
          3. 掩膜深度主峰 -> 像素 + 距离；
          4. 换算到世界系，按 xy 距离关联到最近的「理论格点」；
          5. 距理论格点超过 cell_gate_m 则拒绝（防止关联到错误格子）；
          6. 通过门限后做静态位置滤波，并给该格点的类别累计一票。

        所有测量（含被拒绝的）都会存入 measurement_points，
        供可视化与离线调参；任务判定只使用通过门限的滤波结果。
        返回本对检测产生的测量数量。
        """
        _, depth = self._depth_map(*image_pair)
        self._record_observation_frame()
        right_detections = [d for d in right_message.detections
                            if int(d.class_id) in (0, 1)]
        measurements = 0
        for left_detection in left_message.detections:
            class_id = int(left_detection.class_id)
            # 只关心方形/圆形锥桶，且置信度要达标
            if (class_id not in (0, 1)
                    or float(left_detection.confidence)
                    < float(self.params['min_confidence'])):
                continue
            # 右目必须也检出同一类，作为廉价的误检过滤
            if not any(int(item.class_id) == class_id for item in right_detections):
                continue
            result = self._mask_depth_mode(left_detection, depth)
            if result is None:
                self._emit('measurement_rejected', reason='no_depth_mode', class_id=class_id,
                           measurement_stamp=timestamp)
                continue
            distance, pixel, sample_count = result
            point = self._world_measurement(pixel, distance, pose)
            # 目标关联：按 xy 平面距离选最近的格点（z 不参与，锥桶底都在池底）
            cell = min(self.grid_centers,
                       key=lambda index: np.linalg.norm(
                           point[:2] - self.grid_centers[index][:2]))
            residual = float(np.linalg.norm(point[:2] - self.grid_centers[cell][:2]))
            point_record = {
                'position': point.tolist(),
                'class_id': class_id,
                'confidence': float(left_detection.confidence),
                'depth_m': distance,
                'residual_m': residual,
                'accepted': False,
                'timestamp': float(timestamp),
            }
            # 先无条件记录原始观测点，便于事后分析被拒绝的原因
            with self.lock:
                self.measurement_points[cell].append(point_record)
            if residual > float(self.params['cell_gate_m']):
                self._emit('measurement_rejected', reason='outside_cell_gate', class_id=class_id,
                           position=point.tolist(), residual_m=residual,
                           measurement_stamp=timestamp)
                continue
            # 观测噪声：水平 sigma，深度方向误差更大，故 z 方差放大 2 倍
            covariance = np.eye(3) * float(self.params['measurement_sigma_m']) ** 2
            covariance[2, 2] *= 2.0
            with self.lock:
                track = self.filters.get(cell)
                if track is None:
                    # 该格点首次观测：直接以本次测量初始化滤波器
                    self.filters[cell] = StaticPositionFilter(point, covariance, timestamp)
                    self.class_votes[cell] = [0, 0]
                    accepted, reason = True, 'initial'
                else:
                    accepted, reason = track.update(
                        point, covariance, timestamp,
                        float(self.params['process_noise']),
                        float(self.params['mahalanobis_gate']))
                    # 连续的独立帧形成更强的新簇时允许纠错，单个离群点不能重置地图。
                    if not accepted and reason.startswith('outlier:'):
                        recent = list(self.measurement_points[cell])[-12:]
                        cluster = {}
                        for sample in recent:
                            delta = np.asarray(sample['position']) - point
                            if (sample['residual_m'] <= float(self.params['cell_gate_m'])
                                    and float(delta @ np.linalg.solve(2 * covariance, delta))
                                    <= float(self.params['mahalanobis_gate'])):
                                cluster[sample['timestamp']] = sample
                        required = max(3, min(8, track.accepted + 1))
                        if len(cluster) >= required and len(cluster) > len(recent) / 2:
                            samples = list(cluster.values())
                            center = np.median([s['position'] for s in samples], axis=0)
                            track = StaticPositionFilter(center, covariance.copy(), timestamp)
                            track.accepted = len(samples)
                            self.filters[cell] = track
                            self.class_votes[cell] = [0, 0]
                            for sample in self.measurement_points[cell]:
                                sample['accepted'] = False
                            for sample in samples:
                                sample['accepted'] = True
                                self.class_votes[cell][sample['class_id']] += 1
                            # 当前帧已包含在重建投票中。
                            self.class_votes[cell][class_id] -= 1
                            accepted, reason = True, 'consistent_cluster_reinitialized'
                            self.node.get_logger().warning(
                                f'格点{cell}位置纠错：使用{len(samples)}个独立帧的一致簇重建滤波与类别投票')
                # 只有被滤波器接受的观测才计入类别投票，避免离群点污染类别
                if accepted:
                    self.class_votes[cell][class_id] += 1
                    point_record['accepted'] = True
            self._record_observation_measurement(cell)
            self._emit('cone_measurement', cell=cell, class_id=class_id,
                       confidence=float(left_detection.confidence), depth_m=distance,
                       depth_samples=sample_count, position=point.tolist(),
                       residual_m=residual, accepted=accepted, reason=reason,
                       measurement_stamp=timestamp)
            measurements += 1
        return measurements

    def _observe_cones(self):
        """等待后台感知线程累计当前格点的观测，不再主动取帧。

        主线程只负责「等够 observe_seconds」并对比进入前后的计数增量；
        返回本格点新增同步帧数是否达到 min_observations。
        注意：没有观测不等于任务失败（空格本来就该没有锥桶），
        由 execute() 记录为空格并继续下一格。
        """
        cell = self.current_cell
        with self.lock:
            # 进入观察阶段前的计数快照，用于计算增量
            start = dict(self.cell_observations.get(
                cell, {'synchronized_frames': 0, 'valid_measurements': 0}))
        end = min(self.deadline, time.monotonic() + float(self.params['observe_seconds']))
        while self._ready() and time.monotonic() < end:
            time.sleep(0.05)
        with self.lock:
            current = dict(self.cell_observations.get(cell, start))
        observed_frames = current['synchronized_frames'] - start['synchronized_frames']
        measurements = current['valid_measurements'] - start['valid_measurements']
        self.node.get_logger().info(
            f'格点{cell}观察完成：后台同步帧={observed_frames}，'
            f'新增有效目标测量={measurements}，累计有效目标测量={current["valid_measurements"]}；'
            f'感知统计={self.perception_stats}')
        return observed_frames >= int(self.params['min_observations'])

    def _tag_measurement(self, image_pair, pose):
        """在左目图上尝试识别池底标记，成功则返回 (id, 世界坐标, 深度点数)。

        为了提高解码成功率：
          * 同时尝试原灰度图和 CLAHE 对比度增强图（水下光照/渲染对比度低）；
          * 遍历所有 (字典, 检测器) 组合，并兼容 OpenCV 新旧两套检测接口；
          * 用掩膜深度主峰测距，再换算到世界系（像素已在校正图上，
            因此 rectified=True）。
        失败时把具体原因写进 last_tag_reason（no_marker / wrong_id / no_depth_mode
        / 具体异常），供 _read_tag() 超时时输出诊断信息。
        """
        left, _ = image_pair
        gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        self.tag_scan_stats['frames'] += 1
        self.last_tag_reason = '未发现可解码标记'
        # Rendering, water contrast and the stitched image can make the
        # native detector sensitive to scale.  Keep the original image and a
        # contrast-normalized variant; do not use annotated frames here.
        # 渲染、水体对比度和拼接都会影响检测器的尺度敏感性。这里保留原图并
        # 额外尝试对比度归一化版本；注意不要用带标注框的图像，否则会干扰解码。
        variants = (gray, cv2.createCLAHE(clipLimit=2.0,
                                          tileGridSize=(8, 8)).apply(gray))
        for family, detector in self.tag_detectors:
            for variant in variants:
                corners, ids, _ = (detector.detectMarkers(variant) if detector is not None
                                   else cv2.aruco.detectMarkers(variant, self.tag_dictionary))
                if ids is None:
                    continue
                self.tag_scan_stats['markers'] += len(ids)
                for polygon, tag_id in zip(corners, ids.flatten()):
                    tag_id = int(tag_id)
                    # tag_id < 0 表示接受任意 ID（随机场景）；否则必须是期望 ID
                    if int(self.params['tag_id']) >= 0 and tag_id != int(self.params['tag_id']):
                        self.tag_scan_stats['wrong_id'] += 1
                        self.last_tag_reason = f'{family}:wrong_id:{tag_id}'
                        continue
                    # 用一个轻量匿名对象承载标记四角，复用锥桶的掩膜深度逻辑
                    detection = type('TagMask', (), {
                        'mask_x': polygon.reshape(-1, 2)[:, 0].tolist(),
                        'mask_y': polygon.reshape(-1, 2)[:, 1].tolist(),
                    })()
                    result = self._mask_depth_mode(
                        detection, self._depth_map(*image_pair)[1])
                    if result is None:
                        self.tag_scan_stats['no_depth'] += 1
                        self.last_tag_reason = f'{family}:no_depth_mode'
                        continue
                    distance, pixel, sample_count = result
                    self.last_tag_reason = f'{family}:accepted'
                    return (tag_id, self._world_measurement(pixel, distance, pose, rectified=True),
                            sample_count)
        if self.tag_scan_stats['markers'] == 0:
            self.last_tag_reason = 'no_marker'
        return None

    def _accept_tag_result(self, result, timestamp):
        """将后台线程得到的 AprilTag 测量写入静态滤波器。

        只融合同一个标记 ID 的测量：ID 变化说明识别到了别的标记，
        直接拒绝合并，避免把不同目标混进同一条轨迹。
        返回该测量是否被滤波器接受。
        """
        if result is None:
            return False
        tag_id, point, sample_count = result
        with self.lock:
            if hasattr(self, 'tag_filter') and tag_id != self.tag_id:
                self.last_tag_reason = '标记ID改变，拒绝合并不同目标'
                return False
            covariance = np.eye(3) * float(self.params['measurement_sigma_m']) ** 2
            if not hasattr(self, 'tag_filter'):
                self.tag_id = tag_id
                self.tag_filter = StaticPositionFilter(point, covariance, timestamp)
                accepted, reason = True, 'initial'
            else:
                accepted, reason = self.tag_filter.update(
                    point, covariance, timestamp,
                    float(self.params['process_noise']),
                    float(self.params['mahalanobis_gate']))
            observations = self.tag_filter.observations
        self._emit('tag_measurement', tag_id=tag_id, position=point.tolist(),
                   depth_samples=sample_count, accepted=accepted, reason=reason,
                   measurement_stamp=timestamp)
        if observations >= int(self.params['min_observations']):
            self.node.get_logger().info(
                f'标记确认成功：ID={tag_id}，有效观测={observations}')
        return accepted

    def _perception_loop(self):
        """后台持续处理最新图像、AprilTag 和左右目分割检测。

        与主线程的 WTRAVEL 并行运行：
          * 已处理过的图像时间戳不再重复处理（last_image_processed 游标）；
          * 图像若暂时无法时间配对（_image_for 返回 None），本轮不消费，
            留待下次回调补齐数据后再处理；
          * 每轮都尝试取一对新的左右目检测做锥桶测量；
          * 整轮无事可做才休眠 30ms，用 Event.wait 保证能立刻响应退出。
        """
        last_image_processed = -1.0
        self.node.get_logger().info('建图后台感知线程已启动：视觉与WTRAVEL并行运行')
        while self._ready() and not self.perception_stop.is_set():
            did_work = False
            with self.lock:
                images = list(self.image_history)
            # 按时间升序处理，保证滤波器收到的时间戳单调递增
            for timestamp, _, _ in sorted(images, key=lambda item: item[0]):
                if timestamp <= last_image_processed:
                    continue
                try:
                    pose = self._pose_for(timestamp)
                    image_pair = self._image_for(timestamp)
                except (ValueError, cv2.error) as error:
                    self.last_tag_reason = f'图像或位姿无效：{error}'
                    continue
                if image_pair is None:
                    # 图像时间配对可能在下一次回调才成立，暂不消费该帧。
                    continue
                last_image_processed = timestamp
                # 标记识别失败不影响任务：真正的判定在 _read_tag() 里做
                try:
                    self._accept_tag_result(
                        self._tag_measurement(image_pair, pose), timestamp)
                except (ValueError, cv2.error) as error:
                    self.last_tag_reason = f'标记处理失败：{error}'
                did_work = True

            pair = self._detection_pair()
            if pair is not None:
                left_message, right_message, timestamp = pair
                if timestamp != getattr(self, '_pending_detection_stamp', None):
                    self._pending_detection_stamp = timestamp
                    self._pending_detection_since = time.monotonic()
                    self.perception_stats['detection_pairs'] += 1
                image_pair = self._image_for(timestamp)
                if image_pair is None:
                    if time.monotonic() - self._pending_detection_since >= 2.0:
                        self.last_detection_stamp = timestamp
                        self.perception_stats['image_unavailable'] += 1
                        with self.lock:
                            stamps = [item[0] for item in self.image_history]
                        nearest = min((abs(s - timestamp) for s in stamps), default=None)
                        self._emit('frame_rejected',
                                   reason=f'等待图像2秒超时：缓存帧数={len(stamps)}，最近时间差={nearest}s',
                                   measurement_stamp=timestamp)
                else:
                    try:
                        pose = self._pose_for(timestamp)
                        # 先推进游标，失败也不重复消费同一对检测
                        self.last_detection_stamp = timestamp
                        self._process_cone_pair(
                            left_message, right_message, timestamp, image_pair, pose)
                        self.perception_stats['processed_pairs'] += 1
                    except (ValueError, cv2.error) as error:
                        if 'pose' in str(error):
                            self.perception_stats['pose_unavailable'] += 1
                        self.last_detection_stamp = timestamp
                        self._emit('frame_rejected', reason=str(error),
                                   measurement_stamp=timestamp)
                    did_work = True
            if not did_work:
                self.perception_stop.wait(0.03)
        self.node.get_logger().info('建图后台感知线程已停止')

    def _start_perception_worker(self):
        """启动后台感知线程（幂等：已在运行则不重复启动）。"""
        if self.perception_thread is not None and self.perception_thread.is_alive():
            return
        self.perception_stop.clear()
        self.perception_thread = threading.Thread(
            target=self._perception_loop, name='mapping-perception', daemon=True)
        self.perception_thread.start()

    def _stop_perception_worker(self):
        """通知后台线程退出并回收（最多等 2s，避免任务结束时留下线程）。"""
        self.perception_stop.set()
        if self.perception_thread is not None:
            self.perception_thread.join(timeout=2.0)
            self.perception_thread = None

    def _read_tag(self):
        """等待后台线程累计到足够的 AprilTag 有效观测。

        主线程不重复扫描图像（那是后台线程的工作），只轮询 tag_filter 的
        观测次数；每 3s 打印一次进度统计，超时则输出完整诊断（帧数/标记数/
        ID 不匹配/无深度/最近原因/期望 ID/字典）。
        """
        self.state = 'reading_tag'
        self.node.get_logger().info(
            f'开始识别池底标记：字典={self.params["tag_dictionary"]}，'
            f'期望ID={self.params["tag_id"]}，图像={self.params["image_topic"]}；'
            f'代码路径={__file__}')
        next_log = time.monotonic()
        end = min(self.deadline, time.monotonic() + float(self.params['tag_timeout']))
        while self._ready() and time.monotonic() < end:
            with self.lock:
                tag_filter = getattr(self, 'tag_filter', None)
                observations = tag_filter.observations if tag_filter else 0
            if observations >= int(self.params['min_observations']):
                return True
            if time.monotonic() >= next_log:
                self.node.get_logger().info(
                    f'标记识别进度：{self.tag_scan_stats}，最近原因={self.last_tag_reason}')
                next_log = time.monotonic() + 3.0
            time.sleep(0.05)
        self.node.get_logger().error(
            'AprilTag 识别超时: frames=%d markers=%d wrong_id=%d '
            'no_depth=%d last=%s expected_id=%s families=%s' % (
                self.tag_scan_stats['frames'], self.tag_scan_stats['markers'],
                self.tag_scan_stats['wrong_id'], self.tag_scan_stats['no_depth'],
                self.last_tag_reason, self.params['tag_id'],
                ','.join(name for name, _ in self.tag_detectors)))
        return False

    def _travel_to(self, position, label):
        """用 WTRAVEL 走到世界系 xy 位置。

        z 固定为巡检高度 survey_z，yaw 固定为 survey_yaw_deg（下视相机视野
        与艇体朝向一致，便于左右目图像稳定对齐）。
        单次超时取 move_timeout 与「任务剩余时间」的较小值，避免超出总时限。
        返回值来自 BasicMotion 的执行结果。
        """
        if not self._ready():
            return False
        timeout = min(float(self.params['move_timeout']),
                      max(1.0, self.deadline - time.monotonic()))
        success, message = self.node._send_action_goal(
            BasicMotion.Goal.WTRAVEL,
            [float(position[0]), float(position[1]), float(self.params['survey_z']),
             float(self.params['survey_yaw_deg'])],
            'xyzrz', timeout=timeout,
            task_context=self.node._format_motion_context(label))
        self._emit('travel_result', label=label, success=success,
                   message=message, target=list(map(float, position)))
        return success

    def execute(self):
        """任务主流程（由 task_runner 调用，返回 True/False 表示成功/失败）。

        流程：
          1. 等待左右相机内参和位姿就绪，建立双目标定与 SGBM（最多等 20s）；
          2. 启动后台感知线程（此后视觉与运动并行）；
          3. WTRAVEL 到池底标记附近并确认标记（触发器）；
          4. 按 visit_order 逐格：WTRAVEL 到理论格点中心 -> 等待观察时间 ->
             记录该格点观测统计。单格点观测不足只记警告并继续（空格是允许的）；
          5. 九格走完冻结地图，返回标记上方，先圆形后方形遍历已确认目标；
          6. 任何异常（运动失败、标定失败、标记未确认）都置
             state 并把 node.stopped 置位，安全中止整条任务链；
          7. finally 中停止后台线程并再发布一次最终地图。
        """
        self._emit('started', model='best.pt', class_map={0: 'square_cone', 1: 'round_cone'},
                   depth_method='sgbm_mask_mode')
        try:
            # 等待标定所需数据（CameraInfo 左右各一份 + 至少一条位姿）
            ready_end = min(self.deadline, time.monotonic() + 20.0)
            while self._ready() and time.monotonic() < ready_end:
                with self.lock:
                    ready = len(self.camera_info) == 2 and bool(self.pose_history)
                if ready and self._prepare_calibration():
                    break
                time.sleep(0.05)
            if self.calibration is None:
                raise RuntimeError('camera calibration or pose unavailable')
            self._emit('calibration_ready', baseline_m=self.calibration.baseline_m)
            # 标定完成后立刻启动后台感知，移动途中产生的数据也能被利用
            self._start_perception_worker()
            self.state = 'travel_to_tag'
            if not self._travel_to(
                    [self.params['tag_x'], self.params['tag_y']], 'mapping:travel_to_april_tag'):
                raise RuntimeError('WTRAVEL to tag failed')
            if not self._read_tag():
                raise RuntimeError('AprilTag/ArUco trigger was not confirmed')
            # 按配置顺序访问九个格点
            for cell in tuple(self.params['visit_order']):
                self.current_cell = int(cell)
                self.state = 'travel_to_cell'
                center = self.grid_centers[self.current_cell]
                if not self._travel_to(center[:2], f'mapping:travel_to_cell_{cell}'):
                    # 运动失败会危及安全，直接中止任务（与观测失败区别对待）
                    raise RuntimeError(f'WTRAVEL to cell {cell} failed')
                self.state = 'observe_cell'
                observation_ok = self._observe_cones()
                self.visit_order.append(self.current_cell)
                observation = self.cell_observations.get(self.current_cell, {})
                if not observation_ok:
                    # 观测不足只记录并继续：九宫格里的空格本就应没有锥桶
                    self.observation_failures.append(self.current_cell)
                    self.node.get_logger().warning(
                        f'格点{cell}本次停留观测不足，保留累计证据并继续检索九宫格：'
                        f'同步帧={observation.get("synchronized_frames", 0)}，'
                        f'有效目标测量={observation.get("valid_measurements", 0)}')
                    self._emit(
                        'cell_completed', cell=self.current_cell,
                        observation_ok=False, **observation)
                else:
                    self._emit(
                        'cell_completed', cell=self.current_cell,
                        observation_ok=True, **observation)
                # 每格完成后立刻刷新地图，便于现场实时观察进度
                self.publish_map()
            self._stop_perception_worker()
            self.final_assignment = self._select_final_assignment()
            confirmed = sorted(self.final_assignment)
            self.state = 'mapping_complete'
            self._emit('mapping_completed', confirmed_cells=confirmed,
                       assignment=self.final_assignment, result_complete=len(confirmed) == 4)
            self.publish_map()
            self._traverse_cones()
            self.state = 'complete'
            self._emit('completed', confirmed_cells=confirmed,
                       assignment=self.final_assignment,
                       traversal_order=list(self.traversal_order),
                       result_complete=len(confirmed) == 4)
            return True
        except Exception as error:
            # 失败时区分「被外部停止」和「真实失败」，并停止整条任务链
            self.state = 'stopped' if self.node.stopped else 'failed'
            self._emit('failed', reason=str(error))
            self.node.get_logger().error(f'mapping task failed: {error}')
            self.node.stopped = True
            return False
        finally:
            # 无论成功失败都要回收线程并发布最终地图
            self._stop_perception_worker()
            self.publish_map()

    def _traverse_cones(self):
        """冻结后的地图驱动遍历：返回标记，然后圆形、方形各访问一次。"""
        tag = getattr(self, 'tag_filter', None)
        if tag is None or not np.all(np.isfinite(tag.position)):
            raise RuntimeError('缺少 AprilTag 有效观测位置，不使用 JSON 坐标替代遍历目标')
        tag_position = tag.position[:2].copy()
        # 先验证整条路线，避免已知地图缺陷导致途中才失败。
        route = []
        for kind in (1, 0):
            for cell in sorted(self.final_assignment):
                if self.final_assignment[cell] != kind or cell in self.traversal_order:
                    continue
                track = self.filters.get(cell)
                if track is None or not np.all(np.isfinite(track.position)):
                    raise RuntimeError(f'格点{cell}缺少有效融合位置，无法遍历')
                route.append((cell, kind, track.position[:2].copy()))
        self.current_cell = None
        self.state = 'return_to_tag'
        self.node.get_logger().info('建图结束：返回 AprilTag 上方，准备先圆形后方形遍历')
        self._emit('traversal_started', route=[cell for cell, _, _ in route])
        self.publish_map()
        if not self._travel_mapped(tag_position, 'mapping:return_to_april_tag'):
            raise RuntimeError('返回 AprilTag 上方失败，停止遍历')
        for cell, kind, position in route:
            self.current_cell = cell
            self.state = 'traverse_round_cone' if kind == 1 else 'traverse_square_cone'
            name = '圆形' if kind == 1 else '方形'
            self.node.get_logger().info(f'遍历{name}锥桶：格点{cell}，目标xy={position.tolist()}')
            self.publish_map()
            if not self._travel_mapped(position, f'mapping:traverse_cone_{cell}'):
                raise RuntimeError(f'遍历格点{cell}失败，停止后续移动')
            self.traversal_order.append(cell)
            self._emit('cone_traversed', cell=cell, class_id=kind,
                       traversal_order=list(self.traversal_order))
            self.publish_map()
        self._emit('traversal_completed', traversal_order=list(self.traversal_order),
                   result_complete=len(self.traversal_order) == 4)

    @staticmethod
    def _avoidance_path(start, goal, centers, radius):
        """圆形禁入区的可见图最短折线；仅允许从起点区退出、向终点区进入。"""
        start, goal = np.asarray(start, float), np.asarray(goal, float)
        centers = [np.asarray(center, float) for center in centers]
        if not radius > 0 or not np.all(np.isfinite([start, goal])):
            raise RuntimeError('遍历路径参数无效')
        nodes = [start, goal]
        for center in centers:
            for angle in np.linspace(0, 2 * math.pi, 24, endpoint=False):
                point = center + radius * 1.12 * np.array([math.cos(angle), math.sin(angle)])
                if all(np.linalg.norm(point - other) >= radius for other in centers):
                    nodes.append(point)

        def visible(i, j):
            a, b = nodes[i], nodes[j]
            delta = b - a
            length2 = float(delta @ delta)
            if length2 < 1e-12:
                return True
            for center in centers:
                # 在区内起步只能持续向外；抵达区内目标只能作为最后一段。
                if i == 0 and np.linalg.norm(a - center) < radius:
                    if float((a - center) @ delta) >= -1e-9:
                        continue
                    return False
                if j == 1 and np.linalg.norm(b - center) < radius:
                    if float((b - center) @ delta) <= 1e-9:
                        continue
                    return False
                fraction = np.clip(float((center - a) @ delta) / length2, 0, 1)
                if np.linalg.norm(a + fraction * delta - center) < radius - 1e-9:
                    return False
            return True

        distances = [float('inf')] * len(nodes)
        distances[0] = 0.0
        parents, remaining = {}, set(range(len(nodes)))
        while remaining:
            i = min(remaining, key=lambda n: distances[n])
            if not math.isfinite(distances[i]):
                break
            remaining.remove(i)
            if i == 1:
                path, cursor = [], 1
                while cursor != 0:
                    path.append(nodes[cursor].tolist())
                    cursor = parents[cursor]
                return path[::-1]
            for j in remaining:
                cost = distances[i] + float(np.linalg.norm(nodes[j] - nodes[i]))
                if cost < distances[j] and visible(i, j):
                    distances[j], parents[j] = cost, i
        raise RuntimeError('无法找到不重复经过锥桶的路径，请检查观测位置及避让半径')

    def _travel_mapped(self, position, label):
        """使用观测地图绕开其他锥桶，不把去重仅限于终点列表。"""
        with self.lock:
            if not self.pose_history:
                raise RuntimeError('遍历缺少实时艇位')
            pose = self.pose_history[-1]
            start = [pose.robot_x, pose.robot_y]
        centers = [self.filters[cell].position[:2].copy() for cell in self.final_assignment]
        radius = float(self.params.get('traversal_clearance_m', 0.3))
        margin = float(self.params.get('traversal_tracking_margin_m', 0.15))
        if margin < 0 or not math.isfinite(margin):
            raise RuntimeError('遍历跟踪余量无效')
        self.traversal_path = self._avoidance_path(start, position, centers, radius + margin)
        self._emit('traversal_path_planned', label=label, start=list(map(float, start)),
                   waypoints=self.traversal_path, clearance_m=radius,
                   tracking_margin_m=margin,
                   target_source='observed_map')
        self.node.get_logger().info(
            f'观测地图避让路径：{label}，半径={radius:.2f}m，航点={self.traversal_path}')
        self.publish_map()
        for index, waypoint in enumerate(self.traversal_path):
            # WTRAVEL 按实时艇位起步；到点容差导致的偏差也必须通过安全检查。
            with self.lock:
                pose = self.pose_history[-1]
                actual = [pose.robot_x, pose.robot_y]
            checked = self._avoidance_path(actual, waypoint, centers, radius)
            if len(checked) > 1:
                raise RuntimeError('实际艇位偏离避让走廊，停止遍历，禁止直接穿过锥桶')
            if not self._travel_to(waypoint, f'{label}:waypoint_{index}'):
                return False
        return True

    def _select_final_assignment(self):
        """在有类别观测支持的格点中联合选择两方两圆，不补造无观测目标。"""
        votes = {cell: tuple(counts) for cell, counts in self.class_votes.items()}
        candidates = {kind: sorted(cell for cell, counts in votes.items()
                                   if counts[kind] > 0) for kind in (0, 1)}
        best, best_key = {}, (-1, float('-inf'))
        for squares_n in range(min(2, len(candidates[0])) + 1):
            for squares in combinations(candidates[0], squares_n):
                remaining = [cell for cell in candidates[1] if cell not in squares]
                for rounds_n in range(min(2, len(remaining)) + 1):
                    for rounds in combinations(remaining, rounds_n):
                        assignment = {**dict.fromkeys(squares, 0),
                                      **dict.fromkeys(rounds, 1)}
                        score = sum(np.log1p(votes[cell][kind]) *
                                    votes[cell][kind] / sum(votes[cell])
                                    for cell, kind in assignment.items())
                        key = (len(assignment), score)
                        if key > best_key:
                            best, best_key = assignment, key
        self.node.get_logger().info(
            f'建图最终结果（两方两圆约束）：{best}；'
            f'类别0=方形，类别1=圆形；证据票数={votes}')
        if len(best) < 4:
            self.node.get_logger().warning(
                '观测证据不足四个目标，输出部分结果，缺失目标保持未知')
        return best

    def destroy(self):
        """释放资源：停止感知线程、销毁定时器与所有订阅。"""
        self._stop_perception_worker()
        self.node.destroy_timer(self.publish_timer)
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
