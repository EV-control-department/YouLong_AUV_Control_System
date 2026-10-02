"""九宫格静态目标建图任务。

任务目标
--------
让 AUV 自主遍历池底九宫格（3x3）的九个格点，识别其中的方形/圆形锥桶，
并把每个锥桶的位置与类别发布成一张可供其他节点消费的地图。

感知方案
--------
uv_camera 在同一帧内完成 YOLO-Seg、AprilTag、SGBM、掩膜深度主峰与
世界坐标换算，并发布小型 MappingObservationArray。任务只负责格点关联、
静态目标滤波、类别投票及后续遍历规划。

执行框架（运动串行 + 感知并行）
-------------------------------
* 运动串行：主线程通过 BasicMotion 的 WTRAVEL 一次只去一个目标点，保证
  控制链路安全、可随时被 /task/stop 中断。
* 感知并行：camera 独立处理同帧视觉并持续发布观测；任务回调在
  WTRAVEL 移动期间也会更新格点滤波器，不依赖到格点后临时抓图。

坐标系
------
* 输入位姿来自 ``/basic_motion/pose_info``（PoseInfo），位于 basic_motion
  的 odom 系：START 时以当时艇位为原点，NED 约定（x=北, y=东, z=下，
  yaw 顺时针为正且单位为度）。
* 本任务所有输出统一换算到 ``mapping_odom`` 系，即 PoseInfo 所在的 odom
  系；地图里的目标位置与九宫格理论中心在同一个系里比较。

对外接口
--------
* 订阅：``/basic_motion/pose_info``（遍历安全监测）、
  ``/perception/mapping/observations``（带采集时间和质量的小型观测）。
* 发布：``/task/mapping/map``（低频地图快照，TRANSIENT_LOCAL，后加入的
  订阅者也能立刻拿到最新地图）、``/task/mapping/events``（逐事件调试流）。
"""

from collections import deque
from itertools import combinations
import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from uv_msgs.action import BasicMotion
from uv_msgs.msg import MappingObservation, MappingObservationArray, PoseInfo


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
    """提取消息的采集时间戳并转成秒；PoseInfo 使用顶层 stamp。"""
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
        min_observations / class_vote_ratio / expected_cones
            确认目标所需的观测次数、类别投票占比、期望目标数。
        grid_center_x/y、grid_side_m、grid_yaw_deg、floor_z
            九宫格中心、边长、航向和池底深度，用于推算九个格点的理论中心。
        survey_z、survey_yaw_deg
            巡检高度与巡检航向（下视相机贴得越近视野越小，故抬高艇体）。
        tag_x/tag_y、tag_id、tag_dictionary、tag_timeout
            池底标记的理论位置、期望 ID（-1 表示接受任意 ID）、字典和超时。
        visit_order
            九个格点的访问顺序（默认蛇形，减少往返路程）。
        视觉标定、外参、SGBM 和掩膜深度参数现在属于 uv_camera，
        不能再通过任务 JSON 修改。
        measurement_sigma_m / process_noise / mahalanobis_gate / cell_gate_m
            观测噪声、过程噪声、离群点门限和目标-格点关联距离门限。
    """

    def __init__(self, node, params):
        self.node = node
        self.params = params
        # 任务线程与 ROS 观测回调会并发访问滤波器/计数。
        self.lock = threading.RLock()
        self.subscriptions = []
        self.pose_history = deque(maxlen=100)
        self.last_observation_stamp = -1.0
        self.filters = {}                            # 格点索引 -> StaticPositionFilter
        self.class_votes = {}                        # 格点索引 -> [方形票, 圆形票]
        self.visit_order = []                        # 实际访问过的格点顺序
        self.traversal_order = []                    # 建图后成功遍历的锥桶（独立于巡检）
        self.observation_failures = []               # 没拿到足够观测的格点
        self.cell_observations = {}                  # 格点索引 -> 观测计数统计
        self.current_cell = None                     # 当前正在观察的格点
        self.state = 'initializing'                  # 任务状态机，写入事件流
        self.fallback_cells = set()
        self.fallback_tag = None
        self.fallback_reason = None
        self.perception_stats = {                    # 感知统计，便于调参诊断
            'received_frames': 0,
            'processed_frames': 0,
            'rejected_frames': 0,
            'cone_candidates': 0,
            'cone_confidence_rejected': 0,
            'cone_mask_rejected': 0,
            'cone_depth_rejected': 0,
            'cone_observations': 0,
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
        # 地图用 TRANSIENT_LOCAL（latched），后启动的可视化工具也能拿到最新一帧
        qos_map = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = node.create_publisher(String, '/task/mapping/map', qos_map)
        self.event_pub = node.create_publisher(String, '/task/mapping/events', 100)
        self._subscribe(MappingObservationArray,
                        '/perception/mapping/observations', self._observation_cb)
        self._subscribe(PoseInfo, '/basic_motion/pose_info', self._pose_cb)
        # 1Hz 定时发布地图快照，保证外部始终能看到进展
        self.publish_timer = node.create_timer(1.0, self.publish_map)


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
        with self.lock:
            self.pose_history.append(message)

    def _observation_cb(self, message):
        """Consume camera-produced measurements; no image is transported here."""
        timestamp = _stamp(message)
        with self.lock:
            if timestamp <= self.last_observation_stamp:
                return
            self.last_observation_stamp = timestamp
            self.perception_stats['received_frames'] += 1
        if not message.processed:
            self.perception_stats['rejected_frames'] += 1
            self.last_tag_reason = message.reason
            self._emit('frame_rejected', reason=message.reason,
                       measurement_stamp=timestamp)
            return
        self.perception_stats['processed_frames'] += 1
        self.perception_stats['cone_candidates'] += int(message.cone_candidates)
        self.perception_stats['cone_confidence_rejected'] += int(
            message.cone_confidence_rejected)
        self.perception_stats['cone_mask_rejected'] += int(
            message.cone_mask_rejected)
        self.perception_stats['cone_depth_rejected'] += int(
            message.cone_depth_rejected)
        self.perception_stats['cone_observations'] += sum(
            observation.kind == MappingObservation.CONE
            for observation in message.observations)
        self.tag_scan_stats['frames'] += 1
        self.tag_scan_stats['markers'] += int(message.tag_candidates)
        self.tag_scan_stats['no_depth'] += int(message.tag_depth_rejected)
        if not message.tag_candidates and self.state == 'reading_tag':
            self.last_tag_reason = '当前帧未解码到标记'
        elif message.tag_depth_rejected and self.state == 'reading_tag':
            self.last_tag_reason = '标记已解码但掩膜深度峰无效'
        self._record_observation_frame()
        for observation in message.observations:
            if observation.kind == MappingObservation.TAG:
                self._accept_tag_result(observation, timestamp)
            elif observation.kind == MappingObservation.CONE:
                self._process_cone_observation(observation, timestamp)

    def _accept_tag_result(self, observation, timestamp):
        tag_id = int(observation.tag_id)
        if int(self.params['tag_id']) >= 0 and tag_id != int(self.params['tag_id']):
            self.tag_scan_stats['wrong_id'] += 1
            return
        point = np.array([observation.world_x, observation.world_y,
                          observation.world_z], dtype=float)
        covariance = np.eye(3) * float(self.params['measurement_sigma_m']) ** 2
        with self.lock:
            if hasattr(self, 'tag_filter') and tag_id != self.tag_id:
                self.last_tag_reason = '标记ID改变，拒绝合并不同目标'
                return
            if not hasattr(self, 'tag_filter'):
                self.tag_id = tag_id
                self.tag_filter = StaticPositionFilter(point, covariance, timestamp)
                accepted, reason = True, 'initial'
            else:
                accepted, reason = self.tag_filter.update(
                    point, covariance, timestamp,
                    float(self.params['process_noise']),
                    float(self.params['mahalanobis_gate']))
            count = self.tag_filter.accepted
        self._emit('tag_measurement', tag_id=tag_id, position=point.tolist(),
                   depth_samples=int(observation.depth_samples), accepted=accepted,
                   reason=reason, measurement_stamp=timestamp)
        if accepted and count == int(self.params['min_observations']):
            self.node.get_logger().info(f'标记确认成功：ID={tag_id}，有效观测={count}')

    def _process_cone_observation(self, observation, timestamp):
        """Associate to a grid cell, reject outliers, and update a static track."""
        class_id = int(observation.class_id)
        if class_id not in (0, 1):
            return
        point = np.array([observation.world_x, observation.world_y,
                          observation.world_z], dtype=float)
        if not np.all(np.isfinite(point)):
            return
        cell = min(self.grid_centers,
                   key=lambda index: np.linalg.norm(
                       point[:2] - self.grid_centers[index][:2]))
        residual = float(np.linalg.norm(point[:2] - self.grid_centers[cell][:2]))
        record = {'position': point.tolist(), 'class_id': class_id,
                  'confidence': float(observation.confidence),
                  'depth_m': float(observation.depth_m),
                  'residual_m': residual, 'accepted': False,
                  'timestamp': timestamp}
        with self.lock:
            self.measurement_points[cell].append(record)
        if residual > float(self.params['cell_gate_m']):
            self._emit('measurement_rejected', reason='outside_cell_gate',
                       class_id=class_id, position=point.tolist(),
                       residual_m=residual, measurement_stamp=timestamp)
            return
        covariance = np.eye(3) * float(self.params['measurement_sigma_m']) ** 2
        covariance[2, 2] *= 2.0
        with self.lock:
            track = self.filters.get(cell)
            if track is None:
                self.filters[cell] = StaticPositionFilter(point, covariance, timestamp)
                self.class_votes[cell] = [0, 0]
                accepted, reason = True, 'initial'
            else:
                accepted, reason = track.update(
                    point, covariance, timestamp,
                    float(self.params['process_noise']),
                    float(self.params['mahalanobis_gate']))
                if not accepted and reason.startswith('outlier:'):
                    recent = list(self.measurement_points[cell])[-12:]
                    cluster = {}
                    for sample in recent:
                        delta = np.asarray(sample['position']) - point
                        if (sample['residual_m'] <= float(self.params['cell_gate_m'])
                                and float(delta @ np.linalg.solve(2*covariance, delta))
                                <= float(self.params['mahalanobis_gate'])):
                            cluster[sample['timestamp']] = sample
                    required = max(3, min(8, track.accepted + 1))
                    if len(cluster) >= required and len(cluster) > len(recent)/2:
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
                        self.class_votes[cell][class_id] -= 1
                        accepted, reason = True, 'consistent_cluster_reinitialized'
            if accepted:
                self.class_votes[cell][class_id] += 1
                record['accepted'] = True
        self._record_observation_measurement(cell)
        self._emit('cone_measurement', cell=cell, class_id=class_id,
                   confidence=float(observation.confidence),
                   depth_m=float(observation.depth_m),
                   depth_samples=int(observation.depth_samples),
                   position=point.tolist(), residual_m=residual,
                   accepted=accepted, reason=reason,
                   measurement_stamp=timestamp)


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
                guessed = index in self.fallback_cells
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
                    'position': (track.position.tolist() if track else
                                 center.tolist() if guessed else None),
                    'covariance': track.covariance.tolist() if track else None,
                    'source': 'fallback' if guessed else 'vision' if track else 'none',
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
                'fallback_used': bool(self.fallback_cells or self.fallback_tag),
                'fallback_reason': self.fallback_reason,
                'verified_complete': (len(getattr(self, 'final_assignment', {})) == 4
                                      and not self.fallback_cells
                                      and self.fallback_tag is None),
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
            return self.fallback_tag
        return {
            'id': self.tag_id,
            'position': self.tag_filter.position.tolist(),
            'covariance': self.tag_filter.covariance.tolist(),
            'observations': self.tag_filter.observations,
            'source': 'partial_vision' if self.fallback_tag else 'vision',
        }

    def _ready(self):
        """任务是否应继续：ROS 正常 + 未被停止 + 未超过总时限。"""
        return rclpy.ok() and not self.node.stopped and time.monotonic() < self.deadline


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


    def _observe_cones(self):
        """等待 camera 持续发布的观测，不再主动取帧。

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
            f'格点{cell}观察完成：已处理双目帧={observed_frames}，'
            f'新增有效目标测量={measurements}，累计有效目标测量={current["valid_measurements"]}；'
            f'感知统计={self.perception_stats}')
        return observed_frames >= int(self.params['min_observations'])


    def _read_tag(self):
        """等待 camera 发布足够多的有效 AprilTag 观测。"""
        self.state = 'reading_tag'
        self.node.get_logger().info(
            f'开始识别池底标记：字典={self.params["tag_dictionary"]}，'
            f'期望ID={self.params["tag_id"]}，输入=/perception/mapping/observations')
        next_log = time.monotonic()
        end = min(self.deadline, time.monotonic() + float(self.params['tag_timeout']))
        while self._ready() and time.monotonic() < end:
            with self.lock:
                tag_filter = getattr(self, 'tag_filter', None)
                observations = tag_filter.accepted if tag_filter else 0
            if observations >= int(self.params['min_observations']):
                return True
            if time.monotonic() >= next_log:
                self.node.get_logger().info(
                    f'标记识别进度：{self.tag_scan_stats}，最近原因={self.last_tag_reason}')
                next_log = time.monotonic() + 3.0
            time.sleep(0.05)
        self.node.get_logger().error(
            'AprilTag 识别超时: frames=%d markers=%d wrong_id=%d '
            'no_depth=%d last=%s expected_id=%s family=%s' % (
                self.tag_scan_stats['frames'], self.tag_scan_stats['markers'],
                self.tag_scan_stats['wrong_id'], self.tag_scan_stats['no_depth'],
                self.last_tag_reason, self.params['tag_id'],
                self.params['tag_dictionary']))
        if not self._ready():
            return False
        self._assume_tag('AprilTag 识别超时')
        return True

    def _assume_tag(self, reason):
        guessed_id = int(self.params['tag_id'])
        if guessed_id < 0:
            guessed_id = int(self.params.get('fallback_tag_id', 16))
        self.fallback_tag = {
            'id': guessed_id,
            'position': [float(self.params['tag_x']), float(self.params['tag_y']),
                         float(self.params['floor_z'])],
            'covariance': None,
            'observations': 0,
            'source': 'fallback',
        }
        self.fallback_reason = reason
        self.node.get_logger().warning(
            f'标记识别失败，使用未验证的预设 ID={guessed_id} 和位置；'
            '仅用于地图输出，不据此执行锥桶遍历')
        self._emit('tag_fallback', **self.fallback_tag)
        self.publish_map()

    def _fill_fallback_assignment(self, assignment):
        """保留视觉结论，仅对缺失的两方两圆用配置格点补齐。"""
        result = dict(assignment)
        for kind, key in ((0, 'fallback_square_cells'),
                          (1, 'fallback_round_cells')):
            candidates = list(self.params.get(key, ())) + list(range(9))
            for cell in candidates:
                if sum(value == kind for value in result.values()) >= 2:
                    break
                cell = int(cell)
                if cell not in self.grid_centers or cell in result:
                    continue
                result[cell] = kind
                self.fallback_cells.add(cell)
        if self.fallback_cells:
            self.fallback_reason = self.fallback_reason or '锥桶视觉观测不足'
            self.node.get_logger().warning(
                f'建图使用未验证的默认格点：{sorted(self.fallback_cells)}；'
                '不会依据猜测地图自动遍历')
            self._emit('map_fallback', cells=sorted(self.fallback_cells),
                       assignment=result)
        return result

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
          1. 等待 camera 的小型观测消息（最多 20s）；
          2. camera 持续在独立线程处理图像，运动与感知并行；
          3. WTRAVEL 到池底标记附近并确认标记（触发器）；
          4. 按 visit_order 逐格：WTRAVEL 到理论格点中心 -> 等待观察时间 ->
             记录该格点观测统计。单格点观测不足只记警告并继续（空格是允许的）；
          5. 九格走完冻结地图，返回标记上方，先圆形后方形遍历已确认目标；
          6. 视觉不足时发布明确标记的猜测地图并跳过遍历；运动失败或
             未预期异常仍安全中止整条任务链；
          7. finally 中再发布一次最终地图。
        """
        self._emit('started', model='best.pt', class_map={0: 'square_cone', 1: 'round_cone'},
                   depth_method='sgbm_mask_mode')
        try:
            # camera 只在标定和采集位姿可用后才发 processed=true。
            ready_end = min(self.deadline, time.monotonic() + 20.0)
            ready = False
            while self._ready() and time.monotonic() < ready_end:
                with self.lock:
                    ready = (self.perception_stats['processed_frames'] > 0
                             and bool(self.pose_history))
                if ready:
                    break
                time.sleep(0.05)
            if not ready:
                if self.node.stopped or not rclpy.ok():
                    raise RuntimeError('任务被停止')
                self._assume_tag('相机建图观测或位姿不可用')
                self.final_assignment = self._fill_fallback_assignment({})
                self.state = 'fallback'
                self._emit('completed_with_fallback',
                           reason=self.fallback_reason,
                           assignment=self.final_assignment)
                return True
            self._emit('calibration_ready', source='uv_camera')
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
            self.final_assignment = self._fill_fallback_assignment(
                self._select_final_assignment())
            confirmed = sorted(self.final_assignment)
            self.state = 'mapping_complete'
            self._emit('mapping_completed', confirmed_cells=confirmed,
                       assignment=self.final_assignment,
                       result_complete=(len(confirmed) == 4
                                        and not self.fallback_cells
                                        and self.fallback_tag is None))
            self.publish_map()
            if self.fallback_tag or self.fallback_cells:
                self.state = 'complete_with_fallback'
                self._emit('traversal_skipped',
                           reason='地图包含未验证猜测，禁止自动遍历')
                self.publish_map()
                return True
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
            # camera 的感知线程与任务生命周期独立；这里只发布最终地图。
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
        self.node.destroy_timer(self.publish_timer)
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
