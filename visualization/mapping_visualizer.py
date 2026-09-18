#!/usr/bin/env python3
"""现代化 DDS 建图调试面板。

数据全部来自 ROS 2/DDS：地图和原始世界坐标点来自 ``/task/mapping/map``，
实时状态来自 ``/task/mapping/events``，视觉诊断来自 stitched 图像、YOLO
分割结果和 CameraInfo。面板只做可视化，不发布控制指令。

启动：
    source /opt/ros/humble/setup.bash
    source workspace_sim/install/setup.bash
    source workspace_auv/install/setup.bash
    python3 visualization/mapping_visualizer.py
"""

from __future__ import annotations

import copy
import json
import math
import threading
import time
from collections import deque

import cv2
import matplotlib

# Matplotlib 3.5 and newer PySide6 releases have an incompatible Qt enum
# adapter.  Render off-screen and hand the resulting RGBA image to Qt; this
# also keeps the ROS callback and UI event loops independent.
matplotlib.use("Agg")
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSPresetProfiles, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from uv_msgs.msg import DetectionArray, PoseInfo

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication, QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QMainWindow, QPlainTextEdit, QSplitter, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget,
)


CLASS_NAMES = {0: "方形锥桶", 1: "圆形锥桶"}
CLASS_COLORS = {0: "#ff9f43", 1: "#4dabf7"}
BG, PANEL, PANEL_2 = "#0d1117", "#151c27", "#1c2633"
TEXT, MUTED, ACCENT = "#e6edf3", "#8b98a8", "#4dd0e1"


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def image_to_array(message: Image) -> np.ndarray | None:
    """复制 ROS Image，避免 DDS 回调返回后引用失效。"""
    try:
        channels = 1 if message.encoding in ("mono8", "8UC1") else 3
        row = np.frombuffer(message.data, dtype=np.uint8).reshape(
            (message.height, message.step))
        image = row[:, :message.width * channels].reshape(
            (message.height, message.width, channels))
        if channels == 1:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        elif message.encoding.lower() in ("rgb8", "rgba8"):
            image = cv2.cvtColor(image[:, :, :3], cv2.COLOR_RGB2BGR)
        return image.copy()
    except (ValueError, TypeError):
        return None


def pixmap_from_bgr(image: np.ndarray, size=(500, 250)) -> QPixmap:
    if image is None or image.size == 0:
        return QPixmap()
    rgb = cv2.cvtColor(np.ascontiguousarray(image), cv2.COLOR_BGR2RGB)
    h, w = rgb.shape[:2]
    qimage = QImage(rgb.data, w, h, w * 3, QImage.Format.Format_RGB888).copy()
    return QPixmap.fromImage(qimage).scaled(
        size[0], size[1], Qt.AspectRatioMode.KeepAspectRatio,
        Qt.TransformationMode.SmoothTransformation)


class RosSnapshot(Node):
    """DDS 接收端；回调只保存最新数据，不做重计算。"""

    def __init__(self):
        super().__init__("mapping_dashboard")
        self.lock = threading.RLock()
        self.map_payload = None
        self.pose = None
        self.image = None
        self.image_stamp = 0.0
        self.down_left = None
        self.camera_info = None
        self.events = deque(maxlen=200)
        self.event_points = {}
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/task/mapping/map", self._map_cb, map_qos)
        self.create_subscription(String, "/task/mapping/events", self._event_cb, 100)
        self.create_subscription(PoseInfo, "/basic_motion/pose_info", self._pose_cb, sensor_qos)
        self.create_subscription(Image, "/auv/down_cam/stitched", self._image_cb, sensor_qos)
        self.create_subscription(
            DetectionArray, "/perception/detection/down_left",
            self._detection_cb, sensor_qos)
        self.create_subscription(
            CameraInfo, "/sim/down_cam/left/camera_info",
            self._camera_info_cb, sensor_qos)

    def _map_cb(self, message):
        try:
            payload = json.loads(message.data)
            if isinstance(payload, dict):
                with self.lock:
                    self.map_payload = payload
        except (json.JSONDecodeError, TypeError, ValueError) as error:
            self.get_logger().warning(f"地图消息解析失败: {error}")

    def _event_cb(self, message):
        try:
            event = json.loads(message.data)
            if not isinstance(event, dict):
                return
            with self.lock:
                self.events.append(event)
                if event.get("event") == "cone_measurement":
                    cell = int(event.get("cell", -1))
                    position = event.get("position")
                    if cell >= 0 and position:
                        self.event_points.setdefault(cell, deque(maxlen=400)).append({
                            "position": position,
                            "class_id": int(event.get("class_id", -1)),
                            "confidence": float(event.get("confidence", 0.0)),
                            "depth_m": float(event.get("depth_m", 0.0)),
                            "residual_m": float(event.get("residual_m", 0.0)),
                            "accepted": bool(event.get("accepted", False)),
                        })
        except (json.JSONDecodeError, TypeError, ValueError):
            return

    def _pose_cb(self, message):
        with self.lock:
            self.pose = (float(message.robot_x), float(message.robot_y),
                         float(message.robot_z), float(message.robot_yaw))

    def _image_cb(self, message):
        image = image_to_array(message)
        if image is not None:
            with self.lock:
                self.image = image
                self.image_stamp = stamp_seconds(message.header.stamp)

    def _detection_cb(self, message):
        detections = []
        for detection in message.detections:
            if int(detection.class_id) not in (0, 1):
                continue
            detections.append({
                "class_id": int(detection.class_id),
                "confidence": float(detection.confidence),
                "bbox": (float(detection.bbox_x1), float(detection.bbox_y1),
                          float(detection.bbox_x2), float(detection.bbox_y2)),
                "mask_x": list(detection.mask_x),
                "mask_y": list(detection.mask_y),
            })
        with self.lock:
            self.down_left = (stamp_seconds(message.header.stamp), detections)

    def _camera_info_cb(self, message):
        with self.lock:
            self.camera_info = message

    def snapshot(self):
        with self.lock:
            return {
                "map": copy.deepcopy(self.map_payload),
                "pose": self.pose,
                "image": None if self.image is None else self.image.copy(),
                "image_stamp": self.image_stamp,
                "detections": copy.deepcopy(self.down_left),
                "camera_info": self.camera_info,
                "events": list(self.events),
                "event_points": copy.deepcopy(self.event_points),
            }


class SgbmWorker:
    """后台 SGBM 计算器，避免高分辨率视差计算阻塞 Qt。"""

    def __init__(self, result_callback):
        self.result_callback = result_callback
        self.lock = threading.Lock()
        self.pending = None
        self.wake = threading.Event()
        self.stop = False
        self.thread = threading.Thread(target=self._run, name="sgbm-viewer", daemon=True)
        self.thread.start()

    def submit(self, image, detections, camera_info):
        with self.lock:
            self.pending = (image.copy(), detections, camera_info)
        self.wake.set()

    def close(self):
        self.stop = True
        self.wake.set()
        self.thread.join(timeout=1.0)

    @staticmethod
    def _calibration(info):
        fx, fy, cx, cy, baseline = 672.18, 672.18, 640.0, 480.0, 0.1
        if info is not None:
            try:
                fx, fy, cx, cy = (float(info.k[0]), float(info.k[4]),
                                  float(info.k[2]), float(info.k[5]))
                if len(info.p) >= 4 and abs(float(info.p[3])) > 1e-6:
                    baseline = abs(float(info.p[3]) / fx)
            except (IndexError, TypeError, ValueError):
                pass
        return fx, fy, cx, cy, baseline

    def _run(self):
        while not self.stop:
            self.wake.wait(0.2)
            self.wake.clear()
            with self.lock:
                pending, self.pending = self.pending, None
            if pending is None:
                continue
            try:
                self.result_callback(self._compute(*pending))
            except Exception as error:
                self.result_callback({"error": str(error)})

    def _compute(self, stitched, detections, camera_info):
        width = stitched.shape[1] // 2
        left, right = stitched[:, :width], stitched[:, width:width * 2]
        gray_left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=128, blockSize=5,
            P1=200, P2=800, disp12MaxDiff=1,
            uniquenessRatio=8, speckleWindowSize=80, speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        disparity = sgbm.compute(gray_left, gray_right).astype(np.float32) / 16.0
        valid = disparity > 1.0
        fx, _, _, _, baseline = self._calibration(camera_info)
        depth = np.full(disparity.shape, np.nan, dtype=np.float32)
        depth[valid] = fx * baseline / disparity[valid]
        depth_valid = depth[np.isfinite(depth) & (depth >= 0.2) & (depth <= 8.0)]

        overlay = left.copy()
        combined_mask = np.zeros(depth.shape, dtype=np.uint8)
        modes = []
        for detection in detections or []:
            if (len(detection["mask_x"]) >= 3 and
                    len(detection["mask_x"]) == len(detection["mask_y"])):
                polygon = np.column_stack((detection["mask_x"], detection["mask_y"])).astype(np.int32)
            else:
                x1, y1, x2, y2 = detection["bbox"]
                polygon = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.int32)
            mask = np.zeros(depth.shape, dtype=np.uint8)
            cv2.fillPoly(mask, [polygon], 1)
            combined_mask |= mask
            color_hex = CLASS_COLORS[detection["class_id"]]
            color = tuple(int(color_hex[i:i + 2], 16) for i in (1, 3, 5))
            overlay[mask.astype(bool)] = (
                0.45 * overlay[mask.astype(bool)] + 0.55 * np.asarray(color)).astype(np.uint8)
            values = depth[mask.astype(bool)]
            values = values[np.isfinite(values) & (values >= 0.2) & (values <= 8.0)]
            if len(values) >= 20:
                bins = np.arange(0.2, 8.02, 0.02)
                counts, edges = np.histogram(values, bins=bins)
                index = int(np.argmax(counts))
                modes.append({
                    "class_id": detection["class_id"],
                    "confidence": detection["confidence"],
                    "depth_m": float((edges[index] + edges[index + 1]) * 0.5),
                    "samples": int(counts[index]),
                })
            cv2.polylines(overlay, [polygon], True, color, 3)

        def colorize(array, scale, offset=0.0):
            image = np.zeros(array.shape, dtype=np.uint8)
            finite = np.isfinite(array)
            image[finite] = np.clip((array[finite] - offset) * scale, 0, 255).astype(np.uint8)
            return cv2.applyColorMap(image, cv2.COLORMAP_TURBO)

        return {
            "overlay": overlay,
            "disparity": colorize(disparity, 255.0 / 64.0),
            "depth": colorize(depth, 255.0 / 8.0),
            "mask": cv2.cvtColor(combined_mask * 255, cv2.COLOR_GRAY2BGR),
            "hist_x": np.arange(0.2, 8.0, 0.02),
            "hist_y": np.histogram(depth_valid, bins=np.arange(0.2, 8.02, 0.02))[0]
            if len(depth_valid) else np.zeros(390),
            "modes": modes,
            "valid_pixels": int(len(depth_valid)),
            "valid_ratio": float(np.count_nonzero(valid) / valid.size),
        }


class MplCanvas(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.display = QLabel("等待图形…")
        self.display.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.display.setMinimumSize(320, 180)
        layout.addWidget(self.display)
        self.figure = Figure(facecolor=BG, tight_layout=True)
        self.axis = self.figure.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasAgg(self.figure)

    def draw_idle(self):
        self.canvas.draw()
        rgba = np.asarray(self.canvas.buffer_rgba()).copy()
        height, width = rgba.shape[:2]
        image = QImage(rgba.data, width, height, width * 4,
                       QImage.Format.Format_RGBA8888).copy()
        self.display.setPixmap(QPixmap.fromImage(image).scaled(
            self.display.size(), Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation))


class MappingDashboard(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.last_image_stamp = -1.0
        self.sgbm_result = None
        self.setWindowTitle("YouLong · Mapping Lab")
        self.resize(1560, 980)
        self.setStyleSheet(self._stylesheet())
        self._build_ui()
        self.sgbm = SgbmWorker(self._receive_sgbm)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(180)

    @staticmethod
    def _stylesheet():
        return f"""
        QMainWindow, QWidget {{ background: {BG}; color: {TEXT}; font-family: 'Noto Sans'; }}
        QGroupBox {{ background: {PANEL}; border: 1px solid #263447; border-radius: 10px;
                     margin-top: 12px; padding: 12px; font-weight: 600; color: {MUTED}; }}
        QGroupBox::title {{ subcontrol-origin: margin; left: 12px; padding: 0 5px; }}
        QLabel#title {{ color: {TEXT}; font-size: 25px; font-weight: 700; }}
        QLabel#subtitle {{ color: {MUTED}; font-size: 12px; }}
        QPlainTextEdit, QTableWidget {{ background: {PANEL}; border: 1px solid #263447; color: {TEXT};
                                       gridline-color: #263447; border-radius: 8px; }}
        QHeaderView::section {{ background: {PANEL_2}; color: {MUTED}; border: 0; padding: 5px; }}
        """

    def _build_ui(self):
        root = QWidget()
        outer = QVBoxLayout(root)
        outer.setContentsMargins(18, 14, 18, 14)
        outer.setSpacing(12)
        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title = QLabel("YouLong · Mapping Lab")
        title.setObjectName("title")
        subtitle = QLabel("DDS-only 3D mapping and SGBM parameter workbench")
        subtitle.setObjectName("subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box)
        header.addStretch()
        self.metrics = {}
        for key, label in (("state", "状态"), ("points", "测量点"),
                           ("accepted", "滤波点"), ("sync", "图像同步")):
            card = QFrame()
            layout = QVBoxLayout(card)
            small = QLabel(label)
            small.setStyleSheet(f"color:{MUTED}; font-size:11px;")
            value = QLabel("—")
            value.setStyleSheet(f"color:{ACCENT}; font-size:20px; font-weight:700;")
            layout.addWidget(small)
            layout.addWidget(value)
            self.metrics[key] = value
            header.addWidget(card)
        outer.addLayout(header)

        main_split = QSplitter(Qt.Orientation.Horizontal)
        left = QWidget()
        left_layout = QVBoxLayout(left)
        self.map_canvas = MplCanvas(left)
        left_layout.addWidget(self.map_canvas, 1)
        self.map_hint = QLabel("等待 /task/mapping/map …")
        self.map_hint.setObjectName("subtitle")
        left_layout.addWidget(self.map_hint)
        main_split.addWidget(left)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        vision = QGroupBox("视觉诊断 · 下视双目 / YOLO-Seg / SGBM")
        vision_grid = QGridLayout(vision)
        self.image_labels = {}
        for index, (key, label) in enumerate((
            ("input", "stitched 输入"), ("overlay", "分割掩膜叠加"),
            ("disparity", "视差图"), ("depth", "深度图"))):
            box = QGroupBox(label)
            layout = QVBoxLayout(box)
            image_label = QLabel("等待图像…")
            image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            image_label.setMinimumSize(260, 150)
            image_label.setStyleSheet(f"background:{BG}; color:{MUTED}; border-radius:6px;")
            layout.addWidget(image_label)
            self.image_labels[key] = image_label
            vision_grid.addWidget(box, index // 2, index % 2)
        right_layout.addWidget(vision, 3)

        self.hist_canvas = MplCanvas(right)
        self.hist_canvas.setMinimumHeight(180)
        hist_group = QGroupBox("深度频率 · 掩膜内合理峰值")
        hist_layout = QVBoxLayout(hist_group)
        hist_layout.addWidget(self.hist_canvas)
        right_layout.addWidget(hist_group, 1)
        main_split.addWidget(right)
        main_split.setSizes([760, 700])
        outer.addWidget(main_split, 5)

        bottom = QSplitter(Qt.Orientation.Horizontal)
        table_group = QGroupBox("九宫格状态与聚类结果")
        table_layout = QVBoxLayout(table_group)
        self.table = QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(["格点", "访问", "类别", "原始点", "滤波观测", "位置 / m", "残差 / m"])
        self.table.horizontalHeader().setStretchLastSection(True)
        table_layout.addWidget(self.table)
        bottom.addWidget(table_group)
        log_group = QGroupBox("实时 DDS 事件")
        log_layout = QVBoxLayout(log_group)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        log_layout.addWidget(self.log_view)
        bottom.addWidget(log_group)
        bottom.setSizes([760, 700])
        outer.addWidget(bottom, 2)
        self.setCentralWidget(root)

    def _receive_sgbm(self, result):
        self.sgbm_result = result

    def _refresh(self):
        snapshot = self.ros_node.snapshot()
        payload = snapshot["map"]
        if payload is not None:
            self._update_map(payload, snapshot)
            self._update_table(payload, snapshot)
            self.metrics["state"].setText(str(payload.get("state", "—")))
            self.metrics["points"].setText(str(payload.get("measurement_count", 0)))
            accepted = sum(int(c.get("accepted_observations", 0)) for c in payload.get("cells", []))
            self.metrics["accepted"].setText(str(accepted))
        if snapshot["image"] is not None:
            self.image_labels["input"].setPixmap(pixmap_from_bgr(snapshot["image"]))
            if snapshot["image_stamp"] != self.last_image_stamp:
                self.last_image_stamp = snapshot["image_stamp"]
                detections = snapshot["detections"][1] if snapshot["detections"] else []
                self.sgbm.submit(snapshot["image"], detections, snapshot["camera_info"])
        if self.sgbm_result is not None:
            self._update_vision(self.sgbm_result)
        if snapshot["detections"]:
            delta = abs(snapshot["image_stamp"] - snapshot["detections"][0])
            self.metrics["sync"].setText(f"{delta:.2f}s")
        events = snapshot["events"]
        if events:
            self.log_view.setPlainText("\n".join(self._event_text(item) for item in events[-8:]))
            self.log_view.verticalScrollBar().setValue(self.log_view.verticalScrollBar().maximum())

    @staticmethod
    def _event_text(event):
        name = event.get("event", "unknown")
        cell = event.get("cell", "-")
        if name == "cone_measurement":
            return (f"测量  格点{cell}  {CLASS_NAMES.get(event.get('class_id'), '?')}  "
                    f"深度={float(event.get('depth_m', 0)):.2f}m  "
                    f"残差={float(event.get('residual_m', 0)):.2f}m  "
                    f"{'接受' if event.get('accepted') else '拒绝'}")
        if name == "frame_rejected":
            return f"拒绝  格点{cell}  {event.get('reason', '')}"
        return f"{name}  格点{cell}  {event.get('state', '')}"

    def _cell_points(self, cell, snapshot):
        points = list(cell.get("measurements") or [])
        if not points:
            points = list(snapshot["event_points"].get(int(cell.get("id", -1)), []))
        return points

    def _update_map(self, payload, snapshot):
        axis = self.map_canvas.axis
        axis.clear()
        axis.set_facecolor(BG)
        axis.set_xlabel("X / m", color=TEXT)
        axis.set_ylabel("Y / m", color=TEXT)
        axis.set_zlabel("-Z / m", color=TEXT)
        axis.tick_params(colors=MUTED)
        axis.grid(True, color="#344253", alpha=0.55)
        grid = payload.get("grid", {})
        center = np.asarray(grid.get("center", [2, -4, 1.994]), dtype=float)
        side = float(grid.get("side_m", 2.0))
        for cell in payload.get("cells", []):
            x, y, z = cell.get("center", center)
            color = CLASS_COLORS.get(cell.get("class_id"), "#718096")
            if cell.get("visited"):
                axis.scatter([x], [y], [-z], marker="s", s=90, facecolors="none",
                             edgecolors=color, linewidths=1.8)
            axis.text(x, y, -z, f" {cell.get('id')}", color=MUTED, fontsize=9)
            for point in self._cell_points(cell, snapshot):
                position = point.get("position")
                if not position:
                    continue
                px, py, pz = position
                if point.get("accepted", False):
                    axis.scatter([px], [py], [-pz], s=22, color=color, alpha=0.78)
                else:
                    axis.scatter([px], [py], [-pz], s=18, color="#657080", alpha=0.28, marker="x")
            position = cell.get("position")
            if position:
                px, py, pz = position
                axis.scatter([px], [py], [-pz], s=130, color=color, marker="o",
                             edgecolors="white", linewidths=0.8, label=cell.get("label"))
        tag = payload.get("tag")
        if tag and tag.get("position"):
            tx, ty, tz = tag["position"]
            axis.scatter([tx], [ty], [-tz], marker="*", s=240, color="#ffd43b",
                         edgecolors="white", linewidths=0.7, label="AprilTag")
        if snapshot["pose"]:
            px, py, pz, yaw = snapshot["pose"]
            axis.scatter([px], [py], [-pz], marker="^", s=130, color=ACCENT, label="AUV")
            axis.quiver(px, py, -pz, 0.35 * math.cos(yaw), 0.35 * math.sin(yaw), 0,
                        color=ACCENT, linewidth=2)
        floor = float(grid.get("floor_z", center[2]))
        axis.set_xlim(center[0] - side * 0.75, center[0] + side * 0.75)
        axis.set_ylim(center[1] - side * 0.75, center[1] + side * 0.75)
        axis.set_zlim(-floor - 0.45, -floor + 0.45)
        axis.set_title(f"{payload.get('state', 'unknown')} · 三维原始点集 + 卡尔曼结果",
                       color=TEXT, pad=12)
        handles, labels = axis.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        if unique:
            axis.legend(unique.values(), unique.keys(), loc="upper left", fontsize=8)
        self.map_canvas.draw_idle()
        self.map_hint.setText(
            f"状态: {payload.get('state')}  |  已访问: {len(payload.get('visit_order', []))}/9  |  "
            f"点集: {payload.get('measurement_count', 0)}  |  拒绝格点: {payload.get('observation_failures', [])}")

    def _update_table(self, payload, snapshot):
        cells = payload.get("cells", [])
        self.table.setRowCount(len(cells))
        for row, cell in enumerate(cells):
            points = self._cell_points(cell, snapshot)
            position = cell.get("position")
            values = [
                str(cell.get("id", "")), "是" if cell.get("visited") else "否",
                CLASS_NAMES.get(cell.get("class_id"), "—"), str(len(points)),
                str(cell.get("accepted_observations", 0)),
                "—" if not position else "(%.2f, %.2f, %.2f)" % tuple(position),
                "—" if not cell.get("residual") else "%.3f" % np.linalg.norm(cell["residual"][:2]),
            ]
            for column, value in enumerate(values):
                self.table.setItem(row, column, QTableWidgetItem(value))

    def _update_vision(self, result):
        if result.get("error"):
            self.image_labels["depth"].setText("SGBM: " + result["error"])
            return
        for key in ("overlay", "disparity", "depth"):
            self.image_labels[key].setPixmap(pixmap_from_bgr(result[key]))
        axis = self.hist_canvas.axis
        axis.clear()
        axis.set_facecolor(BG)
        axis.bar(result["hist_x"], result["hist_y"], width=0.018,
                 color="#4dd0e1", alpha=0.75)
        for mode in result["modes"]:
            axis.axvline(mode["depth_m"], color=CLASS_COLORS.get(mode["class_id"], "white"), linewidth=2)
        axis.set_xlim(0.2, 8.0)
        axis.set_xlabel("深度 / m", color=TEXT)
        axis.set_ylabel("像素数", color=TEXT)
        axis.tick_params(colors=MUTED)
        axis.grid(True, color="#344253", alpha=0.4)
        peaks = ", ".join(f"{mode['depth_m']:.2f}m" for mode in result["modes"]) or "无"
        axis.set_title(f"有效视差 {result['valid_ratio'] * 100:.1f}% · 深度峰值 {peaks}",
                       color=TEXT, fontsize=9)
        self.hist_canvas.draw_idle()

    def closeEvent(self, event):
        self.timer.stop()
        self.sgbm.close()
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()


def main():
    rclpy.init()
    node = RosSnapshot()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    application = QApplication.instance() or QApplication([])
    window = MappingDashboard(node)
    window.show()
    try:
        application.exec()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
