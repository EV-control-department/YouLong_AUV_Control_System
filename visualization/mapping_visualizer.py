#!/usr/bin/env python3
"""建图调试面板：DDS 只接收小型地图/事件，图像经 camera HTTP 快照。

地图和原始世界坐标点来自 DDS ``/task/mapping/map``，实时状态来自
``/task/mapping/events``；视觉诊断由 uv_camera 内部计算并经 HTTP
``/mapping/*.jpg`` 提供。面板不订阅 DDS 图像，也不计算 SGBM。

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

import os
import urllib.request
import matplotlib

# Matplotlib 3.5 and newer PySide6 releases have an incompatible Qt enum
# adapter.  Render off-screen and hand the resulting RGBA image to Qt; this
# also keeps the ROS callback and UI event loops independent.
matplotlib.use("Agg")
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib import font_manager
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSPresetProfiles, QoSProfile
from std_msgs.msg import String
from uv_msgs.msg import PoseInfo

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

for font_path in font_manager.findSystemFonts():
    if 'CJK' in font_path or 'cjk' in font_path:
        font_manager.fontManager.addfont(font_path)
matplotlib.rcParams['font.family'] = ['Noto Sans CJK JP', 'Noto Sans CJK SC', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class RosSnapshot(Node):
    """DDS 接收端；回调只保存最新数据，不做重计算。"""

    def __init__(self):
        super().__init__("mapping_dashboard")
        self.lock = threading.RLock()
        self.map_payload = None
        self.pose = None
        self.trajectory = deque(maxlen=1500)
        self.events = deque(maxlen=200)
        self.event_points = {}
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/task/mapping/map", self._map_cb, map_qos)
        self.create_subscription(String, "/task/mapping/events", self._event_cb, 100)
        self.create_subscription(PoseInfo, "/basic_motion/pose_info", self._pose_cb, sensor_qos)

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
            if not self.trajectory or np.linalg.norm(np.subtract(self.pose[:3], self.trajectory[-1])) > 0.02:
                self.trajectory.append(self.pose[:3])


    def snapshot(self):
        with self.lock:
            return {
                "map": copy.deepcopy(self.map_payload),
                "pose": self.pose,
                "trajectory": list(self.trajectory),
                "events": list(self.events),
                "event_points": copy.deepcopy(self.event_points),
            }




class CameraSnapshots:
    """Fetch uv_camera JPEG diagnostics without DDS image traffic."""

    def __init__(self):
        self.base = os.environ.get('UV_CAMERA_MJPEG_URL',
                                   'http://127.0.0.1:8090').rstrip('/')
        self.lock = threading.Lock()
        self.frames = {}
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        names = ('input', 'overlay', 'disparity', 'depth', 'histogram')
        while not self.stop.is_set():
            for name in names:
                if self.stop.is_set():
                    break
                try:
                    request = urllib.request.Request(
                        f'{self.base}/mapping/{name}.jpg',
                        headers={'Cache-Control': 'no-cache'})
                    with urllib.request.urlopen(request, timeout=1.0) as response:
                        payload = response.read()
                        stamp = response.headers.get('X-Frame-Stamp-Ns', '')
                    with self.lock:
                        self.frames[name] = (payload, stamp)
                except (OSError, ValueError):
                    pass
            self.stop.wait(0.7)

    def snapshot(self):
        with self.lock:
            return dict(self.frames)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.5)


class MplCanvas(QFrame):
    def __init__(self, parent=None, projection="3d"):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.display = QLabel("等待图形…")
        self.display.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.display.setMinimumSize(320, 180)
        layout.addWidget(self.display)
        self.figure = Figure(facecolor=BG, tight_layout=True)
        self.axis = self.figure.add_subplot(111, projection=projection)
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
        self.camera_snapshots = CameraSnapshots()
        self.last_image_stamps = {}
        self.setWindowTitle("YouLong · Mapping Lab")
        self.resize(1560, 980)
        self.setStyleSheet(self._stylesheet())
        self._build_ui()
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
        subtitle = QLabel("DDS 地图 + camera 同帧视觉诊断")
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

        self.hist_canvas = QFrame(right)
        self.hist_canvas.display = QLabel('等待 camera 深度统计…')
        self.hist_canvas.display.setAlignment(Qt.AlignmentFlag.AlignCenter)
        QVBoxLayout(self.hist_canvas).addWidget(self.hist_canvas.display)
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
        frames = self.camera_snapshots.snapshot()
        for name in ('input', 'overlay', 'disparity', 'depth', 'histogram'):
            sample = frames.get(name)
            if sample is None or self.last_image_stamps.get(name) == sample[1]:
                continue
            pixmap = QPixmap()
            if pixmap.loadFromData(sample[0], 'JPEG'):
                label = (self.hist_canvas.display if name == 'histogram'
                         else self.image_labels[name])
                label.setPixmap(pixmap.scaled(
                    label.size(), Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation))
                self.last_image_stamps[name] = sample[1]
        self.metrics['sync'].setText(
            'camera同帧' if frames.get('depth') else '等待camera诊断流')
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
        center = np.asarray(grid.get("center", [2, -4, 1.394]), dtype=float)
        side = float(grid.get("side_m", 2.4))
        floor = float(grid.get("floor_z", center[2]))
        angle = math.radians(float(grid.get('yaw_deg', 0)))
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        for offset in np.linspace(-side / 2, side / 2, 4):
            for segment in (np.array([[offset, -side/2], [offset, side/2]]),
                            np.array([[-side/2, offset], [side/2, offset]])):
                line = segment @ rotation.T + center[:2]
                axis.plot(line[:, 0], line[:, 1], [-floor, -floor], color=MUTED, alpha=0.6)
        trajectory = np.asarray(snapshot['trajectory'])
        planned = np.asarray(payload.get('traversal_path', []))
        returning = np.asarray(payload.get('return_path', []))
        if returning.size:
            height = -snapshot['pose'][2] if snapshot['pose'] else -0.1
            axis.plot(returning[:, 0], returning[:, 1],
                      np.full(len(returning), height), ':s', color='#e879f9',
                      linewidth=1.5, markersize=3, label='返回标记')
        if planned.size:
            height = -snapshot['pose'][2] if snapshot['pose'] else -0.1
            axis.plot(planned[:, 0], planned[:, 1], np.full(len(planned), height),
                      '--o', color='#ffd43b', linewidth=1.5, markersize=3, label='遍历规划')
        if trajectory.size:
            axis.plot(trajectory[:, 0], trajectory[:, 1], -trajectory[:, 2], color=ACCENT, alpha=0.6, linewidth=1)
        for pane in (axis.xaxis, axis.yaxis, axis.zaxis):
            pane.set_pane_color((0.08, 0.11, 0.15, 1))
        for cell in payload.get("cells", []):
            x, y, z = cell.get("center", center)
            color = CLASS_COLORS.get(cell.get("class_id"), "#718096")
            if cell.get("visited"):
                axis.scatter([x], [y], [-z], marker="s", s=90, facecolors="none",
                             edgecolors=color, linewidths=1.8)
            axis.text(x, y, -z, f" {cell.get('id')}", color=MUTED, fontsize=9)
            points = self._cell_points(cell, snapshot)
            for accepted in (True, False):
                positions = np.asarray([p['position'] for p in points
                                        if p.get('position') and bool(p.get('accepted')) == accepted])
                if positions.size:
                    axis.scatter(positions[:, 0], positions[:, 1], -positions[:, 2],
                                 s=16 if accepted else 10, color=color if accepted else '#657080',
                                 alpha=0.65 if accepted else 0.2, marker='o' if accepted else 'x')
            position = cell.get("position")
            if position:
                px, py, pz = position
                guessed = cell.get("source") == "fallback"
                axis.scatter([px], [py], [-pz], s=130,
                             facecolors="none" if guessed else color,
                             marker="D" if guessed else "o",
                             edgecolors=color if guessed else "white",
                             linewidths=1.5 if guessed else 0.8,
                             label=("猜测: " if guessed else "") + str(cell.get("label")))
        tag = payload.get("tag")
        if tag and tag.get("position"):
            tx, ty, tz = tag["position"]
            guessed = tag.get("source") in ("fallback", "partial_vision")
            axis.scatter([tx], [ty], [-tz], marker="*", s=240,
                         color="#fb923c" if guessed else "#ffd43b",
                         edgecolors="white", linewidths=0.7,
                         label="猜测 AprilTag" if guessed else "AprilTag")
        if snapshot["pose"]:
            px, py, pz, yaw = snapshot["pose"]
            yaw = math.radians(yaw)
            axis.scatter([px], [py], [-pz], marker="^", s=130, color=ACCENT, label="AUV")
            axis.quiver(px, py, -pz, 0.35 * math.cos(yaw), 0.35 * math.sin(yaw), 0,
                        color=ACCENT, linewidth=2)
        floor = float(grid.get("floor_z", center[2]))
        axis.set_xlim(center[0] - side * 0.75, center[0] + side * 0.75)
        axis.set_ylim(center[1] - side * 0.75, center[1] + side * 0.75)
        if snapshot['pose']:
            px, py, pz, _ = snapshot['pose']
            axis.set_xlim(min(center[0] - side*.75, px-.4), max(center[0] + side*.75, px+.4))
            axis.set_ylim(min(center[1] - side*.75, py-.4), max(center[1] + side*.75, py+.4))
        axis.set_zlim(-max(floor + .45, snapshot['pose'][2] + .3 if snapshot['pose'] else floor), .2)
        axis.set_box_aspect((np.ptp(axis.get_xlim()), np.ptp(axis.get_ylim()), np.ptp(axis.get_zlim())))
        axis.set_title(f"{payload.get('state', 'unknown')} · 三维原始点集 + 卡尔曼结果",
                       color=TEXT, pad=12)
        handles, labels = axis.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        if unique:
            axis.legend(unique.values(), unique.keys(), loc="upper left", fontsize=8)
        self.map_canvas.draw_idle()
        self.map_hint.setText(
            f"状态: {payload.get('state')}  |  已访问: {len(payload.get('visit_order', []))}/9  |  "
            f"点集: {payload.get('measurement_count', 0)}  |  "
            f"兜底: {'未验证' if payload.get('fallback_used') else '无'}  |  "
            f"遍历格序: {payload.get('traversal_plan_cells', [])}  |  "
            f"已到锥桶: {payload.get('traversal_order', [])}")

    def _update_table(self, payload, snapshot):
        cells = payload.get("cells", [])
        self.table.setRowCount(len(cells))
        for row, cell in enumerate(cells):
            points = self._cell_points(cell, snapshot)
            position = cell.get("position")
            values = [
                str(cell.get("id", "")), "是" if cell.get("visited") else "否",
                (("猜测: " if cell.get("source") == "fallback" else "") +
                 CLASS_NAMES.get(cell.get("class_id"), "—")), str(len(points)),
                str(cell.get("accepted_observations", 0)),
                "—" if not position else "(%.2f, %.2f, %.2f)" % tuple(position),
                "—" if not cell.get("residual") else "%.3f" % np.linalg.norm(cell["residual"][:2]),
            ]
            for column, value in enumerate(values):
                self.table.setItem(row, column, QTableWidgetItem(value))


    def closeEvent(self, event):
        self.timer.stop()
        self.camera_snapshots.close()
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
