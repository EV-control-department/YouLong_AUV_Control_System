"""Desktop view of current perception measurements and persistent tracks."""

from __future__ import annotations

from collections import OrderedDict
import math
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from auv_protocol.topics import MEASUREMENTS, STATE_ODOM, TRACKS
from uv_msgs.msg import (
    ObjectMeasurement,
    ObjectMeasurementArray,
    ObjectTrack,
    ObjectTrackArray,
    PoseInfo,
)


_FORM_NAMES = {
    int(ObjectMeasurement.FORM_FRONT_STEREO): "前视双目",
    int(ObjectMeasurement.FORM_FRONT_BEARING): "前视方位",
    int(ObjectMeasurement.FORM_DOWN_DIRECT): "下视平面交点",
}
_STATUS_NAMES = {
    int(ObjectTrack.STATUS_TENTATIVE): "暂定",
    int(ObjectTrack.STATUS_STABLE): "稳定",
    int(ObjectTrack.STATUS_STALE): "过期",
    int(ObjectTrack.STATUS_LOST): "丢失",
}
_COLORS = (
    "#1565c0", "#ef6c00", "#2e7d32", "#6a1b9a", "#00838f",
    "#c62828", "#5d4037", "#455a64", "#ad1457", "#558b2f",
)


def _stamp_text(stamp):
    if stamp.sec == 0 and stamp.nanosec == 0:
        return "-"
    return f"{stamp.sec}.{stamp.nanosec // 1_000_000:03d}"


def _measurement_location(measurement):
    if measurement.has_position:
        return (f"({measurement.world_x:.2f}, {measurement.world_y:.2f}, "
                f"{measurement.world_z:.2f}) m")
    if measurement.has_ray:
        return (f"ray O=({measurement.ray_origin_x:.2f}, "
                f"{measurement.ray_origin_y:.2f}, {measurement.ray_origin_z:.2f}) "
                f"D=({measurement.ray_direction_x:.2f}, "
                f"{measurement.ray_direction_y:.2f}, "
                f"{measurement.ray_direction_z:.2f})")
    return "无有效几何结果"


class PerceptionGui(Node):
    """Show every current track and the measurements associated with a track."""

    def __init__(self):
        super().__init__("perception_gui")
        self.declare_parameter("refresh_period_ms", 150)
        self.declare_parameter("measurement_history_limit", 500)
        self.declare_parameter("association_distance_m", 2.0)
        self.refresh_period_ms = max(
            50, int(self.get_parameter("refresh_period_ms").value))
        self.history_limit = max(
            50, int(self.get_parameter("measurement_history_limit").value))
        self.association_distance_m = max(
            0.1, float(self.get_parameter("association_distance_m").value))

        self.tracks: dict[int, ObjectTrack] = {}
        self.measurements: OrderedDict[int, ObjectMeasurement] = OrderedDict()
        self.measurement_tracks: dict[int, int] = {}
        self.pose = None
        self.selected_track_id: int | None = None
        self._closed = False

        self.root = tk.Tk()
        self.root.title("游龙 AUV Perception：观测与 Track")
        self.root.geometry("1420x900")
        self.root.minsize(1100, 700)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._build_ui()

        self.create_subscription(ObjectTrackArray, TRACKS, self._on_tracks, 10)
        self.create_subscription(
            ObjectMeasurementArray, MEASUREMENTS, self._on_measurements, 10)
        self.create_subscription(PoseInfo, STATE_ODOM, self._on_pose, 10)
        self.root.after(self.refresh_period_ms, self._tick)

    def _build_ui(self):
        toolbar = ttk.Frame(self.root, padding=8)
        toolbar.pack(fill=tk.X)
        ttk.Label(
            toolbar,
            text="选择一个 track 查看其近期观测；取消筛选可查看所有观测。",
        ).pack(side=tk.LEFT)
        self.show_all_measurements = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            toolbar, text="显示所有物品的观测",
            variable=self.show_all_measurements,
            command=self._refresh).pack(side=tk.RIGHT)

        vertical = ttk.Panedwindow(self.root, orient=tk.VERTICAL)
        vertical.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        tables = ttk.Panedwindow(vertical, orient=tk.HORIZONTAL)
        vertical.add(tables, weight=3)

        track_frame = ttk.LabelFrame(tables, text="全部 Track（当前状态）", padding=5)
        tables.add(track_frame, weight=1)
        self.track_tree = ttk.Treeview(
            track_frame,
            columns=("id", "class", "source", "position", "confidence",
                     "count", "status", "age"),
            show="headings", selectmode="browse", height=10,
        )
        track_headings = {
            "id": ("Track ID", 76), "class": ("类别", 145),
            "source": ("来源", 100), "position": ("odom 坐标 x/y/z (m)", 220),
            "confidence": ("置信度", 76), "count": ("观测数", 72),
            "status": ("状态", 72), "age": ("Track 年龄(s)", 90),
        }
        for name, (title, width) in track_headings.items():
            self.track_tree.heading(name, text=title)
            self.track_tree.column(name, width=width, anchor=tk.CENTER,
                                   stretch=name in ("class", "position"))
        track_scroll = ttk.Scrollbar(
            track_frame, orient=tk.VERTICAL, command=self.track_tree.yview)
        self.track_tree.configure(yscrollcommand=track_scroll.set)
        self.track_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        track_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.track_tree.bind("<<TreeviewSelect>>", self._select_track)

        observation_frame = ttk.LabelFrame(
            tables, text="选中物品的当前/近期观测", padding=5)
        tables.add(observation_frame, weight=1)
        self.observation_tree = ttk.Treeview(
            observation_frame,
            columns=("id", "camera", "class", "form", "location",
                     "confidence", "stamp"),
            show="headings", height=10,
        )
        observation_headings = {
            "id": ("Observation ID", 105), "camera": ("相机", 100),
            "class": ("类别", 130), "form": ("测量方式", 105),
            "location": ("位置/射线", 300), "confidence": ("置信度", 76),
            "stamp": ("图像时间", 120),
        }
        for name, (title, width) in observation_headings.items():
            self.observation_tree.heading(name, text=title)
            self.observation_tree.column(
                name, width=width, anchor=tk.CENTER,
                stretch=name in ("class", "location"))
        observation_scroll = ttk.Scrollbar(
            observation_frame, orient=tk.VERTICAL,
            command=self.observation_tree.yview)
        self.observation_tree.configure(yscrollcommand=observation_scroll.set)
        self.observation_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        observation_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        map_frame = ttk.LabelFrame(
            vertical, text="全部 Track 的 N-E 平面位置（右为 East，上为 North）",
            padding=5)
        vertical.add(map_frame, weight=4)
        self.canvas = tk.Canvas(map_frame, background="white", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.status = tk.StringVar(value="等待 /auv/perception/tracks 和 measurements …")
        ttk.Label(self.root, textvariable=self.status, anchor=tk.W,
                  padding=(8, 3)).pack(fill=tk.X)

    def _on_tracks(self, message: ObjectTrackArray):
        self.tracks = {int(track.track_id): track for track in message.tracks}
        if (self.selected_track_id is not None
                and self.selected_track_id not in self.tracks):
            self.selected_track_id = None
        for track in message.tracks:
            observation_id = int(track.last_observation_id)
            if observation_id in self.measurements:
                self.measurement_tracks[observation_id] = int(track.track_id)
        self._associate_unlinked_measurements()
        self._refresh()

    def _on_measurements(self, message: ObjectMeasurementArray):
        for measurement in message.measurements:
            observation_id = int(measurement.observation_id)
            self.measurements[observation_id] = measurement
            self.measurements.move_to_end(observation_id)
            for track in self.tracks.values():
                if int(track.last_observation_id) == observation_id:
                    self.measurement_tracks[observation_id] = int(track.track_id)
                    break
        while len(self.measurements) > self.history_limit:
            removed_id, _ = self.measurements.popitem(last=False)
            self.measurement_tracks.pop(removed_id, None)
        self._associate_unlinked_measurements()
        self._refresh()

    def _associate_unlinked_measurements(self):
        """Join recent measurements to tracks by exact ID, then geometry."""
        tracks_by_class: dict[int, list[ObjectTrack]] = {}
        for track in self.tracks.values():
            tracks_by_class.setdefault(int(track.class_id), []).append(track)

        for observation_id, measurement in self.measurements.items():
            if observation_id in self.measurement_tracks:
                continue
            candidates = tracks_by_class.get(int(measurement.class_id), [])
            if not candidates:
                continue
            exact = [track for track in candidates
                     if int(track.last_observation_id) == observation_id]
            if len(exact) == 1:
                self.measurement_tracks[observation_id] = int(exact[0].track_id)
                continue
            if measurement.has_position:
                ranked = sorted(
                    (math.dist(
                        (float(measurement.world_x), float(measurement.world_y),
                         float(measurement.world_z)),
                        (float(track.world_x), float(track.world_y),
                         float(track.world_z))), int(track.track_id))
                    for track in candidates)
                if (ranked and ranked[0][0] <= self.association_distance_m
                        and (len(ranked) == 1
                             or ranked[1][0] - ranked[0][0] > 0.05)):
                    self.measurement_tracks[observation_id] = ranked[0][1]
            elif len(candidates) == 1:
                self.measurement_tracks[observation_id] = int(candidates[0].track_id)

    def _on_pose(self, message: PoseInfo):
        values = tuple(float(value) for value in (
            message.robot_x, message.robot_y, message.robot_z,
            message.robot_roll, message.robot_pitch, message.robot_yaw))
        if all(math.isfinite(value) for value in values):
            self.pose = values
            self._refresh()

    def _select_track(self, _event):
        selection = self.track_tree.selection()
        self.selected_track_id = int(selection[0]) if selection else None
        self._refresh()

    def _visible_measurements(self):
        measurements = list(self.measurements.values())
        if self.show_all_measurements.get() or self.selected_track_id is None:
            return measurements
        return [measurement for measurement in measurements
                if self.measurement_tracks.get(int(measurement.observation_id))
                == self.selected_track_id]

    def _refresh(self):
        if not hasattr(self, "track_tree"):
            return
        selected = self.selected_track_id
        for item in self.track_tree.get_children():
            self.track_tree.delete(item)
        for track_id, track in sorted(self.tracks.items()):
            position = (f"{track.world_x:.2f}, {track.world_y:.2f}, "
                        f"{track.world_z:.2f}")
            values = (
                track_id, track.physical_class_name or track.class_name,
                track.estimate_source, position, f"{track.confidence:.2f}",
                track.measurement_count,
                _STATUS_NAMES.get(int(track.status), str(track.status)),
                f"{track.age_sec:.2f}",
            )
            self.track_tree.insert("", tk.END, iid=str(track_id), values=values)
        if selected in self.tracks:
            self.track_tree.selection_set(str(selected))

        for item in self.observation_tree.get_children():
            self.observation_tree.delete(item)
        visible = sorted(
            self._visible_measurements(),
            key=lambda item: int(item.observation_id), reverse=True)
        for measurement in visible:
            values = (
                measurement.observation_id, measurement.source_camera,
                measurement.physical_class_name or measurement.class_name,
                _FORM_NAMES.get(int(measurement.measurement_form),
                                str(measurement.measurement_form)),
                _measurement_location(measurement),
                f"{measurement.confidence:.2f}",
                _stamp_text(measurement.observation_stamp),
            )
            self.observation_tree.insert(
                "", tk.END, iid=f"obs:{measurement.observation_id}",
                values=values)
        self._draw_map()
        selected_text = (f"Track {selected}" if selected in self.tracks
                         else "未选中 Track")
        linked_count = sum(
            track_id == selected for track_id in self.measurement_tracks.values())
        self.status.set(
            f"tracks={len(self.tracks)}；缓存观测={len(self.measurements)}；"
            f"{selected_text} 的关联观测={linked_count}；"
            f"odom={self.pose[:3] if self.pose else '等待中'}")

    def _draw_map(self):
        self.canvas.delete("all")
        width = max(200, self.canvas.winfo_width())
        height = max(200, self.canvas.winfo_height())
        points = [(float(track.world_y), float(track.world_x))
                  for track in self.tracks.values()]
        points.extend((float(item.world_y), float(item.world_x))
                      for item in self._visible_measurements()
                      if item.has_position)
        if self.pose is not None:
            points.append((self.pose[1], self.pose[0]))
        if points:
            min_x, max_x = min(p[0] for p in points), max(p[0] for p in points)
            min_y, max_y = min(p[1] for p in points), max(p[1] for p in points)
        else:
            min_x = min_y = -5.0
            max_x = max_y = 5.0
        span = max(max_x - min_x, max_y - min_y, 2.0)
        padding = span * 0.18
        min_x, max_x = min_x - padding, max_x + padding
        min_y, max_y = min_y - padding, max_y + padding
        margin = 42
        scale = min((width - 2 * margin) / (max_x - min_x),
                    (height - 2 * margin) / (max_y - min_y))

        def canvas_point(east, north):
            return (margin + (east - min_x) * scale,
                    height - margin - (north - min_y) * scale)

        left_top = canvas_point(min_x, max_y)
        right_bottom = canvas_point(max_x, min_y)
        self.canvas.create_rectangle(*left_top, *right_bottom, outline="#b0bec5")
        self.canvas.create_text(margin, 12, anchor=tk.NW,
                                text=f"N {max_y:.1f} m", fill="#455a64")
        self.canvas.create_text(width - margin, height - 12, anchor=tk.SE,
                                text=f"E {max_x:.1f} m", fill="#455a64")

        for measurement in self._visible_measurements():
            if not measurement.has_position:
                continue
            x, y = canvas_point(measurement.world_y, measurement.world_x)
            self.canvas.create_oval(x - 3, y - 3, x + 3, y + 3,
                                    fill="#90a4ae", outline="")

        for track_id, track in self.tracks.items():
            x, y = canvas_point(track.world_y, track.world_x)
            color = _COLORS[(track_id - 1) % len(_COLORS)]
            radius = 9 if track_id == self.selected_track_id else 6
            self.canvas.create_oval(
                x - radius, y - radius, x + radius, y + radius,
                fill="white", outline=color,
                width=3 if track_id == self.selected_track_id else 2)
            self.canvas.create_text(
                x + radius + 3, y - radius,
                text=f"{track_id}: {track.physical_class_name or track.class_name}",
                anchor=tk.SW, fill=color)
        if self.pose is not None:
            x, y = canvas_point(self.pose[1], self.pose[0])
            self.canvas.create_line(x - 7, y, x + 7, y, fill="#00acc1", width=3)
            self.canvas.create_line(x, y - 7, x, y + 7, fill="#00acc1", width=3)

    def _tick(self):
        if self._closed:
            return
        try:
            for _ in range(8):
                rclpy.spin_once(self, timeout_sec=0.0)
        except ExternalShutdownException:
            self.close()
            return
        self.root.after(self.refresh_period_ms, self._tick)

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PerceptionGui()
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

