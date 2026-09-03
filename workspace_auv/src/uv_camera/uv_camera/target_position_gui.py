"""Desktop visualizer for static target estimates and their observations.

The node is deliberately independent from the perception pipeline.  Start it
on a desktop only; it subscribes to the localization outputs and never feeds
data back into the estimator.
"""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from uv_msgs.msg import TargetObservation, TargetObservationArray
from uv_msgs.msg import TargetPosition, TargetPositionArray
from uv_msgs.msg import PoseInfo


FORM_NAMES = {
    int(TargetObservation.FORM_FRONT_STEREO): "前视双目",
    int(TargetObservation.FORM_FRONT_MULTI_VIEW): "前视多视角",
    int(TargetObservation.FORM_DOWN_DIRECT): "下视直接",
}
FORM_COLORS = {
    int(TargetObservation.FORM_FRONT_STEREO): "#1565c0",
    int(TargetObservation.FORM_FRONT_MULTI_VIEW): "#ef6c00",
    int(TargetObservation.FORM_DOWN_DIRECT): "#2e7d32",
}
STATUS_NAMES = {
    int(TargetPosition.STATUS_UNINITIALIZED): "未初始化",
    int(TargetPosition.STATUS_ESTIMATING): "估计中",
    int(TargetPosition.STATUS_STABLE): "稳定",
    int(TargetPosition.STATUS_STALE): "过期",
}
TARGET_COLORS = (
    "#c62828", "#6a1b9a", "#00838f", "#ef6c00", "#2e7d32",
    "#283593", "#ad1457", "#5d4037", "#455a64", "#558b2f",
)
SOURCE_NAMES = {"front": "前视", "down": "下视"}
SOURCE_COLORS = {"front": "#1565c0", "down": "#2e7d32"}
ROBOT_FILL = "#b3e5fc"
ROBOT_OUTLINE = "#0277bd"
ROBOT_HEADING = "#d32f2f"


@dataclass(frozen=True)
class RobotPoseSnapshot:
    """Latest robot pose in odom/NED (angles are degrees)."""

    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    stamp: str


def _estimate_source(message) -> str:
    source = str(getattr(message, "estimate_source", "")).strip().lower()
    if source in SOURCE_NAMES:
        return source
    if hasattr(message, "observation_form"):
        form = int(getattr(message, "observation_form", 0))
        return "down" if form == int(TargetObservation.FORM_DOWN_DIRECT) else "front"
    # Pre-source TargetPosition messages were down-only.
    return "down"


def _physical_key(message) -> tuple[str, str, int]:
    """Use source + physical identity, never only the raw detector label."""
    physical_name = str(getattr(message, "physical_class_name", "")).strip()
    return (
        _estimate_source(message),
        physical_name or str(message.class_name),
        int(message.instance_id),
    )


def _form_name(form: int) -> str:
    return FORM_NAMES.get(int(form), f"未知({int(form)})")


def _form_mask_name(mask: int) -> str:
    forms = [name for value, name in FORM_NAMES.items() if int(mask) & value]
    return " + ".join(forms) if forms else "无"


def _stamp_text(stamp) -> str:
    if stamp.sec == 0 and stamp.nanosec == 0:
        return "-"
    return f"{stamp.sec}.{stamp.nanosec // 1_000_000:03d}"


class TargetPositionGui(Node):
    """Tk GUI that visualizes odom/NED X-Y positions and observation factors."""

    def __init__(self):
        super().__init__("target_position_gui")
        # Tk Canvas is not retained-mode.  Rebuilding several hundred items at
        # 10 Hz makes the entire desktop unresponsive, especially over X11.
        self.declare_parameter("refresh_period_ms", 250)
        self.declare_parameter("ray_display_length_m", 4.0)
        self.declare_parameter("observation_table_limit", 200)
        self.declare_parameter("max_plot_observations", 160)
        self.refresh_period_ms = max(
            50, int(self.get_parameter("refresh_period_ms").value))
        self.ray_display_length = max(
            0.1, float(self.get_parameter("ray_display_length_m").value))
        self.observation_table_limit = max(
            10, int(self.get_parameter("observation_table_limit").value))
        self.max_plot_observations = max(
            10, int(self.get_parameter("max_plot_observations").value))

        self.targets: dict[tuple[str, str, int], TargetPosition] = {}
        self.observations: dict[int, TargetObservation] = {}
        self.robot_pose: RobotPoseSnapshot | None = None
        self.selected_key: tuple[str, str, int] | None = None
        self.closed = False
        self.target_stamp = "-"
        self.observation_stamp = "-"
        self.pose_stamp = "-"
        self._refresh_requested = True
        # ``None`` means X-Y follows the automatic data bounds.  Once the
        # user pans or zooms, keep their world-coordinate viewport stable as
        # fresh localization messages arrive.
        self._xy_view_bounds: tuple[float, float, float, float] | None = None
        self._xy_drag_point: tuple[float, float] | None = None
        self._last_plot_view: dict[str, float | str] | None = None

        self.root = tk.Tk()
        self.root.title("游龙 AUV 目标位置与观测可视化")
        self.root.minsize(1180, 720)
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self._build_window()

        self.create_subscription(
            TargetPositionArray,
            "/perception/target_positions",
            self._on_targets,
            10,
        )
        self.create_subscription(
            TargetObservationArray,
            "/perception/target_observations",
            self._on_observations,
            10,
        )
        self.create_subscription(
            PoseInfo,
            "/basic_motion/pose_info",
            self._on_pose,
            10,
        )
        self.root.after(self.refresh_period_ms, self._tick)

    def _build_window(self):
        toolbar = ttk.Frame(self.root, padding=(8, 6))
        toolbar.pack(fill=tk.X)
        ttk.Button(toolbar, text="清空本地显示历史",
                   command=self._clear_observations).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="重置 N-E 视图",
                   command=self._reset_xy_view).pack(side=tk.LEFT, padx=(8, 0))
        self.show_rays = tk.BooleanVar(value=True)
        ttk.Checkbutton(toolbar, text="显示前视多视角射线",
                        variable=self.show_rays,
                        command=self._draw_scene).pack(side=tk.LEFT, padx=12)
        self.show_stale = tk.BooleanVar(value=True)
        ttk.Checkbutton(toolbar, text="显示过期目标",
                        variable=self.show_stale,
                        command=self._refresh).pack(side=tk.LEFT)
        self.view_plane = tk.StringVar(value="xy")
        for value, label in (("xy", "N-E 俯视"), ("xz", "X-Z 侧视"),
                             ("yz", "Y-Z 侧视")):
            ttk.Radiobutton(toolbar, text=label, value=value,
                            variable=self.view_plane,
                            command=self._draw_scene).pack(side=tk.LEFT, padx=3)
        self.only_selected = tk.BooleanVar(value=False)
        ttk.Checkbutton(toolbar, text="仅显示选中目标",
                        variable=self.only_selected,
                        command=self._refresh).pack(side=tk.LEFT, padx=12)
        ttk.Label(
            toolbar,
            text=("N-E：滚轮缩放、左键拖拽；橙色虚线箭头=前视多视角射线；"
                  "蓝框=前视估计；绿圆=下视估计；灰虚线=两者差值"),
        ).pack(side=tk.RIGHT)

        body = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        self.plot_frame = ttk.Labelframe(
            body, text="N-E 俯视图（单位：m）", padding=6)
        self.canvas = tk.Canvas(
            self.plot_frame, width=720, height=650, background="#fafafa",
            highlightthickness=1, highlightbackground="#bdbdbd")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<ButtonPress-1>", self._xy_pan_start)
        self.canvas.bind("<B1-Motion>", self._xy_pan_move)
        self.canvas.bind("<ButtonRelease-1>", self._xy_pan_stop)
        self.canvas.bind("<MouseWheel>", self._xy_mouse_wheel)
        # X11 reports wheel movement as Button-4/Button-5 instead of
        # MouseWheel.  Binding both also keeps the GUI usable over SSH/X11.
        self.canvas.bind("<Button-4>", self._xy_mouse_wheel)
        self.canvas.bind("<Button-5>", self._xy_mouse_wheel)
        self.canvas.bind("<Configure>", lambda _event: self._draw_scene())
        body.add(self.plot_frame, weight=3)

        table_frame = ttk.Frame(body, padding=(8, 0, 0, 0))
        body.add(table_frame, weight=4)
        self._build_target_table(table_frame)
        self._build_observation_table(table_frame)

        self.status_text = tk.StringVar(value="等待定位节点数据…")
        ttk.Label(self.root, textvariable=self.status_text,
                  anchor=tk.W, padding=(10, 3)).pack(fill=tk.X)

    def _build_target_table(self, parent):
        frame = ttk.Labelframe(parent, text="前/下视估计（/perception/target_positions）", padding=5)
        frame.pack(fill=tk.BOTH, expand=False)
        columns = ("source", "id", "class", "x", "y", "z", "sigma", "forms", "status", "age")
        self.target_tree = ttk.Treeview(
            frame, columns=columns, show="headings", height=10, selectmode="browse")
        headings = {
            "source": "估计来源", "id": "物理类/实例", "class": "检测类别", "x": "X 北", "y": "Y 东",
            "z": "Z 下", "sigma": "σ合成", "forms": "已融合观测",
            "status": "状态", "age": "年龄(s)",
        }
        widths = {
            "source": 70, "id": 125, "class": 175, "x": 65, "y": 65, "z": 65,
            "sigma": 65, "forms": 155, "status": 60, "age": 65,
        }
        for column in columns:
            self.target_tree.heading(column, text=headings[column])
            self.target_tree.column(column, width=widths[column], anchor=tk.CENTER,
                                    stretch=column in ("class", "forms"))
        target_scroll = ttk.Scrollbar(frame, orient=tk.VERTICAL,
                                      command=self.target_tree.yview)
        self.target_tree.configure(yscrollcommand=target_scroll.set)
        self.target_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        target_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.target_tree.bind("<<TreeviewSelect>>", self._select_target)

    def _build_observation_table(self, parent):
        frame = ttk.Labelframe(parent, text="观测历史（/perception/target_observations）", padding=5)
        frame.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        columns = ("seq", "id", "class", "form", "kind", "location", "confidence", "stamp")
        self.observation_tree = ttk.Treeview(
            frame, columns=columns, show="headings", height=16, selectmode="browse")
        headings = {
            "seq": "序号", "id": "物理类/实例", "class": "检测类别", "form": "观测形式",
            "kind": "内容", "location": "测量点 / 射线原点", "confidence": "置信度",
            "stamp": "观测时间",
        }
        widths = {
            "seq": 64, "id": 140, "class": 150, "form": 92, "kind": 48,
            "location": 180, "confidence": 65, "stamp": 105,
        }
        for column in columns:
            self.observation_tree.heading(column, text=headings[column])
            self.observation_tree.column(column, width=widths[column], anchor=tk.CENTER,
                                         stretch=column in ("class", "location"))
        observation_scroll = ttk.Scrollbar(frame, orient=tk.VERTICAL,
                                           command=self.observation_tree.yview)
        self.observation_tree.configure(yscrollcommand=observation_scroll.set)
        self.observation_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        observation_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.observation_tree.bind("<<TreeviewSelect>>", self._select_observation)

    def _on_targets(self, message: TargetPositionArray):
        self.targets = {
            _physical_key(target): target for target in message.targets
        }
        self.target_stamp = _stamp_text(message.header.stamp)
        self._refresh_requested = True

    def _on_observations(self, message: TargetObservationArray):
        self.observations = {
            int(observation.observation_id): observation
            for observation in message.observations
        }
        self.observation_stamp = _stamp_text(message.header.stamp)
        self._refresh_requested = True

    def _on_pose(self, message: PoseInfo):
        """Cache the latest vehicle pose for the map model."""
        values = (
            float(message.robot_x), float(message.robot_y), float(message.robot_z),
            float(message.robot_roll), float(message.robot_pitch),
            float(message.robot_yaw),
        )
        if not all(math.isfinite(value) for value in values):
            return
        self.robot_pose = RobotPoseSnapshot(*values, _stamp_text(message.stamp))
        self.pose_stamp = self.robot_pose.stamp
        self._refresh_requested = True

    def _tick(self):
        if self.closed:
            return
        try:
            # Drain the two subscription queues, then render just once.  A
            # single ``spin_once`` per GUI tick otherwise leaves stale
            # observation arrays queued behind newer ones.
            for _ in range(8):
                rclpy.spin_once(self, timeout_sec=0.0)
        except ExternalShutdownException:
            self._close()
            return
        if self._refresh_requested:
            self._refresh()
            self._refresh_requested = False
        self.root.after(self.refresh_period_ms, self._tick)

    def _refresh(self):
        self._refresh_target_table()
        self._refresh_observation_table()
        self._draw_scene()
        self.status_text.set(
            f"前/下视估计：{len(self.targets)} 个；几何观测：{len(self.observations)} 条；"
            f"目标消息：{self.target_stamp}；观测消息：{self.observation_stamp}；"
            f"机器人位姿：{self.pose_stamp}"
        )

    def _visible_key(self, key: tuple[str, str, int]) -> bool:
        return not self.only_selected.get() or key == self.selected_key

    def _recent_observations(self):
        """Return newest-first history without discarding GUI-side records."""
        return sorted(
            self.observations.values(),
            key=lambda observation: int(observation.observation_id),
            reverse=True,
        )

    def _refresh_target_table(self):
        for item in self.target_tree.get_children():
            self.target_tree.delete(item)
        for key, target in sorted(self.targets.items()):
            if not self.show_stale.get() and int(target.status) == int(TargetPosition.STATUS_STALE):
                continue
            covariance = target.position_covariance
            sigma = math.sqrt(max(0.0, covariance[0] + covariance[4] + covariance[8]))
            observed_ids = ",".join(
                str(class_id) for class_id in target.observed_class_ids)
            values = (
                SOURCE_NAMES.get(key[0], key[0]),
                f"{key[1]}/{key[2]}",
                f"{target.class_name} (ID:{observed_ids})",
                f"{target.world_x:.3f}", f"{target.world_y:.3f}",
                f"{target.world_z:.3f}", f"{sigma:.3f}",
                _form_mask_name(target.observation_form_mask),
                STATUS_NAMES.get(int(target.status), str(target.status)),
                f"{target.age_sec:.2f}",
            )
            iid = f"{key[0]}:{key[1]}:{key[2]}"
            self.target_tree.insert("", tk.END, iid=iid, values=values)
        if self.selected_key is not None:
            iid = (f"{self.selected_key[0]}:{self.selected_key[1]}:"
                   f"{self.selected_key[2]}")
            if self.target_tree.exists(iid):
                self.target_tree.selection_set(iid)

    def _refresh_observation_table(self):
        for item in self.observation_tree.get_children():
            self.observation_tree.delete(item)
        history = self._recent_observations()
        for observation in history[:self.observation_table_limit]:
            key = _physical_key(observation)
            if not self._visible_key(key):
                continue
            if observation.has_position:
                kind = "测量点"
                location = (
                    f"({observation.world_x:.2f}, {observation.world_y:.2f}, "
                    f"{observation.world_z:.2f})"
                )
            else:
                kind = "射线"
                location = (
                    f"O=({observation.ray_origin_x:.2f}, "
                    f"{observation.ray_origin_y:.2f}, "
                    f"{observation.ray_origin_z:.2f})"
                )
            values = (
                observation.observation_id,
                f"{key[1]}/{key[2]}", observation.class_name,
                _form_name(observation.observation_form), kind, location,
                f"{observation.confidence:.2f}",
                _stamp_text(observation.observation_stamp),
            )
            self.observation_tree.insert(
                "", tk.END, iid=f"obs:{observation.observation_id}", values=values)

    def _select_target(self, _event):
        selected = self.target_tree.selection()
        if not selected:
            return
        try:
            source, physical_class_name, instance_id = selected[0].split(":", maxsplit=2)
            selected_key = source, physical_class_name, int(instance_id)
        except ValueError:
            return
        # ``selection_set`` in _refresh_target_table emits this event too.
        # Do not turn that programmatic selection into recursive full redraws.
        if selected_key == self.selected_key:
            return
        self.selected_key = selected_key
        self._refresh()

    def _select_observation(self, _event):
        selected = self.observation_tree.selection()
        if not selected:
            return
        try:
            observation_id = int(selected[0].split(":", maxsplit=1)[1])
            selected_key = _physical_key(self.observations[observation_id])
        except (IndexError, KeyError, ValueError):
            return
        if selected_key == self.selected_key:
            return
        self.selected_key = selected_key
        self._refresh()

    def _clear_observations(self):
        """Clear only this GUI's received history; publisher data is untouched."""
        self.observations.clear()
        self._refresh()

    def _reset_xy_view(self):
        """Return X-Y to automatic bounds; X-Z/Y-Z never use this viewport."""
        self._xy_view_bounds = None
        self._draw_scene()

    def _xy_view_mapping(self):
        view = self._last_plot_view
        if view is None or view["plane"] != "xy":
            return None
        return view

    def _xy_world_at_canvas(self, canvas_x: float, canvas_y: float):
        view = self._xy_view_mapping()
        if view is None:
            return None
        scale = float(view["scale"])
        if scale <= 1e-12:
            return None
        world_x = float(view["min_x"]) + (
            float(canvas_x) - float(view["margin"])) / scale
        world_y = float(view["min_y"]) + (
            float(view["height"]) - float(view["margin"])
            - float(canvas_y)) / scale
        return world_x, world_y

    def _xy_pan_start(self, event):
        if self.view_plane.get() != "xy" or self._xy_view_mapping() is None:
            return
        self._xy_drag_point = float(event.x), float(event.y)
        self.canvas.configure(cursor="fleur")

    def _xy_pan_move(self, event):
        if self._xy_drag_point is None or self.view_plane.get() != "xy":
            return
        view = self._xy_view_mapping()
        if view is None:
            return
        scale = float(view["scale"])
        if scale <= 1e-12:
            return
        previous_x, previous_y = self._xy_drag_point
        delta_x = (float(event.x) - previous_x) / scale
        delta_y = (float(event.y) - previous_y) / scale
        min_x, max_x, min_y, max_y = self._current_xy_bounds(view)
        # Move the content together with the pointer, rather than moving a
        # selection rectangle.
        self._xy_view_bounds = (
            min_x - delta_x, max_x - delta_x,
            min_y + delta_y, max_y + delta_y,
        )
        self._xy_drag_point = float(event.x), float(event.y)
        self._draw_scene()

    def _xy_pan_stop(self, _event):
        if self._xy_drag_point is not None:
            self._xy_drag_point = None
            self.canvas.configure(cursor="")

    def _xy_mouse_wheel(self, event):
        if self.view_plane.get() != "xy":
            return
        anchor = self._xy_world_at_canvas(event.x, event.y)
        view = self._xy_view_mapping()
        if anchor is None or view is None:
            return
        wheel_delta = getattr(event, "delta", 0)
        if getattr(event, "num", None) == 4 or wheel_delta > 0:
            zoom = 1.25
        elif getattr(event, "num", None) == 5 or wheel_delta < 0:
            zoom = 1.0 / 1.25
        else:
            return
        min_x, max_x, min_y, max_y = self._current_xy_bounds(view)
        anchor_x, anchor_y = anchor
        # Keep the world point under the pointer fixed while zooming.
        self._xy_view_bounds = (
            anchor_x - (anchor_x - min_x) / zoom,
            anchor_x + (max_x - anchor_x) / zoom,
            anchor_y - (anchor_y - min_y) / zoom,
            anchor_y + (max_y - anchor_y) / zoom,
        )
        self._draw_scene()
        return "break"

    @staticmethod
    def _current_xy_bounds(view):
        return (
            float(view["min_x"]), float(view["max_x"]),
            float(view["min_y"]), float(view["max_y"]),
        )

    def _draw_scene(self):
        self.canvas.delete("all")
        width = max(self.canvas.winfo_width(), 200)
        height = max(self.canvas.winfo_height(), 200)
        observations = [
            observation for observation in self.observations.values()
            if self._visible_key(_physical_key(observation))
        ]
        observations.sort(key=lambda observation: int(observation.observation_id),
                          reverse=True)
        # Keep all observations in memory and in the table, but limit Canvas
        # primitives.  Rays turn into one Tk item each, and drawing all 500
        # history records repeatedly is the dominant source of UI stalls.
        observations = observations[:self.max_plot_observations]
        targets = [
            target for key, target in self.targets.items()
            if self._visible_key(key)
            and (self.show_stale.get()
                 or int(target.status) != int(TargetPosition.STATUS_STALE))
        ]
        auto_bounds = self._scene_bounds(targets, observations, self.robot_pose)
        bounds = (
            self._xy_view_bounds
            if self.view_plane.get() == "xy"
            and self._xy_view_bounds is not None
            else auto_bounds
        )
        min_x, max_x, min_y, max_y = bounds
        margin = 48
        scale = min(
            (width - 2 * margin) / max(max_x - min_x, 1e-6),
            (height - 2 * margin) / max(max_y - min_y, 1e-6),
        )
        self._last_plot_view = {
            "plane": self.view_plane.get(),
            "min_x": min_x,
            "max_x": max_x,
            "min_y": min_y,
            "max_y": max_y,
            "scale": scale,
            "margin": float(margin),
            "height": float(height),
        }

        def canvas_point(x: float, y: float) -> tuple[float, float]:
            canvas_x = margin + (x - min_x) * scale
            if self.view_plane.get() == "xy":
                canvas_y = height - margin - (y - min_y) * scale
            else:
                # NED Z increases down, matching the screen's downward axis.
                canvas_y = margin + (y - min_y) * scale
            return canvas_x, canvas_y

        self._draw_axes(canvas_point, width, height, min_x, max_x, min_y, max_y)

        for observation in observations:
            key = _physical_key(observation)
            color = FORM_COLORS.get(int(observation.observation_form), "#757575")
            selected = key == self.selected_key
            if observation.has_position:
                point_x, point_y = self._plane_coordinates(
                    observation.world_x, observation.world_y, observation.world_z)
                x, y = canvas_point(point_x, point_y)
                radius = 4 if selected else 3
                self.canvas.create_oval(
                    x - radius, y - radius, x + radius, y + radius,
                    fill=color, outline="#111111" if selected else color,
                    width=2 if selected else 1,
                )
            elif self.show_rays.get():
                origin_x, origin_y = self._plane_coordinates(
                    observation.ray_origin_x, observation.ray_origin_y,
                    observation.ray_origin_z)
                endpoint_x, endpoint_y = self._plane_coordinates(
                    observation.ray_origin_x + observation.ray_direction_x * self.ray_display_length,
                    observation.ray_origin_y + observation.ray_direction_y * self.ray_display_length,
                    observation.ray_origin_z + observation.ray_direction_z * self.ray_display_length,
                )
                x1, y1 = canvas_point(origin_x, origin_y)
                x2, y2 = canvas_point(endpoint_x, endpoint_y)
                self.canvas.create_line(
                    x1, y1, x2, y2, fill=color, dash=(5, 3),
                    width=2 if selected else 1, arrow=tk.LAST,
                )

        self._draw_estimate_comparisons(targets, canvas_point)

        for target in targets:
            key = _physical_key(target)
            point_x, point_y = self._plane_coordinates(
                target.world_x, target.world_y, target.world_z)
            x, y = canvas_point(point_x, point_y)
            color_seed = sum(ord(char) for char in key[1])
            color = SOURCE_COLORS.get(key[0],
                                      TARGET_COLORS[(color_seed * 31 + key[2])
                                                     % len(TARGET_COLORS)])
            covariance = target.position_covariance
            variance_x, variance_y = self._plane_variances(covariance)
            sigma_x = 2.0 * math.sqrt(max(0.0, variance_x)) * scale
            sigma_y = 2.0 * math.sqrt(max(0.0, variance_y)) * scale
            if sigma_x >= 1.0 or sigma_y >= 1.0:
                self.canvas.create_oval(
                    x - sigma_x, y - sigma_y, x + sigma_x, y + sigma_y,
                    outline=color, dash=(3, 2), width=1,
                )
            radius = 8 if key == self.selected_key else 6
            if key[0] == "front":
                self.canvas.create_polygon(
                    x, y - radius, x + radius, y,
                    x, y + radius, x - radius, y,
                    fill="white", outline=color, width=2,
                )
            else:
                self.canvas.create_oval(
                    x - radius, y - radius, x + radius, y + radius,
                    fill="white", outline=color, width=2,
                )
            self.canvas.create_line(x - radius - 2, y, x + radius + 2, y,
                                    fill=color, width=2)
            self.canvas.create_line(x, y - radius - 2, x, y + radius + 2,
                                    fill=color, width=2)
            self.canvas.create_text(
                x + 8, y - 9, anchor=tk.SW,
                text=f"{SOURCE_NAMES.get(key[0], key[0])}:{key[1]} [{key[2]}]",
                fill="#212121", font=("TkDefaultFont", 9, "bold"),
            )

        if self.robot_pose is not None:
            self._draw_robot(self.robot_pose, canvas_point)

        self.canvas.create_text(
            8, 8, anchor=tk.NW,
            text=("虚线椭圆：当前平面上的融合位置约 2σ；"
                  f"显示最近 {len(observations)}/{len(self.observations)} 条观测；"
                  "橙色虚线箭头=前视多视角射线；"
                  "蓝色菱形=前视池+聚类+滤波，绿色圆形=下视池+聚类+滤波，"
                  "灰色虚线标注同类前/下视估计的差值；青色模型=AUV，红箭头=机头"),
            fill="#424242", font=("TkDefaultFont", 9),
        )
        if self.robot_pose is not None:
            pose = self.robot_pose
            self.canvas.create_text(
                width - 8, 8, anchor=tk.NE,
                text=(f"AUV  x={pose.x:.2f}  y={pose.y:.2f}  z={pose.z:.2f}  "
                      f"R/P/Y={pose.roll:.1f}/{pose.pitch:.1f}/{pose.yaw:.1f}°"),
                fill=ROBOT_OUTLINE, font=("TkDefaultFont", 9, "bold"),
            )
        plane_titles = {
            "xy": "N-E 俯视图（右=East，上=North，单位：m）",
            "xz": "X-Z 侧视图（X=North，Z=Down，单位：m）",
            "yz": "Y-Z 侧视图（Y=East，Z=Down，单位：m）",
        }
        self.plot_frame.configure(text=plane_titles[self.view_plane.get()])

    @staticmethod
    def _body_to_world(pose: RobotPoseSnapshot, body_point: tuple[float, float, float]):
        """Transform a body/NED model point using the same ZYX convention as localization."""
        roll, pitch, yaw = (
            math.radians(pose.roll), math.radians(pose.pitch), math.radians(pose.yaw)
        )
        cx, sx = math.cos(roll), math.sin(roll)
        cy, sy = math.cos(pitch), math.sin(pitch)
        cz, sz = math.cos(yaw), math.sin(yaw)
        rotation = (
            (cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx),
            (sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx),
            (-sy, cy * sx, cy * cx),
        )
        bx, by, bz = body_point
        return (
            pose.x + rotation[0][0] * bx + rotation[0][1] * by + rotation[0][2] * bz,
            pose.y + rotation[1][0] * bx + rotation[1][1] * by + rotation[1][2] * bz,
            pose.z + rotation[2][0] * bx + rotation[2][1] * by + rotation[2][2] * bz,
        )

    def _draw_robot(self, pose: RobotPoseSnapshot, canvas_point):
        """Draw a lightweight projected AUV model in the selected 2D plane."""
        # The profile is defined in body coordinates: +X is the bow, +Y is
        # starboard, and +Z is down.  Projection through the full pose keeps
        # roll/pitch visible in the side/front views as well as yaw in top view.
        if self.view_plane.get() == "xy":
            profile = (
                (0.62, 0.00, 0.00), (0.28, -0.24, 0.00),
                (-0.42, -0.20, 0.00), (-0.55, -0.08, 0.00),
                (-0.55, 0.08, 0.00), (-0.42, 0.20, 0.00),
                (0.28, 0.24, 0.00),
            )
        elif self.view_plane.get() == "xz":
            profile = (
                (0.62, 0.00, 0.00), (0.28, 0.00, -0.14),
                (-0.42, 0.00, -0.16), (-0.55, 0.00, 0.00),
                (-0.42, 0.00, 0.16), (0.28, 0.00, 0.14),
            )
        else:
            profile = (
                (0.00, -0.24, -0.13), (0.00, 0.24, -0.13),
                (0.00, 0.27, 0.07), (0.00, 0.00, 0.18),
                (0.00, -0.27, 0.07),
            )

        def project(body_point):
            world_point = self._body_to_world(pose, body_point)
            plane_point = self._plane_coordinates(*world_point)
            return canvas_point(*plane_point)

        polygon = [coordinate for point in profile for coordinate in project(point)]
        self.canvas.create_polygon(
            *polygon, fill=ROBOT_FILL, outline=ROBOT_OUTLINE, width=2,
        )

        center_x, center_y = project((0.0, 0.0, 0.0))
        nose_x, nose_y = project((0.76, 0.0, 0.0))
        if math.hypot(nose_x - center_x, nose_y - center_y) >= 3.0:
            self.canvas.create_line(
                center_x, center_y, nose_x, nose_y,
                fill=ROBOT_HEADING, width=2, arrow=tk.LAST,
            )
        self.canvas.create_oval(
            center_x - 3, center_y - 3, center_x + 3, center_y + 3,
            fill=ROBOT_HEADING, outline=ROBOT_OUTLINE, width=1,
        )
        self.canvas.create_text(
            center_x + 9, center_y + 9, anchor=tk.NW,
            text="AUV", fill=ROBOT_OUTLINE,
            font=("TkDefaultFont", 9, "bold"),
        )
        self.canvas.create_text(
            center_x + 9, center_y + 25, anchor=tk.NW,
            text=(f"x={pose.x:.2f} y={pose.y:.2f} z={pose.z:.2f} "
                  f"yaw={pose.yaw:.1f}°"),
            fill=ROBOT_OUTLINE, font=("TkDefaultFont", 8),
        )

    def _plane_coordinates(self, x: float, y: float, z: float) -> tuple[float, float]:
        plane = self.view_plane.get()
        if plane == "xz":
            return float(x), float(z)
        if plane == "yz":
            return float(y), float(z)
        # Keep message coordinates in NED (x=North, y=East), but show a
        # conventional pool plan: east is right and north is up.
        return float(y), float(x)

    def _plane_variances(self, covariance) -> tuple[float, float]:
        plane = self.view_plane.get()
        if plane == "xz":
            return float(covariance[0]), float(covariance[8])
        if plane == "yz":
            return float(covariance[4]), float(covariance[8])
        return float(covariance[4]), float(covariance[0])

    def _draw_estimate_comparisons(self, targets, canvas_point):
        """Connect nearest front/down estimates of the same physical class."""
        by_class = {}
        for key, target in ((
                (_physical_key(target), target) for target in targets)):
            by_class.setdefault(key[1], {"front": [], "down": []})[
                key[0]].append((key, target))
        for entries in by_class.values():
            fronts = entries["front"]
            downs = entries["down"]
            used_down = set()
            for front_key, front in fronts:
                candidates = [
                    (index, down_key, down) for index, (down_key, down)
                    in enumerate(downs) if index not in used_down
                ]
                if not candidates:
                    continue
                index, down_key, down = min(
                    candidates,
                    key=lambda item: math.hypot(
                        float(front.world_x - item[2].world_x),
                        float(front.world_y - item[2].world_y)),
                )
                used_down.add(index)
                front_x, front_y = canvas_point(*self._plane_coordinates(
                    front.world_x, front.world_y, front.world_z))
                down_x, down_y = canvas_point(*self._plane_coordinates(
                    down.world_x, down.world_y, down.world_z))
                self.canvas.create_line(
                    front_x, front_y, down_x, down_y,
                    fill="#616161", dash=(4, 3), width=1,
                )
                mid_x = (front_x + down_x) / 2.0
                mid_y = (front_y + down_y) / 2.0
                delta = math.sqrt(
                    (float(front.world_x - down.world_x)) ** 2
                    + (float(front.world_y - down.world_y)) ** 2
                    + (float(front.world_z - down.world_z)) ** 2
                )
                self.canvas.create_text(
                    mid_x + 3, mid_y - 3, anchor=tk.SW,
                    text=f"Δ={delta:.2f}m", fill="#424242",
                    font=("TkDefaultFont", 8),
                )

    def _scene_bounds(self, targets, observations, robot_pose=None):
        points = []
        if robot_pose is not None:
            points.append(self._plane_coordinates(
                robot_pose.x, robot_pose.y, robot_pose.z))
        for target in targets:
            points.append(self._plane_coordinates(
                target.world_x, target.world_y, target.world_z))
        for observation in observations:
            if observation.has_position:
                points.append(self._plane_coordinates(
                    observation.world_x, observation.world_y, observation.world_z))
            elif self.show_rays.get():
                points.append(self._plane_coordinates(
                    observation.ray_origin_x, observation.ray_origin_y,
                    observation.ray_origin_z))
                points.append(self._plane_coordinates(
                    observation.ray_origin_x +
                    observation.ray_direction_x * self.ray_display_length,
                    observation.ray_origin_y +
                    observation.ray_direction_y * self.ray_display_length,
                    observation.ray_origin_z +
                    observation.ray_direction_z * self.ray_display_length,
                ))
        if not points:
            return -5.0, 5.0, -5.0, 5.0
        min_x = min(point[0] for point in points)
        max_x = max(point[0] for point in points)
        min_y = min(point[1] for point in points)
        max_y = max(point[1] for point in points)
        span = max(max_x - min_x, max_y - min_y, 1.0)
        padding = span * 0.16
        return min_x - padding, max_x + padding, min_y - padding, max_y + padding

    def _draw_axes(self, canvas_point, width, height,
                   min_x, max_x, min_y, max_y):
        if min_y <= 0.0 <= max_y:
            x1, y = canvas_point(min_x, 0.0)
            x2, _ = canvas_point(max_x, 0.0)
            self.canvas.create_line(x1, y, x2, y, fill="#cfd8dc")
        if min_x <= 0.0 <= max_x:
            x, y1 = canvas_point(0.0, min_y)
            _, y2 = canvas_point(0.0, max_y)
            self.canvas.create_line(x, y1, x, y2, fill="#cfd8dc")
        if self.view_plane.get() == "xy":
            horizontal, vertical = "East →", "North ↑"
        elif self.view_plane.get() == "xz":
            horizontal, vertical = "X: North →", "Z: Down ↓"
        else:
            horizontal, vertical = "Y: East →", "Z: Down ↓"
        self.canvas.create_text(width - 8, height - 8, anchor=tk.SE,
                                text=horizontal, fill="#546e7a")
        vertical_anchor = tk.NW if self.view_plane.get() != "xy" else tk.SW
        self.canvas.create_text(8, height - 8 if vertical_anchor == tk.SW else 8,
                                anchor=vertical_anchor, text=vertical,
                                fill="#546e7a")

    def _close(self):
        if self.closed:
            return
        self.closed = True
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TargetPositionGui()
        node.run()
    except tk.TclError as error:
        raise RuntimeError(
            "无法创建图形窗口；请在带桌面显示的环境中运行 target_position_gui"
        ) from error
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
