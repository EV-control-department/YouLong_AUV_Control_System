"""Display helpers for the simulator and annotated preview."""

from __future__ import annotations

import re
import subprocess


def _x11_active_window_center():
    try:
        active = subprocess.run(
            ["xprop", "-root", "_NET_ACTIVE_WINDOW"],
            check=True, capture_output=True, text=True, timeout=2.0,
        )
        window = re.search(r"0x[0-9a-fA-F]+", active.stdout)
        if not window:
            return None
        geometry = subprocess.run(
            ["xwininfo", "-id", window.group(0)],
            check=True, capture_output=True, text=True, timeout=2.0,
        )
        matches = [
            re.search(r"Absolute upper-left X:\s+(-?\d+)", geometry.stdout),
            re.search(r"Absolute upper-left Y:\s+(-?\d+)", geometry.stdout),
            re.search(r"Width:\s+(\d+)", geometry.stdout),
            re.search(r"Height:\s+(\d+)", geometry.stdout),
        ]
        if all(matches):
            return (
                int(matches[0].group(1)) + int(matches[2].group(1)) / 2.0,
                int(matches[1].group(1)) + int(matches[3].group(1)) / 2.0,
            )
    except (OSError, subprocess.SubprocessError):
        pass
    return None


def focused_monitor():
    """Return ``(x, y, width, height)`` with a safe headless fallback."""
    monitors = []
    try:
        result = subprocess.run(
            ["xrandr", "--query"],
            check=True, capture_output=True, text=True, timeout=2.0,
        )
        for line in result.stdout.splitlines():
            if " connected" not in line:
                continue
            match = re.search(r"(\d+)x(\d+)\+(-?\d+)\+(-?\d+)", line)
            if match:
                monitors.append({
                    "x": int(match.group(3)), "y": int(match.group(4)),
                    "width": int(match.group(1)), "height": int(match.group(2)),
                    "primary": " connected primary " in f" {line} ",
                })
    except (OSError, subprocess.SubprocessError):
        pass

    if not monitors:
        return 0, 0, 1920, 1080

    center = _x11_active_window_center()
    if center is not None:
        for monitor in monitors:
            if (
                monitor["x"] <= center[0] < monitor["x"] + monitor["width"]
                and monitor["y"] <= center[1] < monitor["y"] + monitor["height"]
            ):
                return tuple(monitor[key] for key in ("x", "y", "width", "height"))

    monitor = next((item for item in monitors if item["primary"]), monitors[0])
    return tuple(monitor[key] for key in ("x", "y", "width", "height"))


def default_sim_window():
    """Return the existing modest default Stonefish window dimensions."""
    _, _, width, height = focused_monitor()
    sim_width = min(1280, max(800, int(width * 0.50)))
    sim_height = min(720, max(300, min(int(sim_width * 9 / 16), height)))
    return sim_width, sim_height


def default_preview_window():
    """Return the existing default annotated preview dimensions."""
    _, _, width, height = focused_monitor()
    sim_width, _ = default_sim_window()
    return max(160, width - sim_width), max(90, height // 2)
