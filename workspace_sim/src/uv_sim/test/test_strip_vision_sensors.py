"""Guard ``strip_vision_sensors.py``: a stripped scene must still parse.

``stonefish_simulator_nogpu`` refuses any scene with a camera, so this tool is the
gateway to every headless run -- and a scene it corrupts fails late and confusingly
("Parsing of scenario file ... failed", with the real cause only in the parser log).

That is exactly what the first version of ``--keep-camera`` did (2026-09-16): it
used one "skipping" flag for both dropping and keeping, so a kept camera lost its
whole body and the scene became unparseable -- while ``grep -c type="camera"``
still reported the expected number of cameras.  Hence these tests parse the XML
instead of counting lines.
"""

from __future__ import annotations

import importlib.util
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
TOOL = REPO_ROOT / "workspace_sim" / "tools" / "strip_vision_sensors.py"
SCENE = REPO_ROOT / "workspace_sim" / "src" / "stonefish_ros2" / "Data" / "xunyun_fixed.scn"


def _load_tool():
    spec = importlib.util.spec_from_file_location("strip_vision_sensors", TOOL)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules["strip_vision_sensors"] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def tool():
    return _load_tool()


def _sensors(text: str) -> list:
    """Sensor elements of a scene parsed from a string (raises on bad XML)."""
    root = ET.fromstring(text)
    return [element for element in root.iter() if element.tag.endswith("sensor")]


def _names(elements, kind: str | None = None) -> set[str]:
    return {
        element.get("name")
        for element in elements
        if kind is None or element.get("type") == kind
    }


def test_scene_inventory_is_as_expected(tool) -> None:
    """The fixture itself: 4 cameras + the physics sensors."""
    text = SCENE.read_text(encoding="utf-8")
    sensors = _sensors(text)
    assert _names(sensors, "camera") == {
        "front_cam_left", "front_cam_right", "down_cam_left", "down_cam_right"
    }
    assert len(sensors) == 8


def test_strip_all_keeps_the_other_sensors(tool) -> None:
    text = SCENE.read_text(encoding="utf-8")
    stripped, removed, kept = tool.strip_camera_sensors(text)
    assert (removed, kept) == (4, 0)

    sensors = _sensors(stripped)                      # parses -> well formed
    assert _names(sensors, "camera") == set()
    assert _names(sensors) == _names(_sensors(text)) - _names(_sensors(text), "camera")


def test_keep_camera_retains_the_block_body(tool) -> None:
    """``--keep-camera down_cam`` drops the front pair and keeps the down pair.

    Regression: the body of each kept camera must be copied, not skipped.
    """
    text = SCENE.read_text(encoding="utf-8")
    stripped, removed, kept = tool.strip_camera_sensors(text, "down_cam")
    assert (removed, kept) == (2, 2)

    sensors = _sensors(stripped)
    assert _names(sensors, "camera") == {"down_cam_left", "down_cam_right"}
    assert _names(sensors) == _names(_sensors(text)) - {
        "front_cam_left", "front_cam_right"
    }
    # the kept cameras' specs survive, i.e. the block body was not dropped
    assert 'resolution_x="1920"' in stripped
    assert 'horizontal_fov="87.19"' in stripped
    assert stripped.count("</sensor>") == text.count("</sensor>") - 2


def test_keep_camera_with_unknown_name_strips_everything(tool) -> None:
    text = SCENE.read_text(encoding="utf-8")
    stripped, removed, kept = tool.strip_camera_sensors(text, "no_such_camera")
    assert (removed, kept) == (4, 0)
    assert _names(_sensors(stripped), "camera") == set()
