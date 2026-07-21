"""Unit tests for WUURC sector selection and dynamic travel targets."""

import pytest

from uv_task.task_runner import TaskRunnerNode


class _TaskRunnerShape:
    """Minimal TaskRunner-shaped object; no ROS node/DDS is required."""

    CLASS_YELLOW_SECTOR = TaskRunnerNode.CLASS_YELLOW_SECTOR
    CLASS_GREEN_SECTOR = TaskRunnerNode.CLASS_GREEN_SECTOR
    CLASS_RED_SECTOR = TaskRunnerNode.CLASS_RED_SECTOR
    CMD_LED_YELLOW = TaskRunnerNode.CMD_LED_YELLOW
    CMD_LED_GREEN = TaskRunnerNode.CMD_LED_GREEN
    CMD_LED_RED = TaskRunnerNode.CMD_LED_RED

    _sector_for_aruco = TaskRunnerNode._sector_for_aruco
    _resolve_wtravelxy_target = TaskRunnerNode._resolve_wtravelxy_target

    def __init__(self):
        self._selected_sector = None


@pytest.mark.parametrize(
    ('marker_id', 'expected'),
    [
        (1, ('yellow_sector', 1.75, -2.80, 0, 1)),
        (2, ('yellow_sector', 1.75, -2.80, 0, 1)),
        (3, ('green_sector', 3.50, -2.80, 2, 2)),
        (4, ('green_sector', 3.50, -2.80, 2, 2)),
        (5, ('red_sector', 5.25, -2.80, 1, 3)),
        (6, ('red_sector', 5.25, -2.80, 1, 3)),
    ],
)
def test_aruco_id_selects_the_regulation_sector(marker_id, expected):
    sector = _TaskRunnerShape()._sector_for_aruco(marker_id)
    assert (
        sector['name'], sector['x'], sector['y'], sector['class_id'],
        sector['led_command'],
    ) == expected


def test_unknown_aruco_id_does_not_select_a_sector():
    assert _TaskRunnerShape()._sector_for_aruco(7) is None


def test_wtravelxy_uses_literal_target():
    assert _TaskRunnerShape()._resolve_wtravelxy_target(
        {'x': 5.88, 'y': 2.80}) == (5.88, 2.80)


def test_wtravelxy_uses_selected_sector_target():
    runner = _TaskRunnerShape()
    runner._selected_sector = runner._sector_for_aruco(4)
    assert runner._resolve_wtravelxy_target(
        {'target': 'selected_sector'}) == (3.50, -2.80)


def test_wtravelxy_rejects_sector_move_without_aruco_result():
    with pytest.raises(ValueError, match='No selected sector'):
        _TaskRunnerShape()._resolve_wtravelxy_target(
            {'target': 'selected_sector'})
