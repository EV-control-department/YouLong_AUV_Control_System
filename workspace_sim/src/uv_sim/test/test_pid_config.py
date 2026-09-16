"""PID gains must come from one external file -- and only from there.

Before 2026-09-15 the gains lived in two hardcoded copies inside the monolith's
``_init_full`` (a dict used by ``/zit6/get_params`` and a set of ``_Pid`` objects
used by the controllers).  They had drifted: the service reported ``pos.z.ki=30``
/ ``vel.x.kp=300`` while the running controllers used ``80`` / ``600``, and the
yaw-cascade inner loop was invisible to the service altogether.

These tests pin the contract as it stands after T5, where the loader, the service
view and the live controllers are all built by ``uv_sim.pid_config`` and
``uv_sim.mcu_control``:

* ``config/sim_pid.yaml`` ships with the package and covers every channel;
* the dict behind the service and the live ``Pid`` objects come from the same file,
  so they can never disagree;
* pointing ``pid_config_file`` at another file actually changes the gains
  (i.e. the file is read, not decorative);
* the ``/zit6/get_params`` response is built from that same dict;
* a missing or broken file degrades to the built-in defaults instead of taking the
  SIL stack down.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import rclpy
import yaml
from rclpy.parameter import Parameter

from uv_sim.mcu_control import McuController
from uv_sim.mcu_sim import McuSimNode
from uv_sim.pid_config import (
    DEFAULT_PID_PARAMS,
    PID_PARAM_ATTRIBUTES,
    PID_PARAM_PATHS,
    PID_REQUIRED_FIELDS,
    load_pid_params,
    resolve_pid_config_path,
)

EXPECTED_CHANNELS = 9


def _packaged_config() -> Path:
    path = resolve_pid_config_path()
    assert path is not None, "no sim_pid.yaml resolvable (share/ nor source tree)"
    return path


def test_packaged_config_exists_and_covers_every_channel() -> None:
    raw = yaml.safe_load(_packaged_config().read_text(encoding="utf-8"))

    assert isinstance(raw, dict)
    assert set(raw) == set(PID_PARAM_PATHS), (
        "sim_pid.yaml must define exactly the known channels (got "
        f"{sorted(raw)} vs {sorted(PID_PARAM_PATHS)})"
    )
    assert len(PID_PARAM_PATHS) == EXPECTED_CHANNELS
    for channel, gains in raw.items():
        for field in PID_REQUIRED_FIELDS:
            assert field in gains, f"{channel} is missing {field}"


def test_defaults_cover_every_channel() -> None:
    assert set(DEFAULT_PID_PARAMS) == set(PID_PARAM_PATHS)
    assert set(PID_PARAM_ATTRIBUTES) == set(PID_PARAM_PATHS)


def test_loader_falls_back_when_file_is_absent(tmp_path: Path) -> None:
    params = load_pid_params(tmp_path / "does_not_exist.yaml")
    assert params == DEFAULT_PID_PARAMS


def test_loader_reports_but_survives_broken_yaml(tmp_path: Path) -> None:
    broken = tmp_path / "broken.yaml"
    broken.write_text("chassis.pid.pos.x: [unclosed\n", encoding="utf-8")
    assert load_pid_params(broken) == DEFAULT_PID_PARAMS

    partial = tmp_path / "partial.yaml"
    partial.write_text(
        "chassis.pid.vel.x: {kp: 123.0}\nunknown.channel: {kp: 1.0}\n",
        encoding="utf-8",
    )
    params = load_pid_params(partial)
    assert params["chassis.pid.vel.x"]["kp"] == 123.0            # override applied
    assert params["chassis.pid.vel.x"]["ki"] == DEFAULT_PID_PARAMS["chassis.pid.vel.x"]["ki"]
    assert params["chassis.pid.pos.x"] == DEFAULT_PID_PARAMS["chassis.pid.pos.x"]


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _node(ros_context, name: str, config: Path | None = None) -> McuSimNode:
    overrides = []
    if config is not None:
        overrides.append(Parameter("pid_config_file", value=str(config)))
    return McuSimNode(node_name=name, parameter_overrides=overrides or None)


def test_service_view_and_live_objects_agree(ros_context) -> None:
    node = _node(ros_context, "mcu_sim_pid_default")
    try:
        controller = node._controller
        assert len(controller.pids) == EXPECTED_CHANNELS
        for channel, pid in controller.pids.items():
            gains = controller.pid_params[channel]
            assert (pid.kp, pid.ki, pid.kd, pid.i_limit) == (
                gains["kp"], gains["ki"], gains["kd"], gains["i_limit"],
            ), f"{channel}: service view and live PID disagree"
            assert pid.z_thrust_ff == gains.get("z_thrust_ff", 0.0)

        # the yaw-cascade inner loop is exposed too
        assert "chassis.pid.pos.yaw_rate" in controller.pid_params
    finally:
        node.destroy_node()


def test_controller_attributes_match_the_channel_map() -> None:
    """``PID_PARAM_ATTRIBUTES`` is the only place the attribute names come from."""
    controller = McuController(
        {channel: dict(gains) for channel, gains in DEFAULT_PID_PARAMS.items()}
    )
    for channel, attribute in PID_PARAM_ATTRIBUTES.items():
        assert getattr(controller, attribute) is controller.pids[channel], channel


def test_external_file_actually_drives_the_gains(ros_context, tmp_path: Path) -> None:
    custom = tmp_path / "tuned.yaml"
    custom.write_text(
        "chassis.pid.vel.x: {kp: 111.0, ki: 22.0, kd: 3.0, i_limit: 44.0}\n"
        "chassis.pid.pos.z: {kp: 555.0, ki: 66.0, kd: 7.0, i_limit: 88.0, z_thrust_ff: 9.0}\n",
        encoding="utf-8",
    )
    node = _node(ros_context, "mcu_sim_pid_custom", custom)
    try:
        controller = node._controller
        assert controller.pids["chassis.pid.vel.x"].kp == 111.0
        assert controller.pids["chassis.pid.vel.x"].ki == 22.0
        assert controller.pid_params["chassis.pid.vel.x"]["i_limit"] == 44.0

        assert controller.pids["chassis.pid.pos.z"].kp == 555.0
        assert controller.pids["chassis.pid.pos.z"].z_thrust_ff == 9.0

        # channels absent from the custom file keep the packaged values
        packaged = load_pid_params(_packaged_config())
        assert controller.pids["chassis.pid.pos.y"].kp == packaged["chassis.pid.pos.y"]["kp"]
    finally:
        node.destroy_node()


def test_get_params_service_answers_from_the_live_dict(ros_context) -> None:
    """The service must be a view of the running gains, not a second copy."""
    import json

    class _Request:
        paths: list[str] = []

    class _Response:
        success = False
        config_json = ""

    node = _node(ros_context, "mcu_sim_pid_service")
    try:
        response = node._get_params_cb(_Request(), _Response())
        assert response.success is True
        payload = json.loads(response.config_json)
        assert set(payload) == set(PID_PARAM_PATHS)
        assert payload == node._controller.pid_params

        class _Subset(_Request):
            paths = ["chassis.pid.pos.z", "chassis.pid.unknown"]

        subset = json.loads(node._get_params_cb(_Subset(), _Response()).config_json)
        assert set(subset) == {"chassis.pid.pos.z"}
    finally:
        node.destroy_node()
