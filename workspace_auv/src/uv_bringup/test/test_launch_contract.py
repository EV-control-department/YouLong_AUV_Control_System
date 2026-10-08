"""Static contracts for bringup profiles and component defaults."""

import ast
from pathlib import Path
from types import SimpleNamespace

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
LAUNCH_ROOT = PACKAGE_ROOT / "launch"
AUV_SOURCE_ROOT = PACKAGE_ROOT.parent
SIM_SOURCE_ROOT = PACKAGE_ROOT.parents[2] / "workspace_sim" / "src"
REAL_PROFILE_ROOT = PACKAGE_ROOT / "config" / "profiles" / "real"
SIM_PROFILE_ROOT = SIM_SOURCE_ROOT / "uv_sim_bringup" / "config" / "profiles"


def _source(name):
    return (LAUNCH_ROOT / name).read_text(encoding="utf-8")


def _profile(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_formal_mode_entries_exist():
    for name in ("real.launch.py", "readiness.launch.py"):
        assert (LAUNCH_ROOT / name).is_file()
    assert not (AUV_SOURCE_ROOT / "auv_description" / "urdf" /
                "auv_sim.urdf").exists()


def test_real_mode_delegates_only_to_startup_coordinator():
    tree = ast.parse(_source("real.launch.py"), filename="real.launch.py")
    node_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
    ]
    assert len(node_calls) == 1
    keywords = {item.arg: item.value for item in node_calls[0].keywords}
    assert ast.literal_eval(keywords["package"]) == "uv_bringup"
    assert ast.literal_eval(keywords["executable"]) == "real_startup"


def test_real_startup_contract_is_staged_and_task_control_is_external():
    startup = (PACKAGE_ROOT / "uv_bringup" / "real_startup.py").read_text(
        encoding="utf-8")
    task_launch = (AUV_SOURCE_ROOT / "uv_task" / "launch" /
                   "task_launch.py").read_text(encoding="utf-8")
    task_runner = (AUV_SOURCE_ROOT / "uv_task" / "uv_task" /
                   "task_runner.py").read_text(encoding="utf-8")
    assert "_start_core()" in startup
    assert "_start_motion_component()" in startup
    assert "_start_camera_perception()" in startup
    assert "_start_navigation()" in startup
    startup_stages = (
        "self._start_core()",
        "self._start_motion_component()",
        "self._start_camera_perception()",
        "self._start_navigation()",
    )
    startup_positions = [startup.index(stage) for stage in startup_stages]
    assert startup_positions == sorted(startup_positions)
    assert "startup_mode" in startup
    assert "Initial component decisions" in startup
    assert "no later phase will be started" in startup
    assert "SignalHandlerOptions.NO" in startup
    assert "except ImportError" in startup
    assert "rclpy.get_global_executor()" in startup
    assert "_terminate_owned_process_groups" in startup
    assert "start_new_session=True" in startup
    for control_or_task in (
            "MISSION_STATUS", "MISSION_RUN", "TaskStatus", "RunTask",
            "BASIC_MOTION_SAFE_STOP", "send_goal_async", "task_launch.py",
            "enable_task"):
        assert control_or_task not in startup
    assert "origin reset" not in startup
    assert "sigterm_timeout='22'" in _source("real.launch.py")
    assert "auto_start" in task_launch
    assert '"auto_start", default_value="true"' in task_launch
    assert "node._auto_start" in task_runner
    assert "declare_parameter('auto_start', True)" in task_runner
    assert "'auto_start:=false'" not in startup
    real_launch = _source("real.launch.py")
    assert "LaunchConfiguration('enable_task')" not in real_launch
    assert "LaunchConfiguration('mission_file')" not in real_launch


def test_bringup_uses_the_invoking_terminal():
    launch_files = list(LAUNCH_ROOT.glob("*.py"))
    forbidden = (
        "t" + "mux",
        "x" + "term",
        "use_" + "x" + "term",
        "terminal_" + "prefix",
        "process_" + "prefix",
    )
    for path in launch_files:
        source = path.read_text(encoding="utf-8").lower()
        assert not any(token in source for token in forbidden), path


def test_missing_odom_health_only_observes_and_can_become_ready():
    source = PACKAGE_ROOT / 'uv_bringup/real_startup.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'RealStartupManager')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
               and n.name in ('_observe_phase', 'monitor')]
    namespace = {'time': SimpleNamespace(monotonic=lambda: 2.0)}
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), 'exec'), namespace)
    ready = [False]
    manager = SimpleNamespace(
        _observed_phases={}, component_state={}, children=[],
        _last_monitor=0.0, _dashboard=lambda: None)
    namespace['_observe_phase'](manager, 'perception gate', lambda: ready[0],
                                'odom-based tracks')
    assert manager.component_state['perception gate'].startswith('WAITING')
    ready[0] = True
    namespace['monitor'](manager)
    assert manager.component_state['perception gate'] == 'READY'


def test_top_level_launch_modes_use_profile_and_no_preset_selector():
    launch_files = [
        LAUNCH_ROOT / "real.launch.py",
        SIM_SOURCE_ROOT / "uv_sim_bringup" / "launch" / "sim.launch.py",
        SIM_SOURCE_ROOT / "uv_sim_bringup" / "launch" / "hil.launch.py",
    ]
    for path in launch_files:
        source = path.read_text(encoding="utf-8")
        assert "declare_profile(" in source
        assert "declare_launch_preset" not in source
        assert "preset" not in source.lower()


def test_real_profiles_have_expected_startup_combinations():
    expected_names = {"default.yaml", "record.yaml", "debug.yaml", "task.yaml"}
    assert {path.name for path in REAL_PROFILE_ROOT.glob("*.yaml")} == expected_names
    loaded = {path.stem: _profile(path) for path in REAL_PROFILE_ROOT.glob("*.yaml")}
    for name, payload in loaded.items():
        assert payload["runtime"] == "real"
        assert payload["profile"] == name
        assert payload["arguments"].get("enable_nav") is False

    record = loaded["record"]["arguments"]
    assert record["enable_camera"] is True
    assert record["enable_ai"] is False
    assert record["enable_motion"] is True
    assert "enable_task" not in record
    assert record["record_mode"] == "raw"
    assert record["record_session"] is True

    debug = loaded["debug"]["arguments"]
    assert debug["enable_camera"] is True
    assert debug["enable_ai"] is True
    assert debug["enable_motion"] is True
    assert "enable_task" not in debug
    assert debug["record_mode"] == "go2rtc"
    assert debug["enable_stream"] is True

    task = loaded["task"]["arguments"]
    assert task["enable_camera"] is True
    assert task["enable_ai"] is True
    assert task["enable_motion"] is True
    assert "enable_task" not in task
    assert task["record_mode"] == "go2rtc"

    default = loaded["default"]["arguments"]
    assert default["enable_camera"] is False


def test_sim_and_hil_profile_combinations_remain_available():
    expected = {
        "sim": {
            "record": {
                "enable_ai": False, "enable_motion": True,
                "enable_nav": False, "enable_task": False,
                "gpu": True, "gpu_backend": "auto",
                "enable_stream": False, "enable_evaluation": False,
                "record_session": True, "record_mode": "raw",
                "record_use_sim_time": True,
            },
            "debug": {
                "enable_ai": True, "enable_motion": True,
                "enable_nav": False, "enable_task": False,
                "gpu": True, "gpu_backend": "auto",
                "enable_stream": True, "enable_evaluation": True,
                "record_session": True, "record_mode": "go2rtc",
                "go2rtc_stream_mode": "unannotated",
                "go2rtc_video_format": "jpeg",
                "record_use_sim_time": True,
            },
            "task": {
                "enable_ai": True, "enable_motion": True,
                "enable_nav": True, "enable_task": True,
                "gpu": True, "gpu_backend": "auto",
                "enable_stream": True, "enable_evaluation": True,
                "record_session": True, "record_mode": "go2rtc",
                "go2rtc_stream_mode": "unannotated",
                "go2rtc_video_format": "jpeg",
                "record_use_sim_time": True,
            },
        },
        "hil": {
            "record": {
                "enable_ai": False, "enable_motion": False,
                "enable_nav": False, "enable_task": False,
                "gpu": True, "gpu_backend": "auto",
                "enable_stream": False, "record_session": True,
                "record_mode": "raw", "record_use_sim_time": True,
            },
            "debug": {
                "enable_ai": True, "enable_motion": False,
                "enable_nav": False, "enable_task": False,
                "gpu": True, "gpu_backend": "auto",
                "enable_stream": True, "record_session": True,
                "record_mode": "go2rtc",
                "go2rtc_stream_mode": "unannotated",
                "go2rtc_video_format": "jpeg",
                "record_use_sim_time": True,
            },
            "task": {
                "enable_ai": True, "enable_motion": True,
                "enable_nav": True, "enable_task": True,
                "gpu": True, "gpu_backend": "auto",
                "enable_stream": True, "record_session": True,
                "record_mode": "go2rtc",
                "go2rtc_stream_mode": "unannotated",
                "go2rtc_video_format": "jpeg",
                "record_use_sim_time": True,
            },
        },
    }
    for runtime in ("sim", "hil"):
        root = SIM_PROFILE_ROOT / runtime
        assert {path.name for path in root.glob("*.yaml")} == {
            "default.yaml", "record.yaml", "debug.yaml", "task.yaml",
        }
        profiles = {path.stem: _profile(path) for path in root.glob("*.yaml")}
        for name, payload in profiles.items():
            assert payload["runtime"] == runtime
            assert payload["profile"] == name
        assert profiles["default"]["arguments"] == {}
        for name, values in expected[runtime].items():
            assert profiles[name]["arguments"] == values


def test_component_profiles_and_old_pid_files_are_removed():
    component_roots = [
        AUV_SOURCE_ROOT / "uv_camera",
        AUV_SOURCE_ROOT / "uv_hm",
        SIM_SOURCE_ROOT / "uv_sim",
        SIM_SOURCE_ROOT / "uv_sim_bridge",
    ]
    for root in component_roots:
        assert not (root / "config" / "profiles").exists()
        if (root / "setup.py").exists():
            setup = (root / "setup.py").read_text(encoding="utf-8")
            assert "config/profiles" not in setup
            assert "launch_profiles" not in setup

    hm_root = AUV_SOURCE_ROOT / "uv_hm"
    assert (hm_root / "config" / "default.yaml").exists()
    params = _profile(hm_root / "config" / "default.yaml")["/hw_manager"]["ros__parameters"]
    assert "arm_mode" not in params
    assert "heartbeat_rate" not in params
    control_params = _profile(AUV_SOURCE_ROOT / "uv_control" / "config" / "default.yaml")["/basic_motion"]["ros__parameters"]
    assert control_params["arm_mode"] == 1
    assert control_params["heartbeat_rate"] == 15.0
    assert control_params["start_timeout"] == 10.0
    assert control_params["arm_confirmation_timeout"] == 20.0
    assert params["watchdog_timeout"] == 7.0
    assert params["battery_low_threshold"] == 14.0
    hm_setup = (hm_root / "setup.py").read_text(encoding="utf-8")
    assert "config/default.yaml" in hm_setup
    assert "pid_parameters.json" not in hm_setup
    assert "pid_params.yaml" not in hm_setup
    assert not (hm_root / "config" / "pid_parameters.json").exists()
    assert not (hm_root / "config" / "pid_params.yaml").exists()


def test_only_bringup_packages_install_startup_profiles():
    real_setup = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    sim_setup = (SIM_SOURCE_ROOT / "uv_sim_bringup" / "setup.py").read_text(encoding="utf-8")
    assert "config/profiles/real" in real_setup
    assert "config/profiles/sim" in sim_setup
    assert "config/profiles/hil" in sim_setup
    assert "launch_profiles" not in real_setup + sim_setup


def test_component_parameter_file_and_camera_mode_arguments_are_named():
    camera_files = [
        AUV_SOURCE_ROOT / "uv_camera" / "launch" / "camera_launch.py",
        AUV_SOURCE_ROOT / "uv_camera" / "uv_camera" / "driver.py",
        AUV_SOURCE_ROOT / "uv_task" / "launch" / "task_launch.py",
    ]
    camera_source = "\n".join(path.read_text(encoding="utf-8")
                                for path in camera_files)
    assert "camera_mode" in camera_source
    assert "camera_config_profile" not in camera_source

    parameter_launch_files = [
        AUV_SOURCE_ROOT / "uv_control" / "launch" / "control_launch.py",
        AUV_SOURCE_ROOT / "uv_hm" / "launch" / "hardware_launch.py",
        AUV_SOURCE_ROOT / "uv_nav" / "launch" / "navigation_launch.py",
        AUV_SOURCE_ROOT / "uv_planning" / "launch" / "planning_launch.py",
        AUV_SOURCE_ROOT / "uv_task" / "launch" / "task_launch.py",
        SIM_SOURCE_ROOT / "uv_sim" / "launch" / "bridge.launch.py",
        SIM_SOURCE_ROOT / "uv_sim_bridge" / "launch" / "bridge.launch.py",
    ]
    for path in parameter_launch_files:
        source = path.read_text(encoding="utf-8")
        assert "params_file" in source
        assert "profile_params" not in source


def test_uv_sim_scene_selector_is_world_only():
    sim_root = SIM_SOURCE_ROOT / "uv_sim"
    launch = (sim_root / "launch" / "sim.launch.py").read_text(encoding="utf-8")
    scenarios = (sim_root / "uv_sim" / "scenarios.py").read_text(encoding="utf-8")
    assert '"world"' in launch
    assert "PROFILE_ALIASES" not in launch + scenarios
    assert 'DeclareLaunchArgument("profile"' not in launch
