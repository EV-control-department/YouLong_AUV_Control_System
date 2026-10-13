"""状态指示灯的测试（规范：``docs/interfaces/indicator_light.md``）。

分两层：

* **纯函数层**（``indicator_pattern``）—— 灯语表的可执行版本，不需要 ROS、不需要节点，
  也不需要等 ``basic_motion``。
* **节点层** —— 两个互斥发射路径（`OCCUPIED` 直通 / 其它状态由 20 Hz 定时器接管）、
  首次告警、`SKIPPED` 到时交还、以及"每个任务开头置 OCCUPIED"。

这一版**没有任何任务实装状态机**，所以这些测试只覆盖基础设施本身。
"""

from __future__ import annotations

import time

import pytest
import rclpy

from uv_task.task_runner import (
    DETECT_DURATION,
    FAST_HALF,
    INDICATOR_ALIGNING,
    INDICATOR_OCCUPIED,
    INDICATOR_OFF,
    INDICATOR_PRESTART,
    INDICATOR_SEARCHING,
    INDICATOR_SKIPPED,
    INDICATOR_TRANSIT,
    INDICATOR_WORKING,
    SKIPPED_DURATION,
    SLOW_HALF,
    indicator_pattern,
)


# --------------------------------------------------------------------------- #
# 纯函数：灯语表
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("state, elapsed, expected", [
    # OCCUPIED / PRESTART：定时器不驱动（灯归任务处理器管）
    (INDICATOR_OCCUPIED, 0.0, None),
    (INDICATOR_OCCUPIED, 9.9, None),
    (INDICATOR_PRESTART, 0.0, None),
    # 灭 / 常亮：静态，不随时间变
    (INDICATOR_OFF, 0.0, 'off'),
    (INDICATOR_OFF, 9.9, 'off'),
    (INDICATOR_WORKING, 0.0, 'green'),
    (INDICATOR_WORKING, 9.9, 'green'),
    # 慢闪：前半亮 / 后半灭（1 Hz）
    (INDICATOR_SEARCHING, 0.0, 'yellow'),
    (INDICATOR_SEARCHING, SLOW_HALF * 0.5, 'yellow'),
    (INDICATOR_SEARCHING, SLOW_HALF * 1.5, 'off'),
    (INDICATOR_ALIGNING, 0.0, 'green'),
    (INDICATOR_ALIGNING, SLOW_HALF * 1.5, 'off'),
    # 交替慢闪：两个颜色各占一个半周期，中间不留灭
    (INDICATOR_TRANSIT, 0.0, 'yellow'),
    (INDICATOR_TRANSIT, SLOW_HALF * 1.5, 'green'),
    (INDICATOR_TRANSIT, SLOW_HALF * 2.5, 'yellow'),
    (INDICATOR_TRANSIT, SLOW_HALF * 3.5, 'green'),
    # 快闪：半周期是慢闪的 1/5（5 Hz）
    (INDICATOR_SKIPPED, 0.0, 'red'),
    (INDICATOR_SKIPPED, FAST_HALF * 1.5, 'off'),
    (INDICATOR_SKIPPED, FAST_HALF * 2.5, 'red'),
])
def test_pattern_table(state, elapsed, expected) -> None:
    assert indicator_pattern(state, elapsed) == expected


def test_transit_alternation_never_shows_off() -> None:
    """黄绿交替是**交替**，不是"闪" —— 任何时刻都不该是灭的。"""
    for i in range(40):
        color = indicator_pattern(INDICATOR_TRANSIT, i * 0.05)
        assert color in ('yellow', 'green'), f't={i * 0.05}: {color}'


def test_detect_only_overrides_transit() -> None:
    """`DETECT` 是 TRANSIT 的子事件：绿快闪。"""
    assert indicator_pattern(INDICATOR_TRANSIT, 0.0, detect_active=True) == 'green'
    assert indicator_pattern(INDICATOR_TRANSIT, FAST_HALF * 1.5, detect_active=True) == 'off'
    # 其它状态传 detect_active 也不受影响
    assert indicator_pattern(INDICATOR_WORKING, 0.0, detect_active=True) == 'green'
    assert indicator_pattern(INDICATOR_SEARCHING, 0.0, detect_active=True) == 'yellow'
    assert indicator_pattern(INDICATOR_OCCUPIED, 0.0, detect_active=True) is None


def test_fast_is_five_times_the_slow_rate() -> None:
    """规范 §4.1：慢闪 1 Hz、快闪 5 Hz。"""
    assert SLOW_HALF == pytest.approx(0.5)
    assert FAST_HALF == pytest.approx(0.1)


# --------------------------------------------------------------------------- #
# 节点层
# --------------------------------------------------------------------------- #
class _Recorder:
    def __init__(self) -> None:
        self.bytes: list[int] = []

    def publish(self, msg) -> None:
        self.bytes.append(int(msg.data))


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope="module")
def node(ros_context):
    from uv_task.task_runner import TaskRunnerNode

    n = TaskRunnerNode()      # __init__ 里有 5 s wait_for_server，所以共用
    yield n
    n.destroy_node()


@pytest.fixture
def light(node):
    """把灯光 publisher 换成记录器，并把状态机复位到默认 OCCUPIED。"""
    recorder = _Recorder()
    original = node.pub_light
    node.pub_light = recorder
    node._indicator_state = INDICATOR_OCCUPIED
    node._indicator_since = time.monotonic()
    node._indicator_detect_until = 0.0
    node._indicator_last_byte = None
    node._indicator_legacy_warned.clear()
    yield recorder
    node.pub_light = original


@pytest.fixture
def warns(node):
    captured = []
    original = node.get_logger().warn
    node.get_logger().warn = lambda msg: captured.append(str(msg))
    yield captured
    node.get_logger().warn = original


def test_default_state_is_occupied(node) -> None:
    """默认必须兼容：节点起来就是 OCCUPIED（灯交给任务）。"""
    assert node._indicator_state == INDICATOR_OCCUPIED


def test_occupied_passes_set_light_through(node, light) -> None:
    """OCCUPIED 下 set_light 直通 —— 这就是"不改变任何现有行为"。"""
    node.set_light(node.LIGHT_RED, 'probe')
    assert light.bytes == [node.LIGHT_RED]
    node.light_off()
    assert light.bytes == [node.LIGHT_RED, node.LIGHT_OFF]


def test_non_occupied_drops_set_light_and_warns_once(node, light, warns) -> None:
    """状态机接管后，任务侧的 set_light 必须被丢弃，且只告警一次（否则刷爆日志）。"""
    node.set_indicator(INDICATOR_WORKING)
    before = len(light.bytes)

    node.set_light(node.LIGHT_RED, 'legacy-site')
    node.set_light(node.LIGHT_RED, 'legacy-site')   # 同一调用点第二次
    node.light_off()                                 # 另一调用点

    assert len(light.bytes) == before, 'set_light 不该在状态机接管时发出去'
    assert len(warns) == 2, f'应按调用点各告警一次，实际 {warns}'
    assert all('set_indicator' in w for w in warns), warns


def test_set_indicator_is_idempotent(node) -> None:
    """重复设置同一状态不重启动画相位（否则灯语会被反复复位）。"""
    node.set_indicator(INDICATOR_SEARCHING)
    first_since = node._indicator_since
    time.sleep(0.02)
    node.set_indicator(INDICATOR_SEARCHING)
    assert node._indicator_since == first_since


def test_indicate_detect_only_works_in_transit(node) -> None:
    node.set_indicator(INDICATOR_SEARCHING)
    assert node.indicate_detect() is False
    node.set_indicator(INDICATOR_TRANSIT)
    assert node.indicate_detect() is True
    assert node._indicator_detect_until > time.monotonic()


def test_tick_does_not_publish_in_occupied_or_prestart(node, light) -> None:
    """这两个状态下定时器必须完全让开，否则会和任务的直通点灯打架。"""
    for state in (INDICATOR_OCCUPIED, INDICATOR_PRESTART):
        node._indicator_state = state
        node._indicator_last_byte = None
        light.bytes.clear()
        for _ in range(10):
            node._indicator_tick()
        assert light.bytes == [], f'{state} 下定时器不该发灯'


def test_tick_publishes_only_on_change(node, light) -> None:
    """静态灯语（常亮/灭）只发一次，否则 20 Hz 白刷。"""
    node.set_indicator(INDICATOR_WORKING)
    light.bytes.clear()
    for _ in range(10):
        node._indicator_tick()
    assert light.bytes == [node.LIGHT_GREEN], light.bytes


def test_tick_produces_both_halves_of_a_slow_blink(node, light) -> None:
    node.set_indicator(INDICATOR_SEARCHING)
    light.bytes.clear()
    node._indicator_since = time.monotonic()                  # 前半：亮
    node._indicator_tick()
    node._indicator_since = time.monotonic() - SLOW_HALF * 1.2  # 后半：灭
    node._indicator_tick()
    assert light.bytes == [node.LIGHT_YELLOW, node.LIGHT_OFF], light.bytes


def test_skipped_hands_back_to_occupied_after_three_seconds(node, light) -> None:
    """红快闪 3 s 后交还 OCCUPIED（规范 §5 的覆盖结束）。"""
    node.set_indicator(INDICATOR_SKIPPED)
    light.bytes.clear()

    node._indicator_since = time.monotonic() - (SKIPPED_DURATION - 0.5)
    node._indicator_tick()
    assert node._indicator_state == INDICATOR_SKIPPED, '不到 3 s 不该交还'
    assert node.LIGHT_RED in light.bytes or node.LIGHT_OFF in light.bytes

    node._indicator_since = time.monotonic() - (SKIPPED_DURATION + 0.1)
    node._indicator_tick()
    assert node._indicator_state == INDICATOR_OCCUPIED


def test_every_task_starts_in_occupied(node) -> None:
    """**目前唯一的接入点**：每个任务开头把灯交还给任务处理器。

    这保证未迁移的任务行为与改造前完全一致，也让任务可以假设"灯是我的"。
    """
    node.task_map['__probe__'] = lambda p: True
    try:
        node.set_indicator(INDICATOR_WORKING)
        assert node._execute_task('__probe__', {}) is True
        assert node._indicator_state == INDICATOR_OCCUPIED
    finally:
        del node.task_map['__probe__']


def test_detect_duration_default_matches_the_spec() -> None:
    assert DETECT_DURATION > 0
