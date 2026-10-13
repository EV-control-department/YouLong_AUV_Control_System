# 状态指示灯设计规范（task_runner）

> **本文件是状态指示灯的语义与灯语唯一事实源。**
> 值定义（哪个数字是什么颜色）以本文 §2 为准；状态划分以 §3 为准；闪烁频率以 §4 为准。
> 实现落在 `workspace_auv/src/uv_task/uv_task/task_runner.py`，由各任务处理器调用其状态接口。
>
> **为什么要有这份规范**：灯是现场唯一不经过屏幕、不经过网络就能读懂 AUV 在干什么的通道。
> 没有规范之前，灯光是各任务处理器**随手**点的（`line_follower` 找线时点红灯、`arrow_surfacer`
> 选中扇区时点对应颜色、`_start_sequence` 用裸数字点 3/1/2），每种用法都合理，
> 但**没有一种能回答"船现在到底在干什么"**。规范的目标是把灯从"事件回显"变成"状态指示"。

---

## 1. 问题的陈述

改造前的灯光使用是**事件驱动、无状态**的：

| 位置 | 触发 | 灯的用法 |
|---|---|---|
| `line_follower:407` | 识别到三角形 | 亮红灯 1 s |
| `line_follower:481` | 识别到正方形 | 绿灯闪 2 次 |
| `arrow_surfacer:546-551` | 选中某个扇区 | 亮该扇区对应的颜色 |
| `task_runner._start_sequence` | 发车前 | 裸数字 3 → 1 → 2 的闪烁序列 |

后果：
- 灯只在**事件发生的那一瞬间**有意义，事件过后是灭的 —— 现场看到灭灯无法区分
  "正在搜索" / "正在转移" / "卡住了" / "已经跑完"
- 同一时刻多个处理器可能各自点灯，**后点的覆盖先点的**（`/zit6/cmd/light` 无仲裁）
- 转移途中（`wtravel` 可能要几十秒）**全程无灯**，现场完全不知道船在移动

本规范改为**状态机驱动**：任一时刻有且只有一个指示灯状态，状态决定灯语。

---

## 2. 值的权威定义

`/zit6/cmd/light`（`std_msgs/UInt8`）的取值：

| 值 | 颜色 | `task_runner` 常量 |
|---|---|---|
| `0` | 灭 | `LIGHT_OFF` |
| `1` | **红** | `LIGHT_RED` |
| `2` | **绿** | `LIGHT_GREEN` |
| `3` | **黄** | `LIGHT_YELLOW` |

此外无其它合法值（固件调试口的 `parseLight` 只接受 `0,1,2,3`）。

> ✅ **曾经的不一致已修（2026-10-13）**：此前 `mcu_sim._light_cb` 的
> `colors = {1:"red", 2:"yellow", 3:"green"}` 与 `topic_contract.md` 的 `1=红 2=黄 3=绿`
> **把 2 和 3 写反了**（`task_runner` 的常量一直是对的，实机也确认 2=绿）。
> 症状：SIL 调试时 `mcu_sim` 打出反的颜色名（发了绿却打印 `Light: yellow`），把人带偏。
> 两处已对调并改为引用本文件。

---

## 3. 状态定义

状态机只有一个维度：**当前在做什么**。任一时刻恰好处于一个状态。

| 状态 | 含义 | 进入条件 | 退出条件 |
|---|---|---|---|
| `OFF` | 未运行 / 已跑完 / 已停止 | 节点启动、任务列表结束、`/task/stop` | 收到 `start` 任务 |
| `PRESTART` | 发车前的人员安全提示 | 收到 `start` 任务 | `START` action 成功返回 |
| `TRANSIT` | 正在转移到下一个任务地点 | 进入任何**位置类**任务（`wtravel*`/`btravel*`/`wmove*`/`bmove*`/`set*`/`navigate`） | 该任务结束 |
| `SEARCHING` | 已在任务点，正在寻找目标 | 任务处理器进入搜索阶段 | 找到目标，或超时 |
| `ALIGNING` | 已找到目标，正在对准/定位 | 任务处理器进入对准阶段 | 对准完成，或超时 |
| `WORKING` | 正在作业（抓取/投放/采样等） | 任务处理器进入作业阶段 | 作业结束，或超时 |
| `SKIPPED` | 该任务超时/失败被跳过 | 任一任务返回失败 | 3 s 后（自动） |

**子事件**（不是独立状态，叠加在当前状态上）：

| 子事件 | 叠加在 | 含义 |
|---|---|---|
| `DETECT` | `TRANSIT` | 转移途中识别到目标物体 |

### 状态迁移

```
                    ┌─────────── 任务列表结束 / stop ───────────┐
                    ▼                                          │
   [OFF] ──start──► [PRESTART] ──START 成功──► [TRANSIT] ──到达任务点──► [SEARCHING]
                                                 ▲                        │
                                                 │                  找到 ─┤
                                                 │                        ▼
                                                 │                   [ALIGNING]
                                                 │                        │
                                                 │                  对准完成
                                                 │                        ▼
                                                 │                   [WORKING]
                                                 │                        │
                                                 └──── 下一个任务 ──── 作业结束
                                                 ▲
                                                 │
                          [SKIPPED] ──3 s 后───┘   （任一任务失败/超时即进入）
```

---

## 4. 灯语规范

灯语由「颜色 + 节奏」两部分组成。节奏是**在主机侧生成**的（见 §6）。

### 4.1 节奏定义

| 代号 | 定义 |
|---|---|
| **常亮** | 持续点亮 |
| **慢闪** | 500 ms 亮 / 500 ms 灭（1 Hz） |
| **快闪** | 100 ms 亮 / 100 ms 灭（5 Hz） |
| **交替慢闪** | 颜色 A 亮 500 ms → 颜色 B 亮 500 ms → 循环（中间不留灭的间隔） |
| **限时** | 灯语只持续指定时长，到时自动交还给状态机 |

### 4.2 状态→灯语

| 状态 | 灯语 | 现场读作 |
|---|---|---|
| `OFF` | **灭** | 没在动，也没在作业 |
| `PRESTART` | 沿用现有发车序列（见 §4.4） | 准备中，别下水 |
| `TRANSIT` | **黄绿交替慢闪** | 正在换位置 |
| `TRANSIT` + `DETECT` | **绿快闪** | 换位置途中看到目标了 |
| `SEARCHING` | **黄慢闪** | 在找东西 |
| `ALIGNING` | **绿慢闪** | 找到了，正在对准 |
| `WORKING` | **绿常亮** | 正在干活 |
| `SKIPPED` | **红快闪，限时 3 s** | 这一步没过，跳过去了 |

> **颜色的语义约定**（便于现场一眼分类）：
> - **黄** = 还在找 / 还在移动（未锁定）
> - **绿** = 已锁定目标，且是好消息（对准中 / 作业中 / 看到目标）
> - **红** = 出问题了
>
> **节奏的语义约定**：
> - **慢** = 稳定状态（会持续一会儿，可以放心观察）
> - **快** = 瞬态 / 需要立刻注意
> - **交替** = 移动中（两个颜色都在变，对应"位置在变"）

### 4.3 组合规则

- 每个状态**唯一对应**一种灯语，不做"同状态多灯语"
- `DETECT` 只在 `TRANSIT` 内生效；`SEARCHING` 里看到目标就是状态迁移到 `ALIGNING`，
  不额外叠加
- `WORKING` 期间**不叠加** `DETECT`（已经在干活了，不需要再报"看到目标"）
- `SKIPPED` 是**覆盖式**的：打断当前灯语，3 s 后交还给"该任务之后应有的状态"

### 4.4 `PRESTART` 的灯语

`PRESTART` 保留现有发车序列（`_start_sequence`），不纳入 §4.2 的节奏体系 —— 它的目的是
**发车前给人员的安全提示**，语义与"状态指示"不同（它要让人注意到"马上要动了"，而不是
告诉人"我在干什么"）。现有序列为：

| 阶段 | 灯 | 时长 | 现场提示 |
|---|---|---|---|
| 准备 | `3`（黄） | 1 s | 做好拔缆准备 |
| 即将发动 | `1`（红） | 0.5 s ×1 | 请拔缆 |
| 倒数 | `2`（绿） | 0.25 s ×7 + 1 s ×2 | 6 秒后启动 / 2 秒后启动 |
| 发车 | 灭 | — | `START` action |

> ⚠️ 此处**必须改用 §2 的常量**（`LIGHT_YELLOW` 等）而不是裸数字 —— 现在写的是
> `set_light(3, ...)` / `set_light(1, ...)` / `set_light(2, ...)`，同一件事有两份事实源，
> 改常量时这段不会跟着改，而它恰恰是最关键的一段。

---

## 5. 优先级与抢占

同一时刻只允许一个灯语在输出，因此需要仲裁规则。优先级从高到低：

| 优先级 | 灯语 | 理由 |
|---|---|---|
| 1 | `SKIPPED`（红快闪 3 s） | 失败必须立刻可见，不可被掩盖 |
| 2 | `WORKING`（绿常亮） | 作业中不允许被视觉子事件打断 |
| 3 | `TRANSIT` + `DETECT`（绿快闪） | 瞬态发现，需要立刻可见 |
| 4 | `TRANSIT` / `SEARCHING` / `ALIGNING` | 常规状态 |
| 5 | `OFF` | 兜底 |

**`SKIPPED` 的处理**：不改变状态机的"当前任务状态"，只是一个 3 s 的**显示层覆盖**。
3 s 结束后，显示层交还给状态机当时的状态（通常是下一个任务的 `TRANSIT`）。

---

## 6. 实现约束

### 6.1 闪烁必须在主机侧生成

`/zit6/cmd/light` 携带的是**静态颜色字节**（固件 `MotionController_Driver::setLightState(state)`
只是把 `state` 原样打包成 UART `LightPacket(0x03)` 发出去），**硬件板不认识"闪烁"这个概念**。
所以所有慢闪/快闪/交替都必须由 task_runner **周期性发布**产生。

- 刷新率：**20 Hz**（50 ms）—— 足以让 100 ms 的快闪有 2 个采样点，同时开销可忽略
  （UInt8，10 Hz DDS 都绰绰有余）
- 实现方式：一个 `create_timer(0.05, ...)`，每次根据 `(当前状态, 子事件, 已持续时长)`
  算出该发什么颜色，再发

### 6.2 单一写者

`/zit6/cmd/light` 的发布者**只有 task_runner 一个**，且**只有状态机一个代码路径**。
任务处理器不得再直接调用 `set_light()`。

> 这是本仓库已经付过两次代价的教训（B19：`/zit6/state/vel` 两个写者；P7：两个 action goal
> 同写 `/zit6/cmd/setpoint`）。灯光同理：`line_follower` 和 `arrow_surfacer` 各自点灯时，
> 谁最后发布谁赢，而且没人知道。

### 6.3 生命周期

- 节点启动 → `OFF`
- `/task/stop` → `OFF`（并停止闪烁定时器）
- 节点退出 → `OFF`（发一次 `0`，与 `basic_motion` 退出时发推力归零同理：
  别把灯留在亮着）
- 任务列表跑完 → `OFF`

---

## 7. API 设计

在 `TaskRunnerNode` 上暴露状态接口，任务处理器只调这两个：

```python
# 状态常量
INDICATOR_OFF       = 'off'        # 灭
INDICATOR_PRESTART  = 'prestart'   # 发车序列
INDICATOR_TRANSIT   = 'transit'    # 黄绿交替慢闪
INDICATOR_SEARCHING = 'searching'  # 黄慢闪
INDICATOR_ALIGNING  = 'aligning'   # 绿慢闪
INDICATOR_WORKING   = 'working'    # 绿常亮
INDICATOR_SKIPPED   = 'skipped'    # 红快闪 3 s（覆盖式）

def set_indicator(self, state: str) -> None:
    """切换指示灯状态（幂等：重复设置同一状态不重启动画）。"""

def indicate_detect(self, duration: float = 2.0) -> None:
    """TRANSIT 期间"识别到目标物体"的一次性事件 → 绿快闪 duration 秒。

    Only meaningful in TRANSIT (§4.3)；其它状态下忽略。
    """
```

**谁调什么**（改造后）：

| 调用方 | 调用 |
|---|---|
| `run_task_list`（每个任务开始前） | `set_indicator(TRANSIT)` —— 位置类任务 |
| `run_task_list`（任务返回失败） | `set_indicator(SKIPPED)` |
| `_start_sequence` | `set_indicator(PRESTART)`（灯语仍由该函数自己跑，§4.4）|
| `line_follower` 搜索阶段 | `set_indicator(SEARCHING)` |
| `line_follower` / `arrow_surfacer` 对准阶段 | `set_indicator(ALIGNING)` |
| 任意任务处理器进入作业阶段 | `set_indicator(WORKING)` |
| 转移途中的目标检测 | `indicate_detect()` |

---

## 8. 与现有代码的差异（迁移清单）

> 本规范是**目标设计**。下表是现有代码需要相应调整的地方；「是否已实施」见 `docs/roadmap.md`。

| 现状 | 规范要求 | 处理 |
|---|---|---|
| `_start_sequence` 用裸数字 `3/1/2` | 用 §2 常量 | 改成 `LIGHT_YELLOW` / `LIGHT_RED` / `LIGHT_GREEN` |
| `line_follower:407` 直接 `set_light(LIGHT_RED, ...)` | 不允许直接点灯 | 改为 `set_indicator(...)`；"识别到三角形"是状态迁移到 `ALIGNING`，不是点红灯 |
| `line_follower:481` 直接 `set_light(LIGHT_GREEN, ...)` 闪 2 次 | 同上 | 改为状态迁移 |
| `arrow_surfacer:546-551` 按扇区颜色点灯 | 同上 | 改为 `set_indicator(WORKING)`；扇区颜色是任务的**业务**输出，不是指示灯语义 —— 若确实需要"按扇区亮不同颜色"，应当作**独立需求**重新讨论，因为它与状态指示抢同一个通道 |
| `mcu_sim._light_cb` 的 2/3 颜色名反了 | 与 §2 一致 | 对调 2/3 |
| `topic_contract.md:99` 写 `1=红 2=黄 3=绿` | 与 §2 一致 | 改成 `1=红 2=绿 3=黄` |

---

## 9. 待决问题

1. **`PRESTART` 是否纳入状态机**：现在是 §4.4 的独立序列。若纳入，需要为它定义 §4.2 意义上的
   灯语，但它的"给人员安全提示"语义与状态指示不同，建议保持独立
2. **`SKIPPED` 之后**：直接进入下一个任务的 `TRANSIT`，还是先回 `SEARCHING`？取决于任务列表
   的组织方式（现在 `run_task_list` 是纯顺序执行）
3. **`arrow_surfacer` 的扇区颜色**：它与状态指示争用同一个通道（§8）。要么放弃按扇区亮色，
   要么为它定义优先级/时隙
4. **是否需要回读**：`ZitStatus` 没有灯光字段，所以**没有任何回读**，无法确认灯真的亮了。
   若现场需要确认，得在硬件侧加回读

---

## 10. 现场调试

```bash
# 直接看灯的状态字节（唯一写者是 task_runner）
ros2 topic echo /zit6/cmd/light

# SIL 下 mcu_sim 会把字节翻成颜色名 —— 但注意它现在 2/3 反了（§2），
# 所以调试时以 task_runner 的 value= 日志为准
ros2 topic echo /zit6/cmd/light & ros2 topic echo /zit6/state/status
```

SIL 里**没有**任何灯光视觉反馈（`mcu_sim` 只打日志、场景 `xunyun_fixed.scn` 无灯光元素、
GUI 不显示灯光状态），所以验证灯语只能靠：① `ros2 topic echo /zit6/cmd/light` 看字节序列；
② 真机上直接看灯。**改灯语后至少在真机上确认一遍**。
