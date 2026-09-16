# 启动入口清单（T4）

> **本文件是"哪个入口启动哪些节点"的唯一事实源**；参数默认值同样列在 `docs/node_parameters.md`
> （那份由 `scripts/check_docs_contract.py` 与源码逐条对拍，本文件不复制默认值）。
>
> 当前阶段：**只做软件在环（SIL）**，HIL / 实机入口保留在册但标 ⏸、不参与验收
> （见 `docs/architecture/sim_real_boundary.md`）。
>
> **书写约定**：本文档引用**旧入口/旧命令时一律写成「包 文件」**，不写 `ros2 launch` 前缀。
> 原因是 `check_docs_contract.py` 的 `launch-refs` 检查要求**文档里出现的每条
> `ros2 launch <包> <文件>.py` 都必须真实存在**（`devlog/` 等历史目录除外）——
> 这条约定让"文档指向已删除入口"从隐性错误变成构建期失败（B6）。

## 1. 入口与所有权

| 入口（包/文件） | 工作区 | 模式 | 启动内容 | 参数 |
|---|---|---|---|---|
| `uv_bringup/launch/real_bringup.py` | workspace_auv | REAL ⏸ | include `uv_hm` + `uv_control` + `uv_perception` + `uv_nav` + `uv_task` | `enable_agent`、`enable_motion`、`enable_ai`、`enable_nav`、`enable_task` |
| `uv_sim_bringup/launch/sim_bringup.py` | workspace_sim | **SIL** | include 引擎 + `uv_sim` + `uv_control` + `uv_perception` + `uv_nav` + `uv_task` | `scenario_desc`、`headless`、`simulation_rate`、`window_res_x/y`、`rendering_quality`、`enable_mcu`、`enable_motion`、`enable_ai`、`enable_nav`、`enable_task` |
| `uv_sim_bringup/launch/hil_bringup.py` | workspace_sim | HIL ⏸ | 同 SIL，但 `sim_bridge` 以 `hil_mode:=true` 运行，并 include `uv_hm` | 同 SIL + `enable_agent`、`serial_dev`、`serial_baud` |
| `uv_hm/launch/hm_launch.py` | workspace_auv | REAL / HIL | `hw_manager` + micro-ROS agent | `enable_agent`、`agent_bin`、`serial_dev`、`serial_baud` |
| `uv_control/launch/control_launch.py` | workspace_auv | 全部 | `basic_motion` | — |
| `uv_perception/launch/perception_launch.py` | workspace_auv | 全部 | `vision` + `position` | `sim_mode`、`model_path`、`enable_gortc` |
| `uv_nav/launch/nav_launch.py` | workspace_auv | 全部 | `navigator` | — |
| `uv_task/launch/task_launch.py` | workspace_auv | 全部 | `task_runner` | — |
| `uv_sim/launch/bridge_launch.py` | workspace_sim | SIL / HIL | `sim_bridge` | `hil_mode`、`pid_config_file` |
| `stonefish_ros2/launch/stonefish_simulator{,_nogpu}.launch.py` | workspace_sim | SIL / HIL | Stonefish 引擎（**引擎自带的 launch，被 include**） | 6 个 / 3 个引擎参数（见 `workspace_sim/stonefish_notes.md`） |

## 2. 新旧入口映射（T4）

| 旧（T4 之前） | 新 | 说明 |
|---|---|---|
| `uv_bringup/launch/real_bringup.py` | 同名同包 | 入口不变，内容重写为纯编排（不再内联 `Node`） |
| `uv_bringup/launch/sim_bringup.py` | `uv_sim_bringup/launch/sim_bringup.py` | 迁到仿真侧；补齐引擎参数并让它们真正生效 |
| `uv_bringup/launch/sim_core.py` | **无此文件** | 用 `sim_bringup.py` 的两个开关等价取代（见下） |
| `uv_bringup/launch/hil_bringup.py` | `uv_sim_bringup/launch/hil_bringup.py` | 迁到仿真侧 ⏸ |
| `uv_hm/launch/hm_launch.py`（失效） | 同名（已修） | B2：原先运行不存在的 `uv_hm/sim_bridge` |
| `uv_control/launch/control_launch.py`（失效） | 同名（已修） | B2：原先运行不存在的 `minimal_control` |
| （无） | `uv_sim/launch/bridge_launch.py` | 新增：`sim_bridge` 的节点定义归 `uv_sim` |
| （无） | `uv_bringup/uv_bringup/launch_common.py` | 新增：共享编排助手（`arg()` / `include()`） |

**命令级映射**

> 左列按本文件的书写约定**省略 `ros2 launch` 前缀**（写成「包 文件」）——否则那行本身就是一条
> "文档指向已删除入口"，会被 `launch-refs` 检查判为失败。实际使用请把左列补上前缀后照旧。

| 旧命令（省略前缀） | 新命令 |
|---|---|
| `uv_bringup sim_bringup.py …` | `ros2 launch uv_sim_bringup sim_bringup.py …` |
| `uv_bringup sim_core.py` | `ros2 launch uv_sim_bringup sim_bringup.py enable_motion:=false enable_ai:=false` |
| `uv_bringup hil_bringup.py …` | `ros2 launch uv_sim_bringup hil_bringup.py …` |
| `uv_bringup real_bringup.py …`（不变） | `ros2 launch uv_bringup real_bringup.py …` |

**参数级映射**

| 旧参数 | 新参数 | 说明 |
|---|---|---|
| `scenario`（`sim_core`/`hil_bringup`） | `scenario_desc` | 统一命名；且**从"只打日志"变为真正传给引擎** |
| `gpu`（`sim_core`，声明了但未使用） | `headless` | 语义反转：`gpu:=false` ⇒ `headless:=true`；现在是真参数（选 `stonefish_simulator_nogpu`） |
| （无） | `headless` / `simulation_rate` / `window_res_x` / `window_res_y` / `rendering_quality` | 引擎参数经预设透传，不再三处写死 |
| （无） | `enable_mcu` | 单独控制 `sim_bridge`（`sim_core` 的"只要引擎+MCU"由此表达） |
| （无） | `enable_agent` | REAL/HIL 的 micro-ROS agent 开关（转发给 `hm_launch.py`） |

## 3. 编排规则（T4 起，不要绕过）

1. **节点定义归各包**：预设（`real_bringup` / `sim_bringup` / `hil_bringup`）里不出现 `Node(...)`，
   只出现 `include(...)` 与 `condition`。
2. **`enable_*` 门控只在预设**：包级 launch 不声明同名开关，避免"起不起"有两个作者。
3. **引擎参数由预设透传**：`uv_sim_bringup` include 引擎自带的 launch（GPU 6 参 / console 3 参），
   不重复定义引擎节点。
4. **`Data/` 目录解析唯一实现**：`uv_sim_bringup/uv_sim_bringup/stonefish_common.py` 的
   `resolve_simulation_data()`（`$STONEFISH_DATA_DIR` → share → 源码树，失败给警告）。
5. **`workspace_auv` 不含仿真依赖**（不变式 I6）：干净环境只 source `/opt/ros/jazzy` 与
   `workspace_auv/install` 就能 `launch real_bringup.py`。

## 4. 校验方式（无需跑栈）

```bash
python3 scripts/check_docs_contract.py --check --only launch-targets   # launch 里的可执行是否存在
python3 scripts/check_docs_contract.py --check --only launch-refs      # 文档里的入口是否还存在
python3 -m pytest workspace_sim/src/uv_sim/test/test_launch_contract.py -q
```

外加两条运行时（sourced 两个工作区后）：

```bash
ros2 launch uv_bringup     real_bringup.py     --show-args
ros2 launch uv_sim_bringup sim_bringup.py      --show-args
```

## 5. 仍欠（T3b）

| 欠账 | 说明 |
|---|---|
| 实时快照入库 | `scripts/dump_interfaces.sh` + `interfaces/snapshots/` 的 `ros2 topic/service/action list -t` 快照 |
| 话题频率 / QoS 列 | 见 `interfaces/topic_contract.md` 的待补说明 |
| 服务与动作契约 | `interfaces/service_action_contract.md` |
| 引擎自带 launch 的 6/3 个参数 | 目前只在 `workspace_sim/stonefish_notes.md` 记录，未进 `node_parameters.md` 的校验面 |
