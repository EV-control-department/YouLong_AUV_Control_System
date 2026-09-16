# 包依赖与启动顺序

> **当前阶段：只做软件在环（SIL）。** HIL/实机相关依赖保留在册但标 ⏸。
> 依赖事实取自各包 `package.xml` 与 `setup.py`（2026-09-16 核对），
> 由 `python3 scripts/check_docs_contract.py --check` 校验。

## 1. 包依赖图（package.xml 实测）

```
                      ┌──────────────┐   ┌──────────────────┐
                      │   uv_msgs    │   │ zit6_interfaces  │   （消息/服务定义，无依赖）
                      └──────┬───────┘   └────────┬─────────┘
                             │                    │
        ┌────────────────────┼────────────────────┼───────────────┐
        ▼                    ▼                    ▼               ▼
   uv_control           uv_nav              uv_task           uv_hm ⏸
        │                    │                    │               │
        └────────────────────┴─────────┬──────────┴───────────────┘
                                       ▼
                                  uv_bringup（实机预设，只 include 各包 launch）
                                       │
   uv_perception ──▶ uv_msgs           │        （uv_bringup 不引用任何仿真包：I6）
                                       ▼
   uv_sim_bringup ──▶ uv_bringup, uv_sim, uv_control, uv_hm, uv_nav, uv_perception,
   （仿真预设）        uv_task, stonefish_ros2
   uv_sim ──▶ stonefish_ros2, zit6_interfaces
   stonefish_ros2 ──▶（tf2, pcl, image_transport, …）
```

| 包 | 声明的依赖（package.xml，已剔除 ament/rosidl/test） | 工作区 |
|---|---|---|
| `uv_msgs` | `std_msgs`, `sensor_msgs`, `action_msgs`, `builtin_interfaces` | workspace_auv |
| `zit6_interfaces` | （无） | workspace_auv |
| `uv_control` | `rclpy`, `std_msgs`, `uv_msgs`, `zit6_interfaces` | workspace_auv |
| `uv_hm` ⏸ | `rclpy`, `std_msgs`, `sensor_msgs`, `nav_msgs`, `uv_msgs`, `zit6_interfaces` | workspace_auv |
| `uv_nav` | `rclpy`, `std_msgs`, `uv_msgs` | workspace_auv |
| `uv_perception` | `rclpy`, `std_msgs`, `sensor_msgs`, `uv_msgs`, `cv_bridge` | workspace_auv |
| `uv_task` | `rclpy`, `std_msgs`, `uv_msgs`, `zit6_interfaces` | workspace_auv |
| `uv_bringup` | `uv_hm`, `uv_control`, `uv_perception`, `uv_nav`, `uv_task`, `ros2launch` | workspace_auv |
| `uv_sim` | `rclpy`, `sensor_msgs`, `nav_msgs`, `std_msgs`, `cv_bridge`, `stonefish_ros2`, `zit6_interfaces` | workspace_sim |
| `uv_sim_bringup` | `uv_bringup`, `uv_control`, `uv_hm`, `uv_nav`, `uv_perception`, `uv_task`, `uv_sim`, `stonefish_ros2`, `ros2launch` | workspace_sim |
| `stonefish_ros2` | `rclcpp`, `std_srvs`, `geometry_msgs`, `nav_msgs`, `visualization_msgs`, `sensor_msgs`, `pcl`, `pcl_conversions`, `image_transport`, `tf2`, `tf2_ros`, `ros2launch` | workspace_sim |

**构建顺序由 colcon 按依赖自动推导**：`uv_msgs`/`zit6_interfaces`（ament_cmake）先构建，
其余 ament_python 依次；`workspace_sim` 是 overlay，**必须先 source `workspace_auv/install/setup.bash`**。

## 2. 依赖缺口（实测，均为 manifest 层缺失）

| # | 缺口 | 影响 | 豁免 |
|---|---|---|---|
| ~~D1~~ | ~~`uv_bringup` 的 launch 引用 `stonefish_ros2` 与 `uv_sim`，但 `package.xml` 未声明~~ | — | ✅ **T4 已修**：仿真预设迁到 `uv_sim_bringup`，并在其 `package.xml` 里声明了全部引用；`uv_bringup` 现在不引用任何仿真包（`launch-targets` + `launch-refs` 检查把关）。**注意此行已无「已知」标记**：若缺口复发，校验会直接失败 |
| D2 | `uv_perception` 代码 import `cv2`、`numpy`、`ultralytics`，`package.xml` 只声明了 `cv_bridge` | 缺 `python3-opencv`/`python3-numpy`/`ultralytics`；`rosdep install` 装不全 | **已知**（补 `<exec_depend>` 即可） |
| D3 | `uv_sim` 代码 import `yaml`、`cv2`（经 `cv_bridge` 间接）、`numpy`，`package.xml` 均未声明 | 同 D2；`setup.py` 的 `install_requires` 里有 `PyYAML`，但那是 pip 层，`rosdep` 不读 | **已知**（补 `<exec_depend>`） |
| D4 | `uv_task` 代码 import `numpy`、`cv2`，未声明 | 同 D2 | **已知**（补 `<exec_depend>`） |

> D2–D4 是**登记**，不是本轮修复项：随各包 `package.xml` 的 `<exec_depend>` 补齐即可。
> `check_docs_contract.py --check` 会把这些报成"未声明的依赖"，**只有带「已知」标记的行才豁免**
> （见 §5 的豁免机制），所以 D1 修掉之后就没有豁免行了。

## 3. SIL 启动顺序与就绪判据

`sim_bringup.py`（包 `uv_sim_bringup`）**不做分阶段延迟**（实测无 `TimerAction`），所有节点同时起，
各自靠"等数据"自然对齐：

| 顺序 | 节点 | 依赖谁 | 就绪判据（代码事实） |
|---|---|---|---|
| 1 | `stonefish_simulator` | 场景文件、mesh、GPU/X | 打印 `Ready for running...`；随后开始发 `/auv/*` 与 `/sim/*_cam/*` |
| 2 | `sim_bridge` | Stonefish 的 `/auv/*` | 收到 `/zit6/cmd/setpoint` 即可动；状态发布按 60 Hz tick 有节奏门控（pos 30 / vel 60 / status 10 / thr 30） |
| 3 | `basic_motion` | `/zit6/state/{pos,vel,status}` | 收到状态后开始更新 `self.pose`；**未收到时会话仍启动**，动作会因拿不到位姿而失败/超时 |
| 4 | `vision`（`enable_ai`） | `sim_mode:=true` 时订阅 `/auv/*_cam/stitched` | 采不到图会按退避重试（`read_failures` 指数退避，上限 0.5 s），不阻塞其它节点 |
| 5 | `position`（`enable_ai`） | `/perception/detection/*` + `/basic_motion/pose_info` | **两者都有**才会产出 `/perception/objects`；缺位姿时静默不出结果 |
| 6 | `navigator`（`enable_nav`） | `/basic_motion/pose_info` | `navigate_to` 在收到第一个位姿前**拒绝**并提示 `no pose yet on /basic_motion/pose_info` |
| 7 | `task_runner`（`enable_task`） | BasicMotion action server | `main()` 里等 action server 最多 15 s；随后按 `debug_mode` 决定是否自动跑 `config/tasks.json`（**非 debug 模式启动即跑**） |
| — | `hw_manager` ⏸ | 真实 MCU（`/zit6/state/zithbt`） | SIL 不启动（`sim_bringup` 不含它）；HIL/REAL 经 `uv_hm/hm_launch.py` 启动，心跳超时进 `DEGRADED` |

**`basic_motion` 是单点**：它是唯一动作入口，也是唯一的 `PoseInfo` 发布者。
它没起来 ⇒ `position`/`navigator`/`task_runner` 全部拿不到位姿。

## 4. 失效影响面（SIL）

| 挂掉的节点 | 直接后果 | 还能验证什么 |
|---|---|---|
| `stonefish_simulator` | 没有 `/auv/*`，`sim_bridge` 无输入 | 无（除静态检查） |
| `sim_bridge` | 无 `/zit6/state/*`，`basic_motion` 拿不到位姿；`/auv/thrusters_cmd` 无人发 | 静态检查、参数服务文档 |
| `basic_motion` | `position`/`navigator`/`task_runner` 全部失能；`/zit6/cmd/setpoint` 无发布者 | `vision` 仍能发检测（`/perception/detection/*` 照常） |
| `vision` | 无检测 ⇒ `position` 无输入 ⇒ 无 `/perception/objects` | 控制闭环、导航（无目标时） |
| `position` | 无 `/perception/objects` ⇒ `navigator`/`task_runner` 的物体相关任务失能 | 控制闭环、巡线（`line_follower` 直接订阅检测） |
| `navigator` | 无 `/nav/path`；`task_runner` 的 `navigate` 任务失败 | 其余动作 |
| `task_runner` | 无 `/task/status`；竞争任务不执行 | 手动发动作 |

## 5. 校验机制与豁免

`scripts/check_docs_contract.py --check` 会做八类比对（参数/launch 参数/节点/话题/依赖/README
+ 两类 launch 一致性），其中"launch 引用 vs manifest 声明"就是把 §2 的 D1–D4 变成可发现项。
**已知缺口不应让校验永远红**，因此约定：

- 缺口写在**本文件 §2 的表格**里，表格带 `豁免` 列（值为 `已知` 即跳过失败）；
- 新增缺口必须先登记再豁免，否则 `--check` 失败；
- 修复后把 `已知` 改成 `已修`（或删行），校验自动要求它真的不再出现。
