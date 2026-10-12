# 文档索引（`docs/`）

> 本目录是**项目文档的单一根**。本文件是索引 + 维护规则；新增文档必须在此登记
> （`scripts/check_docs_contract.py` 会检查 `docs/**/*.md` 是否都被提及）。

## 当前阶段

**只做软件在环（SIL）**。HIL 与实机（REAL）相关文档保留在册，但标注 ⏸ 且**不参与验收**
（决定日期 2026-09-16）。SIL 范围界定见 `architecture/sim_real_boundary.md` §4。

## 阅读顺序

| 你是 | 建议顺序 |
|---|---|
| 新人 / AI agent 接手 | 本文件 → `roadmap.md`（要做什么）→ `architecture/system_architecture.md`（长什么样）→ `interfaces/topic_contract.md`（怎么通信） |
| 改控制逻辑 | `frames/frame_convention.md`（坐标系与单位）→ `node_parameters.md`（参数）→ `workspace_auv/docs/basic_motion_action.md`（动作语义） |
| 改仿真 | `workspace_sim/stonefish_notes.md`（引擎行为与坑）→ `frames/extrinsics_reference.md`（外参）→ `architecture/node_dependency.md`（启动顺序） |
| 动 launch / 入口 | `interfaces/launch_entries.md`（入口清单与编排规则）→ `node_parameters.md`（参数） |
| 查一个话题/类型 | `interfaces/topic_contract.md`（唯一权威） |

## 文档地图

### 接口与契约（唯一事实源）

| 文档 | 内容 | 状态 |
|---|---|---|
| `interfaces/topic_contract.md` | **话题 / 消息类型 / 发布者 / 订阅者 / 缺口登记** | ✅ 权威 |
| `interfaces/topic_rates.md` | **话题频率与 QoS**（实测 + 快照口径与陷阱） | ✅ 权威（T3b 建，sil-console 一份快照） |
| `interfaces/snapshots/code_inventory.json` | `--update` 生成的代码库存（评审用，非权威） | 🔄 生成物 |
| `interfaces/snapshots/<日期>_<模式>/` | `scripts/dump_interfaces.sh` 的实时快照（topics/services/actions/qos/rates/meta） | 🔄 生成物（入库作证据） |
| `node_parameters.md` | 各节点 ROS 参数与 launch 参数（含默认值、「已知缺口」清单） | ✅ 已逐条核对 |
| `frames/frame_convention.md` | 坐标系、单位、rpy 顺序、TF 现状、θ/α/ψ 术语 | ✅ 权威 |
| `frames/extrinsics_reference.md` | 外参数值（仿真表 / 实机表 / 推导 / T6 必改清单） | ✅ 权威 |
| `interfaces/service_action_contract.md` | **服务与动作的字段语义**（含引擎自带服务与死定义登记） | ✅ T3b 建立 |
| `interfaces/launch_entries.md` | **启动入口清单与新旧映射**（T4 起，含编排规则） | ✅ 权威 |

### 架构

| 文档 | 内容 |
|---|---|
| `architecture/system_architecture.md` | 分层、9 个可执行节点、SIL 数据流、两条入口 |
| `architecture/sim_real_boundary.md` | SIL/HIL/REAL 边界、话题所有权矩阵、当前范围界定 |
| `architecture/node_dependency.md` | 包依赖图、**依赖缺口登记**、启动顺序与就绪判据、失效影响面 |
| `hil_architecture.md` | ⏸ HIL 设计与 micro-ROS 链路（暂不参与验收） |
| `auv-architecture.html` / `.json` | archify 生成的架构图（生成物，不手改） |
| `architecture/dataflow_sil.html` / `.json` | **SIL 信息路线图**（交互式；`.json` 是可编辑的源，改完要重新 deliver） |
| `architecture/dataflow_hil.html` / `.json` | **HIL 信息路线图**（⏸ 未验证，仅供理解角色分工） |

### 计划与日志

| 文档 | 内容 |
|---|---|
| `roadmap.md` | 主计划 T1–T7、基线缺陷 B1–B18、验收总表 |
| `../devlog/` | 开发日志（按日期） |

### 包级文档（不在本目录，链接见下）

| 文档 | 内容 |
|---|---|
| `workspace_auv/docs/basic_motion_action.md` | `basic_motion` 动作完整用法 |
| `workspace_auv/docs/perception.md` | 感知架构与数据流 |
| `workspace_auv/docs/debug_guide.md` | 调试手册（ros2 CLI、动作测试、仿真） |
| `workspace_auv/docs/coordinate-design.md` | `Coordinate` 类设计 |
| `workspace_sim/stonefish_notes.md` | Stonefish 引擎行为、场景结构、已知坑 |

## 维护规则

1. **改代码必须同一次改动里改文档**：

   | 你改了什么 | 必须更新 |
   |---|---|
   | ROS 参数默认值 / launch 参数 | `node_parameters.md` |
   | 话题名、消息类型、发布者/订阅者 | `interfaces/topic_contract.md` |
   | 包依赖、启动顺序 | `architecture/node_dependency.md` |
   | 坐标系、外参、DVL 波束角 | `frames/` 两篇 + 重跑 `derive_extrinsics.py --out` |

2. **校验命令**（都不需要跑栈，纯源码/文档比对）：

   ```bash
   python3 scripts/check_docs_contract.py --check     # 漂移即 exit 1
   python3 workspace_sim/tools/derive_extrinsics.py --check
   python3 workspace_sim/tools/check_dvl_geometry.py
   python3 -m pytest workspace_sim/src/uv_sim/test/ -q
   python3 -m pytest workspace_auv/src/uv_control/test/ -q
   ```

3. **标记含义**（各文档通用）：

   | 标记 | 含义 |
   |---|---|
   | ✅ | 已与代码逐条核对，当前准确 |
   | ⏳ | 已知不完整或即将失效，等后续任务回填 |
   | 🔗 | 事实源在别处，本节只留链接 |
   | ⏸ | HIL/实机相关，当前阶段暂缓（SIL 优先） |

4. **禁止复制事实**：话题表只在 `topic_contract.md` 存在；参数表只在 `node_parameters.md` 存在；
   外参数值只在 `frames/extrinsics_reference.md` 存在。其他文档用链接。
   （T3a 之前 `hil_architecture.md`/`node_parameters.md` 各有一份话题表，已经出现不一致。）

5. **缺口要登记，不要静默**：已知问题写进对应文档的「缺口 / 待补」表并标注豁免，
   这样校验工具既能保持绿色，又不会让人误以为问题不存在。

## 待补清单（跨文档汇总）

| # | 缺口 | 何时 |
|---|---|---|
| 1 | ~~`interfaces/service_action_contract.md`（服务/动作字段语义）~~ | ✅ T3b（含引擎自带服务与死定义登记） |
| 2 | ~~`interfaces/launch_entries.md` 与 launch 参数表（T4 重排 `uv_bringup` 后）~~ | ✅ T4 |
| 3 | ~~话题频率 / QoS 列（需实时快照）~~ | ✅ T3b（`interfaces/topic_rates.md` + 入库快照；**sil-gpu / HIL 的快照仍欠**） |
| 4 | ~~`sim_bridge` 参数与话题归属按 T5 的新结构改写~~ | ✅ T5（拆成 7 节点，见 `node_parameters.md` §仿真节点与 `topic_contract.md` §1） |
| 5 | 实机外参的实测复测（`frames/extrinsics_reference.md` §2 中「未验证」的行） | ⏸ 待硬件 |
| 6 | `node_dependency.md` §2 的 D2/D4 依赖缺口（D1/D3 已修） | 随包补齐 |
