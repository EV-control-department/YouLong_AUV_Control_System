# 昨日完整调试参数导入与校正（2026-10-12）

来源：/home/laurie/AUV_2026_Robocup/.pytest_cache/missions 与 .pytest_cache/tasks。
正式目标：workspace_auv/src/uv_task/config/missions 与 config/tasks。
原始调试文件未修改；导入前配置备份在/tmp/auv_parameter_import_20261012。
共核对11个任务、7个调试任务树；新增missions/turntable_simple.yaml，其他等价文件保留当前注释。
已有competition.yaml不在调试源内，保留其阶段策略和靠岸配置。

## 已写入实测/调试值

| 项目 | 写入值 |
| --- | --- |
| 海参抓取补偿body XY | [+0.20,+0.04] m |
| 速度环上浮上限 | 0.10 m/s（原注释5cm/s已校正为10cm/s） |
| 海参运输/配置投放点兜底超时 | 190秒 |
| 建图中心 | [1.20,-1.37] m，原空值同时修成合法JSON |
| 建图类别投票比例 | 0.80 |
| 结论fallback方形格点 | [0,8] |
| 结论fallback圆形格点 | [5,6] |
| require_complete_survey | true，保留当前实现说明：完整尝试九格，定位正常时不因视觉不足跳过路线 |
| 转盘预输入盘心 | [4.9,-2.4,0.85,-90]，同一次START的odom坐标 |
| 转盘观察距离 | 0.50 m；对应艇体观察目标约[4.9,-1.67,0.774,-90] |
| 转盘目标丢失等待 | 3秒 |
| 独立海参链START | skip_preparation=false，采用调试文件值 |
| 默认mapping_grid.json任务树 | START skip_preparation=false、enable_traversal=true |
| full_flow_debug | 保持调试值，一次START skip_preparation=true；mapping→简单转盘→海参→返回原点 |

其他调试值如海参搜索[4,-2,0.10,0]、投放[5.7,-0.70,0.20,0]、
池底1.2m、下压速度0.50m/s/总5.3秒、下压推力0.2、离底推力-0.3等与当前文件一致，已保留。
这里的“实测/调试值”是用户提供的现场配置来源，不代表本次重新测量或水下验证。

## 没有回滚的后续修正

| 调试缓存旧值 | 最终值/处理 | 原因 |
| --- | --- | --- |
| ascent.tolerance_m=0.25 | 0.05m | 25cm门限会把投放0.20→搜索0.10m的上浮直接判为已到位。 |
| total_timeout_seconds=365 | 900秒 | 保留此前多轮超时修正；局部预算仍受总预算截断。 |
| search.travel_timeout_seconds=90 | 180秒 | 保留首次/投放后返回搜索区超时修正。 |
| servo.timeout=30 | 60秒 | 保留视觉小步实测稳定与新帧等待的预算修正。 |
| return_timeout=300 | 360秒 | 抓后上浮及水平回观察位的局部预算。 |
| 缺少small_step_tolerance_m | 保留0.015m | 避免1.8cm步长因0.9cm到位门限卡在厘米级反馈噪声。 |
| 缺少ascent.slowdown_gain/min_speed_mps | 保留0.20/0.003 | 使用近期已修正的速度上浮公式。 |
| 旧collection区域/地标参数 | 不恢复 | 已按用户要求改成双目bbox中心伺服→独立偏置→稳定→释放。 |
| 旧max_press_distance_m=2 | 不恢复 | 原来没有实际限幅作用，已从必填/配置移除。 |

收集框继续使用独立投放偏置[+0.30,0.0]，不叠加海参抓取[+0.20,+0.04]。
投放深目标必须实测到位再释放，释放等待完成后上浮到search.pose.z，最后一轮也上浮。
仍假定近底真实水平漂移为零的B锚点校正开关保留用户调试值true。

## 任务树覆盖与几何核对

competition.yaml原空disk_pose_odom覆盖已移除，继承正式转盘任务的实测盘心。
其海参total_timeout_seconds=360仍是明确的6分钟阶段覆盖，不在缓存调试树内，未擅自改掉。
因此competition海参有效预算360秒；full_flow_debug和独立海参链有效预算900秒。

抓取补偿与左光心[-0.130,+0.030]推算有效抓取XY为[-0.330,-0.010]，
contact_body_xyz仍为[-0.430,0,0.290]；平面位置差约10cm。
两者分别用于水平补偿和最低点离底计算，未凭补偿量改写机械接触参考点。
该差异已在YAML标记，需核对是否代表同一有效接触点。

配置使用相对任务引用，所有导入任务与任务树均可由正式config_loader读取。
默认建图直接运行现在会执行START准备及反向第二圈，区别于原来的跳准备/仅第一圈调试模式。
部署需构建uv_task并重启task_runner。软件检查不代表新的水下试验。
