# Mapping 任务代码导读（当前实现）

本文按代码实际执行路径解释 `mapping_grid`，供修改任务、排查日志和调参使用。它描述的是当前分支的实现，不是比赛规则的替代说明；场地坐标、相机外参和控制参数须在实机上重新核验。任务的场景背景与运行命令另见 [mapping_task.md](mapping_task.md)。

## 1. 从哪里开始读

| 文件 | 作用 | 推荐入口 |
| --- | --- | --- |
| [config/missions/mapping_grid.json](../src/uv_task/config/missions/mapping_grid.json) | 定义 `start → mapping_grid` 两步任务 | 全文 |
| [config/tasks/mapping_grid.json](../src/uv_task/config/tasks/mapping_grid.json) | 建图几何、时间、相机、识别与滤波参数 | `params` |
| [uv_task/config_loader.py](../src/uv_task/uv_task/config_loader.py) | 解析 mission、合并任务配置、校验参数类型 | `load_mission()`、`TASK_SCHEMAS` |
| [uv_task/task_runner.py](../src/uv_task/uv_task/task_runner.py) | 建立 ROS 节点、自动运行任务列表、发送 BasicMotion Action | `main()`、`run_task_list()`、`_task_mapping_grid()`、`_send_action_goal()` |
| [uv_task/mapping_task.py](../src/uv_task/uv_task/mapping_task.py) | 建图、融合、地图发布、返程及遍历的主体 | `MappingTask.execute()` |
| [uv_control/basic_motion.py](../src/uv_control/uv_control/basic_motion.py) | WTRAVEL 的转向、步进、到达容差与超时 | `_travel_world()`、`_step_move_world()` |
| [uv_camera/ai.py](../src/uv_camera/uv_camera/ai.py) | YOLO 分割检测，发布左右目 `DetectionArray` | `_detect()` |
| [uv_camera/object_localizer.py](../src/uv_camera/uv_camera/object_localizer.py) | 此任务复用的 `StereoCalibration` 标定类 | `StereoCalibration.from_camera_info()` |
| [visualization/mapping_visualizer.py](../../visualization/mapping_visualizer.py) | DDS 地图、图像、分割和深度的只读上位机 | `RosSnapshot` |

`sim.launch.py` 的启动顺序是 Stonefish/桥接 → 控制 → 相机标定就绪 → 感知 → 导航/任务。`wait_for_detections` 默认是 `false`：任务不必等 YOLO 首帧才启动，但任务自身必须在观察窗口内拿到足够数据。`enable_task:=true` 和正确的 `mission_file` 才会运行本任务。

`TaskRunner.main()` 在非调试模式加载 mission，另起任务线程执行 `run_task_list()`；ROS 主线程继续 `rclpy.spin()` 处理订阅、定时器及 Action 结果。mission 中的 `start` 配置 `skip_preparation=true`，直接调用 `_do_start()` 发送 `BasicMotion.Goal.START`，将此刻艇位设为 odom 原点；然后 `_task_mapping_grid()` 构造 `MappingTask`，调用 `execute()`，最后 `destroy()` 清理订阅和定时器。若跳过 `start`，WTRAVEL 可能因原点未建立而被拒绝。

## 2. 两条并行的执行链

```text
任务线程：START → 等标定 → 到理论 Tag 位置 → 等 Tag 确认
                 → 逐格 WTRAVEL + 停留观察（9 格）
                 → 最终分配 → 返回观测 Tag → 按圆形/方形遍历 → completed
                         │
                         └── 后台 mapping-perception 线程（标定后启动、九格后停止）
                               新下视拼接图 → 校正 → AprilTag 检测/融合
                               新左右 YOLO 检测对 → 图像/位姿配对 → SGBM
                                                    → 深度峰 → 格点关联/融合
```

运动**串行**：`_travel_to()` 每次发送一个 `WTRAVEL`，阻塞等待结果，不会同时发两个运动目标。它把调用方 xy、`survey_z` 和 `survey_yaw_deg` 组成 Action 目标；单次超时取 `move_timeout` 和总任务剩余时间中较小者。**注意实际 WTRAVEL 实现并不总遵守传入的 yaw**：`basic_motion._travel_world()` 在 xy 距离大于 0.01 m 时，先转向“当前位置→目标”的方位角，随后保持该方位角步进前进；只有短于该距离时才使用传入的 `survey_yaw_deg`。`TaskRunner._send_action_goal()` 在任务线程等待 Action 结果，ROS 主线程仍可处理回调。

若关心执行速度，应沿 [basic_motion.py](../src/uv_control/uv_control/basic_motion.py) 看 `_travel_world()` → `_step_move_world()` → `_wait_step_convergence()` / `_wait_reached()`：`STEP_X=0.6 m`、`STEP_Y=0.4 m` 决定方向相关基础步长，`STEP_PERIOD=0.2 s` 参与步进收敛判断，最终到达容差为 `TOL_X/Y/Z=0.1 m`、`TOL_RZ=5°`。`move_timeout` **只延长/缩短允许等待时间，不会提高速度**。更底层的实际响应还取决于 ZIT6 控制和仿真水动力，调步长或容差前应核对超调和航迹安全。

感知**并行**：`_start_perception_worker()` 开启 `mapping-perception` 线程。ROS 回调仅把位姿、左右检测和下视拼接图放进有限长度队列；后台线程反复取最新可配对数据，更新 Tag 与各格滤波器。运动途中获得的观测也会进入地图，不必等 AUV 到格点。`_observe_cones()` 仅停留 `observe_seconds` 并统计这个窗口新增的同步帧/测量，不负责即时运行 YOLO 或 SGBM。

共享状态通过 `RLock` 保护。位姿缓存 300 条、拼接图 12 帧、左右检测各 24 条；每格原始测量点最多保留 120 个。队列满后旧数据被覆盖，故画面低帧率和计算积压不能靠无限等待解决。

## 3. 坐标与九宫格编号

`PoseInfo` 来自 `/basic_motion/pose_info`，任务以 START 建立的 odom/NED 坐标计算；地图输出标为 `mapping_odom`。NED 中 x 向北、y 向东、z 向下。`grid_center_x/y`、`grid_side_m`、`grid_yaw_deg` 定义九宫格的理论几何，格点间距为边长除以 3。编号由 `row * 3 + column` 得到，未旋转时为：

```text
       y 较小 ←       → y 较大
x 较大       0   1   2
             3   4   5
x 较小       6   7   8
```

当前 `visit_order=[0,1,2,5,8,7,6,3,4]` 是**建图巡检次序**，不是最终锥桶遍历次序。理论格心只用于巡检目标和测量关联；遍历锥桶时使用滤波得到的观测位置。`floor_z` 写入格点理论中心和地图元数据，不作为 WTRAVEL 深度；当前所有建图运动由 `survey_z=0.10` 控制。

`tag_x/tag_y` 只是首次寻找 Tag 的先验航点；Tag 经视觉确认后，返程目标必须取 `tag_filter.position`，不会用配置坐标顶替测量。当前字典为 `DICT_APRILTAG_16h5`；`tag_ids=[0,...,6]` 限定可接受编号，`tag_id=-1` 表示这七个编号均可触发，但第一次建轨之后只融合相同 ID。

## 4. `execute()` 按顺序做什么

1. 构造函数初始化网格、滤波器容器、发布器和订阅器，并设置 `deadline = monotonic() + timeout`。地图由 1 Hz 定时器发布，同时关键步骤主动发布。
2. `execute()` 在最多 20 秒且不超过总时限的窗口中等待左右 `CameraInfo` 和至少一条位姿。`_prepare_calibration()` 用 `StereoCalibration` 建立双目标定、校正映射及 SGBM；缺失则失败。
3. 启动后台感知后，先 WTRAVEL 到配置中的 Tag xy。`_read_tag()` 不亲自读图，只等待后台的 Tag 有效观测数达到 `min_observations`；等到 `tag_timeout` 或总任务超时仍不足就失败。
4. 依 `visit_order` 访问九个理论中心。每个中心 WTRAVEL 成功后，进入 `observe_cell` 并停留 `observe_seconds`。窗口内同步帧数小于 `min_observations` 时，记录 `observation_failures` 和警告，**继续下一格**；这不是对该格下“必为空”的结论，移动途中已有的滤波证据仍保留。运动失败则立即终止。
5. 九格全部走完后停止后台感知。`_select_final_assignment()` 从累计的有效类别票中选择最多两方两圆的格点分配，冻结 `final_assignment`；发布 `mapping_completed`。即使证据不足四个也可得到部分地图，不虚构缺失锥桶。
6. `_traverse_cones()` 先规划回 Tag 的路径，走到**视觉融合的 Tag xy**；再在九宫格四邻接图上走格点中心/观测锥桶位置，先遍历圆形，再遍历方形。全部动作完成后发 `traversal_completed` 和总任务 `completed`。
7. 任何异常都会设 `state=failed`（外部停止则为 `stopped`）、发布 `failed` 事件、设置 `node.stopped`，从而停止 mission 后续任务。`finally` 无论成功失败都会停止感知线程并发布最终地图。

目前**没有“已有四个高置信目标就提前结束九格巡检”**的路径：执行链仍走完整个 `visit_order`。总超时 `timeout=1800 s` 覆盖校准、Tag、九格巡检、返程和遍历，不会在某阶段重置。任务返回 `True` 表示代码链路运行完毕；地图事件里的 `result_complete` 才表示四个目标是否齐全，二者不是同一判据。

## 5. 图像、检测、位姿怎样配成一条测量

输入话题是 `/auv/down_cam/stitched`、`/perception/detection/down_left`、`/perception/detection/down_right`、两路 `/sim/down_cam/*/camera_info` 和 `/basic_motion/pose_info`。下视拼接图是左右并排的 ROS `Image`，`_image_cb()` 从中点切开后缓存；这里不是直接从 MJPEG/go2rtc 推流读图。推流仅供预览，不参与任务测量。

`_detection_pair()` 取尚未消费的最新左目检测，优先以 `stereo_pair_id` 找右目同一对；没有有效 pair ID 才按 `detection_slop_s` 比较时间戳。`_image_for()` 找最接近该检测时间的拼接帧并做双目校正；`_pose_for()` 找最接近图像/检测时间的位姿。正常门限分别是 `image_slop_s=0.12`、`pose_slop_s=0.10`，但代码在 `reading_tag`/`observe_cell` 阶段可放宽到 5 秒，其他状态可放宽到 1 秒，并记录限流警告。**放宽是低帧率下的权宜措施；艇在动时，时间差仍会造成世界坐标误差。**

找不到图像时，同一检测对最多等待 2 秒，之后增加 `perception_stats.image_unavailable` 并发 `frame_rejected`；位姿不合门限或图像异常也会发拒绝事件。`last_detection_stamp` 避免重复消费同一左目检测。`perception_stats` 中 `detection_pairs` 是见到的新检测对数、`processed_pairs` 是进入锥桶处理的对数；两者之差不是“全部目标被识别失败”的数量。Tag 另按图像时间顺序扫描，不依赖 YOLO 检测对。

### AprilTag 支路

`_tag_measurement()` 对校正后的左目图尝试原灰度与 CLAHE 对比度增强，使用 OpenCV ArUco 接口解码当前配置的 AprilTag 字典；只接受允许 ID。它把标记四角构成掩膜，复用 SGBM 深度峰算法，得到 Tag 的相机距离和像素中心，再经 `_world_measurement(..., rectified=True)` 变成 odom 三维位置。`_accept_tag_result()` 首帧创建 `tag_filter`，后续只融合同 ID 且通过卡尔曼门限的观测。`_read_tag()` 等滤波器有效观测数达到阈值才继续，3 秒打印一次诊断；`markers=0` 偏向解码问题，`wrong_id` 是 ID 被过滤，`no_depth` 是解码成功但深度峰无效。

### 锥桶支路

`uv_camera` 的 YOLO-seg 发布类别及 `mask_x/mask_y` 分割多边形。`_process_cone_pair()` 只接受左目类别 0/1、置信度不低于 `min_confidence=0.35`，并要求右目至少有一个**同类别**检测；这里没有逐目标的左右几何匹配。整个校正双目图先运行一次 SGBM；有效视差深度按 `Z = fx × baseline / disparity` 计算。`_mask_depth_mode()` 在左目掩膜内过滤无效和越界深度，按 `depth_bin_m=0.02 m` 分桶，要求主峰满足 `min_depth_points=20` 及 `depth_peak_ratio=0.12`，再取该峰深度中值与掩膜像素质心。

`_world_measurement()` 将像素/深度依次换算为左目光学系、机体系、odom 系三维点。然后以 xy 距离选最近的理论格心；超过 `cell_gate_m=0.48 m` 的观测只存原始点并拒绝入轨。通过格心门限的观测用 `StaticPositionFilter` 更新：状态仅为静态目标 `[x,y,z]`，预测矩阵是单位阵，协方差随时间加 `process_noise × dt`；新息的马氏距离平方超过 `mahalanobis_gate=11.345` 则判离群。连续独立帧组成占优势的新簇时，允许重建先前被首帧误导的滤波轨迹与类别票。被接受的观测才给类别投票；原始点连同拒绝信息保留给上位机。

注意两个统计口径：`valid_measurements` 在通过格心门限后就增加，**即使卡尔曼随后拒绝**；`accepted_observations` 和类别票只统计真正进入滤波器的观测。`_observe_cones()` 的成功条件只看停留窗口的 `synchronized_frames`，并不要求该格有锥桶或有足够类别票。因此“观察不足”警告不等于“该格最终没有目标”。

## 6. 最终分配与遍历约束

`_select_final_assignment()` 仅使用有投票的格点，枚举最多两个类别 0（方形）和最多两个类别 1（圆形）的不重叠组合，**先最大化入选目标个数，再按票数和占比评分**。低票格也可能入选，不能把结果理解为经过独立的置信度认证。`class_vote_ratio=0.67` 只影响冻结前地图快照是否显示临时类别；冻结后的地图标签由 `final_assignment` 决定。配置中的 `expected_cones=4` 当前未在任务主体中参与判定；“四目标完整”是代码内 `len(confirmed)==4` 的判断。部分结果仍可能走遍历流程，并带 `result_complete=false`。

`_plan_grid_exit()` 用四邻接空格从当前格找一个可直接离开九宫格的边界格，再去观测 Tag；`_plan_grid_traversal()` 在九宫格图上搜索，状态是“当前格 + 已进入锥桶格的位掩码”。空格可重复作为通道；锥桶格进入一次后不能再次进入，且圆形目标全完成前不许进入方形格。目标之间只能走水平/垂直相邻格，不能对角跳格；锥桶航点用滤波位置，空格航点用理论格心。如果这些约束下无路线，会直接报无解，不会偷偷改走禁入格。

遍历阶段每段发动作前，`_segment_cells()` 检查当前位置到航点的直线可能穿过哪些格；`_pose_cb()` 同时监测实测艇位，进入非预期或已访问锥桶格时发 `traversal_violation` 并请求取消当前 Action。靠近格边界 3 cm 内不计入“已进入”，减少位姿抖动。**这只检查规划折线和离散位姿点，不保证艇体轮廓或两条位姿消息之间绝对不越界。**`traversal_order` 仅在某个锥桶格 WTRAVEL 成功后追加，与九格巡检的 `visit_order` 含义不同。

## 7. 输出、状态与排障入口

| DDS 输出 | 关键内容 | 何时看 |
| --- | --- | --- |
| `/task/mapping/map` (`std_msgs/String` JSON) | `state`、`grid`、`tag`、9 个 `cells`、`final_assignment`、`visit_order`、`traversal_order`、返程/遍历路径及原始测量点 | 全程 1 Hz 快照；TRANSIENT_LOCAL 可让后启动客户端拿到最新图 |
| `/task/mapping/events` (`std_msgs/String` JSON) | `started`、`calibration_ready`、`tag_measurement`、`cone_measurement`、`frame_rejected`、`measurement_rejected`、`cell_completed`、`mapping_completed`、遍历事件、`completed/failed` | 看具体因果与拒绝原因 |
| `/task/status` (`uv_msgs/TaskStatus`) | TaskRunner 整体状态 | 判断 mission 是否停止；不替代地图证据 |

地图 `cells[*].center` 是理论格心，`position/covariance` 是滤波估计，`measurements` 是原始点列表（含 `accepted=false`），`residual` 是滤波位置与理论中心差值。`observations`/`accepted_observations` 是滤波器字段；`cells[*].observation` 是格点观察窗口累计帧/测量，来源不同。`mapping_completed` 只说明九格建图结束，**不是**整个任务结束；只有遍历后发出的 `completed` 才是执行链终点。

常见定位顺序：

1. 无任务日志：检查 `enable_task`、mission 路径及启动就绪门槛；查看 `start` 是否完成。
2. `camera calibration or pose unavailable`：查两路 `CameraInfo`、位姿及 20 秒准备窗口。
3. Tag 超时：先看 `frames/markers/wrong_id/no_depth/last`，再核对场景纹理、字典/ID、深度图及下视画面。
4. `image_unavailable` 或 `no pose close enough ...`：按消息时间戳核对拼接图、检测和位姿发布频率；仅放大 slop 不能消除运动时的时空错配。
5. 格点 `synchronized_frames=0`：看后台 `detection_pairs` 与 `processed_pairs`，以及 `frame_rejected`；这不是 YOLO “肯定没看到锥桶”。
6. `mapping_completed` 后失败：看 `final_assignment` 是否有有效融合位置，以及返程/四邻接路径是否有解或发生实测航迹违规。

## 8. 值得重点复核的当前实现细节

- `expected_cones` 目前只是配置项；最终分配固定最多两方两圆，任务不会因不足四个而必然失败。比赛评分若要求四个齐全，应单独决定是否把它设为任务级硬条件。
- 锥桶分支在 `_image_for()` 对图像 `remap` 后，把掩膜质心送入 `_world_measurement()` 的默认 `rectified=False` 分支；后者会再次调用 `StereoCalibration.rectified_pixel()`。Tag 分支明确传 `rectified=True`。这意味着锥桶像素存在**重复校正的风险**，需要用真实标定/真值测量验证位置残差；本文只记录现状，未改代码。
- 背景线程逐帧扫描 Tag；发现标记时会对左右图再算一次 SGBM。若视觉 CPU/GPU 管线已拥塞，这段计算和 1 Hz 全量地图 JSON（含原始点）可能进一步增加延迟，应结合频率与耗时日志评估。
- 右目“有相同类别”只是类别一致性，不是同一个锥桶的立体匹配。深度可信度主要来自校正图上的 SGBM 与掩膜深度峰，需要用残差、原始点云和真实场景检验。
