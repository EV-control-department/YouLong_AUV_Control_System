# 九宫格锥桶建图任务

## 1. 任务边界

本任务对应《水下具身智能》中的中心区域建图。比赛机器人必须自主运行，不能依靠拖缆、遥控、外部定位或外部数据回传改变运动结果。任务只使用艇上 ROS 2/DDS 数据：机器人位姿、左右下视相机图像、相机标定信息和视觉检测结果。

当前场景使用 AprilTag。OpenCV 通过 `cv2.aruco.ArucoDetector` 提供 AprilTag
家族检测接口；建图配置当前使用 `DICT_APRILTAG_36H11`，`tag_id=-1` 接受随机场景的实际 ID，多帧融合时保持同一 ID。场景中
资源文件仍可能保留 `aruco_*.png` 的历史文件名，但不代表编码家族仍是 ArUco。

当前场景给出的几何量为：

- 标记中心：`(1.50, -2.00)`。
- 九宫格中心：`(2.00, -4.00)`。
- 九宫格边长：`2.00 m`。
- 九宫格航向：`0°`。
- 每个格子的理论中心间距：`2 / 3 m`。
- AprilTag 上方巡检高度：`survey_z=0.10 m`，用于扩大下视相机视野；池底高度仍为 `floor_z=1.994 m`。
- 锥桶类别：YOLO 分割模型 `best.pt` 的 `0 = 方形锥桶`、`1 = 圆形锥桶`。

实际比赛前应将九宫格中心、边长、航向、池底深度和标记 ID 写入 `config/tasks/mapping_grid.json`，不要把仿真坐标直接当成现场坐标。

## 2. 执行流程

历史测试中曾因格点0四秒观察窗口内同步帧为0而直接中止，导致后续八个格点没有访问。
当前实现已将“单格点观测失败”和“运动失败”分离：前者只记录该格点状态并继续，后者仍然中止任务以保证运动安全。

场景模板及当前随机场景引用 `apriltag_36h11_id0.png`。随机生成器用 OpenCV
生成含黑色编码边框、白色留白的 RGB 纹理，并自检解码 ID 后才输出场景；
不再选择旧的 `aruco_4x4_id*.png`。标记朝上的网格表面已修正镜像 UV。
读取标记时每三秒输出中文进度；`frames=0` 并伴随位姿错误表示时间同步失败，
`markers=0` 表示未解码，`wrong_id` 表示 ID 不匹配，`no_depth` 表示解码后深度无效。
启动日志包含实际 Python 文件路径，可核对运行的是否是新安装版本。

```text
仿真起点 `(0, 0, 0.10)`，跳过实机拔缆倒计时并执行 START
        ↓
WTRAVEL 到池底标记附近
        ↓
下视图识别标记，并记录多帧位置
        ↓
按固定访问顺序进入九宫格九个理论中心
        ↓
左右下视图同步获得检测结果和分割掩膜
        ↓
对左右图像做 SGBM，读取掩膜内深度直方图主峰
        ↓
根据相机标定和机器人位姿换算到建图坐标系
        ↓
按理论格位关联目标，使用静态位置卡尔曼滤波器更新
        ↓
输出九宫格地图和访问记录
```

任务访问所有九个格子，最终应确认四个目标格。路线顺序在 JSON 的 `visit_order` 中配置，当前为 `[0, 1, 2, 5, 8, 7, 6, 3, 4]`。

### 2.1 本次问题与解决方法

此前建图采用“移动到格点后，再在主流程中读取一次检测”的串行方式。该方式有两个明显缺陷：

1. WTRAVEL 期间不处理视觉数据，YOLO 推理延迟会使检测结果在格点观察窗口之外才到达。
2. 检测消息、拼接图像和位姿消息由不同回调发布，严格要求它们在很小的时间差内相等，容易将真实检测误判为 `image timestamp unavailable` 或 `no pose close enough to image timestamp`。

当前解决方案是“运动控制串行、感知处理并行”：WTRAVEL 仍然一次只执行一个目标，保证控制链路安全；`MappingTask` 在标定准备完成后启动独立的后台感知线程，持续消费图像和检测消息。感知结果不再依赖某一个格点的瞬时窗口，而是持续进入观测池和静态位置滤波器。

### 2.2 `uv_task` 修正方案

核心修改位于 `uv_task/mapping_task.py`，不改变现有 ROS 任务接口：

- 增加 `mapping-perception` 后台线程，统一处理 AprilTag 和锥桶感知。
- 主线程的 `_read_tag()` 改为等待后台线程累计到足够的 AprilTag 有效观测，不再重复扫描同一批图像。
- 主线程的 `_observe_cones()` 改为读取当前格点的累计计数，只负责等待观察时间结束和输出统计。
- 抽出 `_process_cone_pair()`，对每个左右目检测对执行掩膜深度主峰、世界坐标换算、格点关联、卡尔曼更新和类别投票。
- 单格点无检测、深度无效或时间配对失败，只产生事件和警告，不会抛出任务级异常。
- WTRAVEL 失败、标定失败、AprilTag 在规定时间内未确认，以及最终目标数量不足，仍会按任务失败处理。
- 后台线程在 `execute()` 的 `finally` 和 `destroy()` 中停止并回收，避免任务结束后遗留线程。

线程之间共享滤波器、类别投票和观测计数时使用 `self.lock`；图像和检测队列使用有限长度缓存，避免积压过期帧。

## 2.3 具体任务执行逻辑

任务列表由 `mapping_grid.json` 提供，正常包含 `start` 和 `mapping_grid` 两个任务。整体伪代码如下：

```text
start
  └─ START：以当前 map 位姿建立 odom 原点

mapping_grid
  ├─ 等待左右相机 CameraInfo 和机器人位姿
  ├─ 创建双目标定、立体校正映射和 SGBM
  ├─ 启动后台感知线程
  │    ├─ 新图像：尝试识别 AprilTag，并更新 tag_filter
  │    └─ 新检测对：计算锥桶深度和世界坐标，并更新 cone_filter
  ├─ WTRAVEL 到 AprilTag 理论位置
  ├─ 等待 tag_filter 达到 min_observations
  ├─ 按 visit_order 逐格执行
  │    ├─ WTRAVEL 到理论格点中心
  │    ├─ 等待 observe_seconds，让后台线程继续累计观测
  │    ├─ 保存该格点的同步帧数和有效测量数
  │    ├─ 无有效观测：记录为空格，继续下一格
  │    └─ 发布地图快照和 cell_completed 事件
  ├─ 确认九个格点均已访问
  ├─ 统一检查 expected_cones、观测次数和类别投票
  └─ 发布 completed 或 failed
```

后台线程不要求当前状态必须是 `observe_cell` 才处理视觉数据。移动期间得到的有效目标测量也可以进入对应理论格点的滤波器；只有“同步帧数”统计限定在当前格点停留阶段，用于判断该格点是否真正获得过可用感知输入。这样可以利用运动期间已经产生的有效数据，同时避免把移动过程中的帧伪装成格点停留观测。

## 2.4 单个格点的状态和任务失败边界

每次访问格点时，任务保存进入观察阶段前的计数快照。观察结束后计算增量：

```text
本格点新增同步帧 = 结束时同步帧 - 进入时同步帧
本格点新增测量   = 结束时测量数 - 进入时测量数
```

`synchronized_frames=0` 不再意味着任务失败，它只说明当前停留期间没有形成完整的图像、位姿和检测数据。九宫格中的空格本来就不应包含锥桶，因此空格允许正常通过。任务只有在完成全部路线后，才检查是否找到了规定数量的锥桶。

失败边界如下：

| 情况 | 处理方式 |
|---|---|
| 单格点无检测 | 记录空格，继续下一个格点 |
| 图像时间戳暂时无法配对 | 保留检测等待重试，不终止任务 |
| 深度主峰无效 | 拒绝本次测量，继续任务 |
| 卡尔曼马氏距离超门限 | 拒绝离群观测，继续任务 |
| WTRAVEL 动作失败 | 中止任务，避免继续盲目运动 |
| AprilTag 超时未确认 | 中止任务，避免使用未校准的地图坐标 |
| 九格访问结束但确认目标数不足 | 完整检索后报告建图失败 |

## 2.5 时间戳配对策略

当前配对优先级如下：

1. 左右检测均带有非零且相同的 `stereo_pair_id` 时，直接按 pair ID 配对。该 ID 来自仿真立体拼接器，能够覆盖左右相机约 `0.1 s` 的采集时间差。
2. 没有 pair ID 的旧接口，才使用 `detection_slop_s` 比较左右检测时间戳。
3. 用检测时间戳在 `/auv/down_cam/stitched` 缓存中查找最近图像。严格时间窗由 `image_slop_s` 控制；差值不超过 1 秒时，在机器人标记识别或格点停留阶段使用最近帧，并输出中文警告。
4. 位姿同样优先取最近时间戳。停稳阶段允许不超过 1 秒的有限延迟；超过上限才拒绝该帧。
5. 检测已经到达但对应图像尚未进入缓存时，不更新 `last_detection_stamp`，后台线程会继续重试，避免检测被提前消费。

上述策略的原则是：运动阶段尽量使用严格时间对齐，停稳阶段允许有限延迟；任何放宽都必须有上限，避免使用无限期的陈旧图像。

单个格点没有目标是正常情况：四个锥桶只占九个格点中的四个。当前实现启动独立的后台感知线程，
在 WTRAVEL 运动期间和格点停留期间持续处理 AprilTag、左右目检测和 SGBM 深度；主线程只负责按路线发送运动动作。
因此不会因为某个格点没有有效分割深度、暂时没有同步帧或图像时间戳未配对而停止任务，而是将该格点记录为已访问的空格并继续访问下一个格点。
九个格点全部访问结束后，才统一检查是否确认了 `expected_cones` 个目标；因此“完整检索九宫格”和“最终建图结果满足比赛要求”是两个阶段。
地图快照的 `observation_failures`、`all_cells_visited` 以及每个格子的 `observation` 字段用于区分这两种状态。

检测消息与拼接图像由不同回调产生。左右检测若带有相同 `stereo_pair_id`，优先按该 ID 配对，不受左右相机约 0.1 秒采集偏差影响；
没有 pair ID 的旧接口才使用 `detection_slop_s`。严格时间差超过 `image_slop_s` 但不超过 1 秒时，任务会使用最近一帧图像并输出中文警告；
如果检测先于图像到达，则不会立即消费该检测，会在观察窗口内重试，避免误报空格。

## 3. 视觉和测距实现

### 3.1 YOLO 分割

`uv_camera` 使用 `resource/best.pt`。`uv_ai` 将每个检测的分割轮廓通过 `Detection.mask_x` 和 `Detection.mask_y` 发布到对应的左右检测话题；原有 bbox 和类别字段保持不变。

建图任务只接受类别 ID 0 和 1，并要求左右图像都检测到同一类别。这样可以避免把单目误检直接写入地图。

### 3.2 SGBM 深度主峰

建图节点订阅 `/auv/down_cam/stitched`，拆出左右图像后根据 `CameraInfo` 完成去畸变和立体校正，再运行 `StereoSGBM`。

对每个 YOLO 分割掩膜：

1. 取掩膜范围内的视差。
2. 删除无效视差、负深度和超出深度范围的数值。
3. 将深度按 `depth_bin_m` 分箱。
4. 选取频数最高的合理峰值，并用峰值箱内深度的中位数作为本次测量深度。
5. 以掩膜像素中心和主峰深度恢复目标点。

这种处理会丢弃掩膜边缘、背景和错误匹配形成的深度群，不直接使用整幅深度图平均值。`depth_peak_ratio` 和 `min_depth_points` 用于拒绝深度分布不可靠的帧。

### 3.3 世界坐标换算

立体校正后的相机点先恢复到左相机光学坐标，再使用下视相机外参换算到艇体坐标，最后使用 `/basic_motion/pose_info` 中的艇体位姿换算到 `mapping_odom` 建图坐标系。

当前实现使用艇体启动后的 odom 坐标作为地图坐标。`START` 前必须确认机器人已经位于规则要求的投放/起始位置；如果现场坐标需要额外平移，应在 JSON 中调整九宫格和标记坐标。

## 4. 静态位置卡尔曼滤波器

锥桶和标记在任务期间视为静止目标，状态只包含三维位置：

```text
x(k+1) = x(k) + w(k)
z(k)   = x(k) + v(k)
```

状态转移矩阵和观测矩阵都是单位阵。过程噪声很小，用来吸收机器人位姿误差和场景微小误差；测量协方差由当前深度测距的固定噪声近似给出。

每次更新前计算马氏距离，超过 `mahalanobis_gate` 的测量会被拒绝。类别不单独做位置滤波，而是对同一格的 0/1 识别结果进行投票；达到 `class_vote_ratio` 后才输出类别。

## 5. DDS 接口

任务使用的输入：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/basic_motion/pose_info` | `uv_msgs/msg/PoseInfo` | 机器人位姿和时间对齐 |
| `/auv/down_cam/stitched` | `sensor_msgs/msg/Image` | 左右下视原始图像 |
| `/sim/down_cam/left/camera_info` | `sensor_msgs/msg/CameraInfo` | 左相机标定 |
| `/sim/down_cam/right/camera_info` | `sensor_msgs/msg/CameraInfo` | 右相机标定 |
| `/perception/detection/down_left` | `uv_msgs/msg/DetectionArray` | 左目类别和分割掩膜 |
| `/perception/detection/down_right` | `uv_msgs/msg/DetectionArray` | 右目类别和分割掩膜 |

任务输出：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/task/mapping/map` | `std_msgs/msg/String` | 低频完整地图快照，Transient Local |
| `/task/mapping/events` | `std_msgs/msg/String` | 标记测量、锥桶测量、拒绝原因和状态变化 |
| `/task/status` | `uv_msgs/msg/TaskStatus` | 通用任务状态 |

地图 JSON 中保留格子理论中心、测量位置、协方差、观测次数、类别投票、残差和访问顺序。

## 6. 日志和可视化

现有 `uv_log` 默认主题正则已经包含 `/task/.*`、`/perception/.*`、`/basic_motion/.*`，因此新的地图快照和事件会进入 rosbag。建图事件中的每条数据同时通过 ROS logger 输出关键失败原因，不需要扩大日志包的接口。

可视化脚本只订阅 DDS，不读取地图文件：

```bash
source /opt/ros/humble/setup.bash
source workspace_auv/install/setup.bash
python3 visualization/mapping_visualizer.py
```

它显示九宫格理论位置、滤波后位置、残差连线、标记位置和当前任务状态。

## 7. 运行方式

当前任务清单明确包含两个步骤：先执行 `start` 初始化 odom 原点，再执行
`mapping_grid`。如果日志出现 `odom origin not set, call start() first`，说明运行的仍是旧版任务清单或没有重新 source 安装空间。

仿真启动默认不再等待四路 YOLO detection 才释放任务。相机和模型在后台继续工作，建图任务在每个格点观察阶段自行等待新鲜检测；这样可以避免 AUV 在模型首次推理期间漂离起点。需要复现旧的全量感知门控时，可显式添加 `wait_for_detections:=true`。

先构建两个工作区：

```bash
cd YouLong_AUV_Control_System/workspace_auv
colcon build --symlink-install --packages-up-to uv_msgs uv_camera uv_task
source install/setup.bash

cd ../workspace_sim
colcon build --symlink-install
source install/setup.bash
```

启动相机、定位和任务时需要打开 AI；模型路径也可以显式指定：

```bash
cd ..
source /opt/ros/humble/setup.bash
source workspace_auv/install/setup.bash
source workspace_sim/install/setup.bash

ros2 launch uv_bringup sim.launch.py \
  profile:=sim_dev \
  gpu:=true \
  gpu_backend:=nvidia \
  enable_ai:=true \
  enable_nav:=false \
  enable_task:=true \
  wait_for_detections:=false \
  scenario_desc:=water_embodied_intelligence_random.scn \
  mission_file:="$PWD/workspace_auv/src/uv_task/config/missions/mapping_grid.json"
```

这里明确使用 `workspace_sim/src/stonefish_ros2/Data/water_embodied_intelligence_random.scn` 作为 Stonefish 场景；`scenario_desc` 传入文件名后，由仿真启动流程从 Stonefish `Data` 目录解析该文件。

如果启动日志出现 `ultralytics not installed`，需要在运行任务的 Python 环境中安装 Ultralytics 和对应的 PyTorch；仅构建 ROS 包不会自动下载这两个推理依赖。模型文件 `resource/best.pt` 受当前仓库的 `*.pt` 忽略规则管理，应在运行机器上准备好。

单独检查输出：

```bash
ros2 topic echo /task/mapping/map
ros2 topic echo /task/mapping/events
ros2 topic echo /task/status
```

当前实现的建图任务名称为 `mapping_grid`，也可用调试模式通过 `/task/exec` 单独调用。正式比赛前应先在仿真中验证坐标原点、锥桶深度主峰、SGBM 参数和四个目标的格位关联。

本任务不使用 `yellow_golf`、`pink_golf` 等旧竞赛目标元数据；默认 `target_id` 为
`mapping_grid`。旧任务若需要使用旧目标，必须在启动命令中显式传入对应的 `target_id`。

相机延迟排查时应区分两类视频：`/front`、`/down` 是低延迟原始 MJPEG；
`/front_annotated`、`/down_annotated` 需要等待 YOLO 分割推理，CPU 仿真下可能明显滞后，不能用它们判断原始图像传输是否正常。当前仿真默认不以四路 detection 作为全局启动门控；建图任务只在逐格观察时等待新鲜检测，因此首次模型推理不会阻塞 AUV 发车。
