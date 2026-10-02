# 九宫格建图：相机内处理、任务只消费观测

## 信息流与边界

```text
真机 V4L2 → uv_camera.Sensor → 内存 FrameGate → uv_camera.Ai (YOLO-Seg)
仿真 Stonefish 左/右目 Image → uv_camera.Sensor 直接配对 → 同一 FrameGate
                                                   ↓
uv_camera.MappingVision：AprilTag 16h5 + 校正/SGBM + 掩膜深度峰 + 采集位姿
                                                   ↓ 小型观测数组 (DDS)
                           /perception/mapping/observations
                                                   ↓
uv_task.MappingTask：九格关联 → 静态卡尔曼滤波 → 类别投票 → 路径规划/状态机
                                                   ↓
                       /task/mapping/map 和 /task/mapping/events
```

真机从 V4L2 到 YOLO/建图视觉均是进程内 BGR 数组，不发布 DDS Image。
仿真受 Stonefish 原生接口限制，第一跳仍是 `/sim/{front,down}_cam/{left,right}/image_color`；
camera 直接接收并在本进程配对/拼接。默认关闭 sim_bridge 旧的 `/auv/*/stitched`
二次发布；确有旧消费者时可在 sim_bridge 设置 `enable_camera_passthrough:=true`。
camera 的普通 YOLO DetectionArray 话题仍保留供其他模块使用，但 MappingTask 不订阅它们。

camera 生成的每帧观测使用 `uv_msgs/msg/MappingObservationArray`：
`header.stamp` 是左目采集时间；`processed`/ `reason` 区分有效、标定缺失和位姿过期；
数组同时带标记/锥桶候选数量及深度拒绝数量，方便区分“没识别”与“识别到但测距失败”。
每个 `MappingObservation` 带 `kind=TAG/CONE`、标记 ID 或锥桶类 ID、
置信度、深度米数、主峰有效像素数、`mapping_odom` 三维坐标和采集位姿时间差。
空帧也发布数组，让任务能统计真实处理过的同步帧；不传图像和掩膜多边形。
camera 使用采集时间查找最近 PoseInfo，默认最大位姿差 1 秒，超出时明确拒绝，
不复用任意旧位姿。视觉线程采用 latest-wins 队列，不积压过时图像。

## 任务执行

任务清单先 `start`（建立 odom 原点），再 `mapping_grid`。
camera 感知线程独立运行，运动与观测并行；任务主流程仍串行 WTRAVEL：

1. 等待 camera 的有效观测帧和 PoseInfo；WTRAVEL 到 AprilTag 理论位置。
2. camera 在校正后的下视左图解码 `DICT_APRILTAG_16h5`，允许 ID 0–6
   （任务 `tag_id=-1` 表示接收任意 ID，但同一条轨迹只融合同一个 ID）。
   多次观测确认后，依配置 `visit_order` 逐格巡检。
3. camera 对 YOLO 0=方形、1=圆形的分割掩膜运行 SGBM 深度主峰，
   以主峰像素中位数反投影并用采集时位姿换算世界坐标。
   Task 按 XY 最近理论格关联，先做格位门限，再做静态目标滤波/离群剔除/类别投票。
4. 空格或暂时无深度只记观测不足，仍访问剩余格子。运动失败或标记未确认
   才提前中止。九格后按“两方两圆”约束选最可信的部分/完整结果并发布地图。
5. 返回已观测 AprilTag 位置，先圆形后方形遍历已确认锥桶。
   遍历在九宫格四邻接图上沿格点中心走；锥桶格进入一次后不可再次进入。
   有目标的终点使用融合坐标，不回退到 JSON 理论中心。

任务仍订阅小型 `/basic_motion/pose_info`，用于遍历阶段实时位置安全监测；
它不再做图像/检测/CameraInfo 的时间配对。九格关联和 Kalman 状态都在 task，
不会被 camera 的单帧视觉判断固定。

## 配置与标定

九宫格中心为 `(2.0, -4.0) m`，单格边长 `0.8 m`，总边长 `2.4 m`；
`grid_side_m` 表示整个 3×3 网格的边长，随机锥桶中心也按 0.8 m 格距生成。
`workspace_auv/src/uv_task/config/tasks/mapping_grid.json` 只保留任务几何、
巡检/遍历、滤波和目标数量参数。视觉参数在 `uv_camera` 节点：
`mapping_tag_dictionary`、`mapping_tag_id`、`mapping_*depth*`、
`mapping_sgbm_*`、`mapping_left/right_translation`、`mapping_camera_rotation`。
仿真采用 Stonefish CameraInfo 及场景中的左右相机外参；默认基线 0.10 m。
`sim_dev`/`sim_ci`/`hil_lab` 的下视相机平移必须与
`xunyun_fixed.scn` 一致（左目 `[0,-0.05,0.176]`，右目 `[0,0.05,0.176]`）。
位姿按左右相邻采样插值（yaw 使用最短角差），避免快速转向时用最近 30Hz
离散姿态直接投影产生横向位置抖动。双目 SGBM 已直接提供深度，因此锥桶
不再要求右目 YOLO 必须同时检出；观测消息分别统计低置信度、掩膜缺失和
深度峰失败，便于判定观测停在哪一道检查。

真机使用 `mapping_calibration_file`（默认 `uv_camera/config/down.npz`）。
现有 NPZ 对应的默认检查尺寸为每目 1280×960，而当前 V4L2 采集配置为
每目 1920×1080；尺寸不匹配会发布 `processed=false` 和清晰错误，
不会输出貌似正确的世界坐标。实机运行前必须在实际采集模式重新标定并设置
`mapping_calibration_file`、`mapping_calibration_width`、
`mapping_calibration_height`，或把采集模式改为标定时的模式并验证分辨率。
仿真不受此 NPZ 限制。

## 上位机与日志

`visualization/mapping_visualizer.py` 只从 DDS 读取地图、事件、PoseInfo。
图像、掩膜叠加、视差、深度和深度直方图由 camera 用同一帧计算，
经 HTTP 快照提供：

```text
http://127.0.0.1:8090/mapping/input.jpg
http://127.0.0.1:8090/mapping/overlay.jpg
http://127.0.0.1:8090/mapping/disparity.jpg
http://127.0.0.1:8090/mapping/depth.jpg
http://127.0.0.1:8090/mapping/histogram.jpg
```

面板启动前确保 camera 的 `enable_gortc=true`（会启动本地 8090 MJPEG/快照源）。
远程显示设置 `UV_CAMERA_MJPEG_URL=http://艇载主机:8090`；真实比赛不依赖面板。
地图和观测为小型 DDS 消息，可用
`ros2 topic echo /perception/mapping/observations` 和
`ros2 topic echo /task/mapping/events` 排错。
`uv_log` 的 `/perception/.*` 与 `/task/.*` 规则可记录它们；HTTP 图像不进入 rosbag。

## 构建与仿真启动

在仓库根目录：
colcon build --symlink-install --packages-select uv_msgs uv_camera uv_task
colcon build --symlink-install --packages-select uv_sim
```bash
source /opt/ros/humble/setup.bash
cd workspace_auv
source install/setup.bash
cd ../workspace_sim
source install/setup.bash
cd ..
ros2 launch uv_bringup sim.launch.py \
  profile:=sim_dev gpu:=true gpu_backend:=nvidia ai_device:=cuda:0 \
  enable_ai:=true enable_nav:=false enable_task:=true \
  scenario_desc:=water_embodied_intelligence_random.scn \
  mission_file:="$PWD/workspace_auv/src/uv_task/config/missions/mapping_grid.json"
```

场景使用 `apriltag_16h5_id*.png`（随机生成器可选 0–6）。
若视觉话题没有 `processed=true`，先查 CameraInfo、位姿时间差和标定错误；
不要再用旧的 `/auv/down_cam/stitched` 观察任务是否在处理。

## 仅启动仿真并录制视频数据集

以下命令分别在三个终端执行，工作目录均为仓库根目录
`YouLong_AUV_Control_System`。先启动仿真，等 `uv_camera` 显示 MJPEG 服务已
监听 `8090`，再启动 GUI 和录制脚本。

终端 1：启动 Stonefish、控制桥与相机，不启动建图任务。这里必须保留
`enable_ai:=true`，因为当前 `sim.launch.py` 仅在启用感知时包含相机节点；
`enable_task:=false` 不会自动运行 `task_runner`。

```bash
cd /home/laurie/AUV_2026_Robocup/YouLong_AUV_Control_System
source /opt/ros/humble/setup.bash
source workspace_auv/install/setup.bash
source workspace_sim/install/setup.bash
ros2 launch uv_bringup sim.launch.py \
  profile:=sim_dev gpu:=true gpu_backend:=nvidia ai_device:=cuda:0 \
  enable_ai:=true enable_nav:=false enable_task:=false enable_preview:=true \
  scenario_desc:=water_embodied_intelligence_random.scn
```

终端 2：打开 ZIT6 上位机。`gui.py` 使用包内相对导入，不能直接执行文件路径；
通过模块方式启动，并将仓库中的 `upper_examples` 加入 Python 搜索路径。

```bash
cd /home/laurie/AUV_2026_Robocup/YouLong_AUV_Control_System
source /opt/ros/humble/setup.bash
source workspace_auv/install/setup.bash
source workspace_sim/install/setup.bash
PYTHONPATH="$PWD/third_party/AUV_zit6_cmake/upper_examples${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m upper_examples.gui
```

终端 3：直接连接相机节点的 MJPEG 源，逐帧保存前视 `/front` 与下视 `/down`
双目拼接画面；不经过 DDS 图像话题，也不依赖 go2rtc 转发。

```bash
cd /home/laurie/AUV_2026_Robocup/YouLong_AUV_Control_System
python3 scripts/record_camera_streams.py \
  --base-url http://127.0.0.1:8090 \
  --output "$PWD/records/datasets"
```

默认持续录制，按 `Ctrl-C` 停止；可加 `--frames 100` 指定每路保存 100 帧，
或加 `--fps 5` 限制每路保存帧率。每次录制创建独立的 `stream_...` 会话目录，
内含 `front/`、`down/` JPEG 图像、逐帧时间戳清单 `frames.jsonl` 和状态文件。
这是**推流画面**（JPEG 压缩、前后视各为左右目拼接图，可能带位姿叠加），
不是无损原始图像，也不会自动生成检测标签。要从另一台机器录制，把
`--base-url` 改为 `http://仿真主机IP:8090`。

也可在终端 2 的 GUI「图像监控」页直接设置“数据集保存帧率（每路）”，
点击“开始录制前视 + 下视”。GUI 会调用同一个
`scripts/record_camera_streams.py`，结果同样写入仓库的 `records/datasets/`；
录制时帧率输入锁定，停止后可调整下一次录制的帧率。`0` 表示保存全部收到的帧。
GUI「键盘遥控」页提供 W/S 前后、A/D 左右、R/F 上浮/下潜、Q/E 左右转；
先关闭手柄控制与自动任务，再点击“启用键盘控制”。按住按键运动，松键、Esc、
失焦、切换页面或关闭 GUI 均发送零推力。页面默认每轴最大归一化推力为 `0.2`，
可在启用前调整；零推力不等同物理急停。


给我一个python直接完成数据集整理，你需要把画面直接切开，前视和下视都要，datasets文件夹中的所有子文件夹中的内容都要，并俺200份一组打包为zip且注意，三个文件夹中的图像名称相同，不要覆盖，直接给我数据zip

1.抓