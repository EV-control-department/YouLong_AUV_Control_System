# mapping_grid 代码导读（相机内感知版）

## 从哪里读起

| 文件 | 作用 |
|---|---|
| [uv_camera/sensor.py](../src/uv_camera/uv_camera/sensor.py) | 真机 V4L2 采集；仿真直接订阅 Stonefish 左右目并在进程内配对 |
| [uv_camera/composed.py](../src/uv_camera/uv_camera/composed.py) | 进程内 FrameGate、模型与 mapping_vision 组装、HTTP 推流 |
| [uv_camera/ai.py](../src/uv_camera/uv_camera/ai.py) | 同帧左右目 YOLO-Seg；保留普通检测话题给其他消费者 |
| [uv_camera/mapping_vision.py](../src/uv_camera/uv_camera/mapping_vision.py) | 标定、立体校正、SGBM、Tag、锥桶中心/池底投影、小型观测发布 |
| [uv_task/mapping_task.py](../src/uv_task/uv_task/mapping_task.py) | 格位关联、观测池、Kalman、类别投票、巡检/遍历状态机 |
| [uv_msgs/msg/MappingObservation.msg](../src/uv_msgs/msg/MappingObservation.msg) | 单次带采集时间和质量的 Tag/锥桶测量 |
| [visualization/mapping_visualizer.py](../../../visualization/mapping_visualizer.py) | DDS 地图/事件/位姿 + camera HTTP 视觉快照，不做 SGBM |

## 一帧图像怎样变成地图

```text
Stonefish 四路原始 DDS Image 或真机两路 V4L2
  → Sensor：仿真同一左右目时间窗配对；真机直接取拼接帧
  → FrameGate：只保留每路最新帧，避免队列滞后
  → Ai._process_frame：同帧 YOLO-Seg 左右检测
  → MappingVision.process：最近采集位姿 + 标定/校正 + SGBM
      ├─ 左目 AprilTag 16h5：0–6 ID → 深度主峰 → 世界位置
      └─ 左目锥桶掩膜 → 掩膜几何中心 + 周围池底深度 → 锥桶轴线中点
  → /perception/mapping/observations (MappingObservationArray)
  → MappingTask._observation_cb
      ├─ Tag：同 ID 静态滤波，满足次数才允许离开标记
      └─ Cone：最近格位 + 残差门限 + 静态滤波/一致簇纠错 + 类别票
  → /task/mapping/map（低频快照）和 /task/mapping/events（逐事件）
```

`MappingObservationArray.header.stamp` 是相机左目采集时间；
`processed=false` 带原因，表示标定/位姿/图像处理失败。
数组还带 Tag/锥桶候选数和深度拒绝数；即使没有目标但帧处理成功，仍发空数组。
每个观测包含 kind、ID/类、置信度、深度、支持点数、odom 三维位置和位姿时间差。
不在 DDS 上传图像、分割轮廓或视差图。真机视觉链路全在 uv_camera 进程内；
仿真唯一不可避免的大图 DDS 跳是 Stonefish 自带的原始相机接口。
sim_bridge 默认不再二次发布 stitched 图像。

CameraInfo 只供 uv_camera 内部在仿真初始化标定。真机下视使用
`docs/stereo_parameters.json` 原始每目 1280×960 标定，等比缩放至每目
640×480；V4L2 左右拼接必须是 1280×480，尺寸不符即拒绝启动。实际下水前
仍需核验左右目顺序、机体外参及时间同步。

## 任务状态机

`MappingTask.execute()`：

1. 等待至少一帧 `processed=true` 的观测和 PoseInfo，确认视觉链路可用。
2. WTRAVEL 到 JSON 中的 Tag 理论 XY；`_read_tag()` 等候同 ID 多次有效观测。
3. 按 `visit_order` 去九个理论格位。任务运动串行，camera 独立持续观测；
   `_observe_cones()` 只计算停留期新增帧数/目标测量数。空格继续，WTRAVEL
   失败立即停机。
4. `_select_final_assignment()` 在两方两圆约束下选最多四个最可信格位；
   若不足四个，以部分结果报告，不伪造目标。
5. `_traverse_cones()` 返回融合 Tag XY，先圆后方，四邻接格点规划。
   空格可复用作通道；锥桶格不可二次进入。入目标格的终点使用观测融合 XY，
   不使用 JSON 理论中心替代。遍历安全监测仍需要 PoseInfo。

`StaticPositionFilter` 状态只有目标三维位置；预测 F=I，过程噪声随采样间隔
膨胀；更新使用 Joseph 形式并以马氏距离剔除离群观测。若第一帧错误，
`_process_cone_observation()` 会检查最近 12 条观测中是否形成足够大的新一致簇，
满足条件则重建该格滤波器和类别票，单个离群点不能重置轨迹。
`measurement_points` 保留原始位置及接受标志供地图/上位机诊断。

## 排查顺序

- `ros2 topic echo /perception/mapping/observations --once`：
  `processed=false` 时先看 `reason`；`tag_candidates>0` 但
  `tag_depth_rejected>0` 是测距问题，候选为 0 才看字典/纹理/视野。
- `ros2 topic echo /task/mapping/events`：看格位残差、拒绝原因、Tag
  融合与路径状态。任务不再有 `image timestamp unavailable` 配对环节。
- `http://127.0.0.1:8090/mapping/{input,overlay,disparity,depth,histogram}.jpg`：
  camera 用同帧生成诊断 JPEG，上位机只显示，不参与任务决策。
- camera 参数与模型置信度在 uv_camera 配置，九宫格、滤波和路线在任务 JSON；
  调深度直方图参数时无需改任务状态机。
