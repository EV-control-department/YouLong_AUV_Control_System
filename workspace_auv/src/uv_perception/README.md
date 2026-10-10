# JPEG 感知、镜头去畸变与双路模型

## 像素链路

真机 `uv_camera` 读取 V4L2 MJPG，压缩域无损旋转 180° 后发布 JPEG；仿真拼接后编码一次 JPEG，不旋转。Iceoryx2 的 48 字节帧头和服务名不变。

`object_detector` 每个拼接帧只解码一次。四眼分别缓存镜头去畸变映射，保持每眼原尺寸、完整 K（含 skew）和光学坐标系，输出 D=0、R=I、P=[K|0]。不做极线校正、额外旋转、裁剪或 JPEG 重编码。零畸变输入跳过重采样。只有左右眼标定与本路模型、类别表都就绪后才检测；单路故障不影响另一条链路。

检测框、中心、门架特征、分割多边形、导引线和圆环方向均使用校正后的每眼图像坐标。无效框及无效区域内的中心/特征点被丢弃。ArUco 使用校正后的前视双目图。

## 模型与类别发布

新增 `front_model_path`、`down_model_path`，各自拥有独立 YOLO 实例。路径优先级：本路参数 → 原有 `model_path` → `UV_YOLO_MODEL` → 包内 `HQQ7_aug.pt`。即使路径相同，也不跨线程共享模型。现有 confidence 语义不变：单独启动默认 0.5，真机 bringup 默认 0.8。

共享类别表默认 `weights/HQQ7_aug.yaml`，仍支持 `UV_MODEL_MAPPING_FILE`。两路权重的类别 ID 和规范化名称必须与这份表一致；不一致时仅该路报告模型故障。ModelClassMapping 描述共享类别表；各路实际权重路径在检测器日志和健康消息中报告。

发布过滤严格遵循 YAML 的 `camera`：`front` 仅前视、`down` 仅下视、`any` 两路均允许；非法取值报错，未知 ID 不发布。两路 `class_id` 不加偏移，用 `camera_name` 区分前下视及左右眼。过滤后空帧仍发布空 DetectionArray。`multi_instance` 不用于压缩检测结果数量。分割与检测过滤保持索引对应，导引线也只使用允许发布的结果。

```bash
ros2 launch uv_perception perception_launch.py \
  front_model_path:=/models/front.pt down_model_path:=/models/down.pt
ros2 launch uv_bringup real.launch.py enable_camera:=true \
  front_model_path:=/models/front.pt down_model_path:=/models/down.pt
ros2 launch uv_sim sim.launch.py \
  front_model_path:=/models/front.pt down_model_path:=/models/down.pt
```

`uv_sim_bringup` 的 sim/HIL 入口也接受这两个参数。真机 profile 默认仍不启动相机，可稍后单独启动；不需要更改整个 bringup 的启动顺序。adopt 模式会检查双路权重参数是否一致。

## 坐标与标定接口

每眼发布可靠、TRANSIENT_LOCAL 的 `uv_msgs/PerceptionCameraInfo`：

- `/auv/perception/camera/front/{left,right}/calibration`
- `/auv/perception/camera/downward/{left,right}/calibration`

消息包含 `camera_name`、源 `camera_info_version`、`calibration_id`、原始 `source_info` 和校正后的 `image_info`。标定 ID 来自尺寸、K/D、源版本及处理约定的 SHA-256 摘要前 64 位，不受消息时间戳变化影响。映射仅在标定或源版本变化时重建。

DetectionArray 新增 `image_space`（0=RAW、1=UNDISTORTED）、`calibration_id` 和 `camera_info_version`。定位和任务必须找到匹配标定后才使用校正坐标。共享像素转射线函数使用完整 K；UNDISTORTED 不再应用原始 D。任务中的前视搜索、门框跟踪、下视伺服及三角定位已接入，左右眼各用自己的内参。由检测框派生的方向端点通过 reference 参数继承父检测的标定。

原始 CameraInfo 主题继续描述 Ice JPEG。配置 K/D 继续采用已有方向修正后的眼图坐标约定；运行期间固定标定，修改后统一重启相关节点。

## 推流与录像

普通流解码原始 JPEG 后直接缩放；标注流选帧后解码、按相同映射校正、缩放到 1280×480，再叠框和 H.264 编码。画框须匹配相机、坐标系、标定 ID 和源帧；时间回退、capture ID 回退、源标定版本改变时清理旧缓存。标定尚未到达时保持无标注原图推流，并限频报告。

raw JPG 始终是收到的 Ice JPEG 原字节，不包含感知去畸变，不解码、不重编码。录制默认包含上述小型标定主题，manifest 的 perception 段记录检测 schema=2、检测坐标 undistorted、raw 坐标 distorted。

回放及 MCAP 导出支持旧 DetectionArray CDR，通过内部 LegacyDetectionArray 转成 RAW；新 schema 的截断检测消息不得退回旧解析。新旧 JPG/PNG 图片读取行为保留，CompressedImage 仍按实际文件标记 jpeg/png。回放标定和类别表使用持久化 QoS，支持迟加入消费者。

## 部署与验证

Foxy/Python 3.8 和 Jazzy 均需重新构建 uv_msgs 与消费者，禁止旧消费者直接接收新的 DetectionArray。停止相机/感知/任务/推流/录制相关进程，构建后统一加载同一套 overlay 并重启：

```bash
source /opt/ros/foxy/setup.bash  # Jazzy 环境改为 jazzy
cd workspace_auv
colcon build --packages-up-to uv_bringup uv_task uv_stream uv_record
source install/setup.bash
```

相机的 libturbojpeg 依赖和 Docker libturbojpeg0-dev 保留。去畸变和第二个模型会增加 CPU/内存；共享内存、raw 存盘仍保持 JPEG 优势。性能以目标机器实测为准；验证标定方向、左右眼、控制误差和持续运行吞吐后再用于真机任务。
