# 感知观测与目标 GUI

此 GUI 属于 uv_perception，用于查看当前目标 track、对应几何测量和机器人位置。它只订阅数据，不参与定位或控制。

## 启动

先启动仿真/实机 bringup 或分别启动相机、感知组件。在另一个有桌面环境的终端运行：

```bash
ros2 run uv_perception perception_gui
```

也可以在 uv_perception/perception_launch.py 中设置 enable_gui:=true。

参数可通过 ROS 参数覆盖：

```bash
ros2 run uv_perception perception_gui --ros-args \
  -p refresh_period_ms:=150 \
  -p measurement_history_limit:=500 \
  -p association_distance_m:=2.0
```

- refresh_period_ms：界面刷新周期，默认 150 ms，最小 50 ms。
- measurement_history_limit：本地保留的观测数量，默认 500，最小 50。
- association_distance_m：GUI 将无精确 track ID 的测量临时关联到 track 时使用的距离上限，默认 2.0 m。

## 订阅接口

| 话题 | 消息 | 用途 |
|---|---|---|
| /auv/perception/tracks | ObjectTrackArray | 目标类别、来源、世界坐标、置信度、观测数和状态 |
| /auv/perception/measurements | ObjectMeasurementArray | 单次几何测量或方位观测 |
| /auv/state/odom | PoseInfo | AUV 当前姿态和位置 |

## 界面说明

上方表格显示 track ID、类别、来源、odom 坐标、置信度、观测数、状态和年龄；另一张表显示选中 track 的近期测量。勾选“显示所有物品的观测”可取消当前 track 筛选。下方平面图以 North 为上、East 为右，显示 track、带位置的测量及机器人当前位置。

测量方式包括前视双目、前视方位和下视平面交点。没有可靠三维位置的方位观测会显示射线信息，不会作为 track 位置画入地图。

GUI 默认读取 /auv/perception/tracks、/auv/perception/measurements 和 /auv/state/odom。相关检测与几何测量由 uv_perception 发布；相机图像通过 uv_image_transport 的 iceoryx2 服务传输。
