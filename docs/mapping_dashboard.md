# 建图调试上位机

`visualization/mapping_visualizer.py` 是建图任务专用的 DDS 监控面板。它不依赖本地文件或仿真场景内容，所有显示数据均来自运行中的 ROS 2 话题。

## 启动

```bash
cd YouLong_AUV_Control_System
source /opt/ros/humble/setup.bash
source workspace_sim/install/setup.bash
source workspace_auv/install/setup.bash
python3 visualization/mapping_visualizer.py
```

面板使用 Qt 界面和离屏 Matplotlib 渲染，不调用 Qt Matplotlib 后端，因此兼容当前的 PySide6/Matplotlib 版本组合。

## 主要视图

- 三维地图：显示九宫格理论位置、AprilTag、AUV、每一帧世界坐标原始测量点，以及卡尔曼滤波后的聚类中心。
- 聚类信息表：显示每个格点的原始点数量、滤波接受数量、类别、估计位置和水平残差。
- stitched 输入：显示当前下视双目拼接图。
- 分割掩膜叠加：将 YOLO-Seg 掩膜绘制到左目图像上；橙色为方形锥桶，蓝色为圆形锥桶。
- 视差图：使用当前下视左右目图像运行 SGBM，颜色越亮表示视差越大。
- 深度图：使用 `Z = fx * baseline / disparity` 计算深度。
- 深度频率图：显示有效深度像素的频率分布，并标出掩膜内合理峰值，便于调节 `depth_bin_m`、`depth_peak_ratio`、`min_depth_points` 和 SGBM 参数。
- DDS 事件：显示同步拒绝、深度峰值拒绝、越界拒绝和有效测量等事件。

## DDS 数据扩展

建图地图消息的 `schema_version` 已升级为 2。每个 `cells[]` 元素新增：

```json
{
  "measurements": [
    {
      "position": [2.0, -4.0, 1.99],
      "class_id": 0,
      "confidence": 0.91,
      "depth_m": 1.02,
      "residual_m": 0.08,
      "accepted": true,
      "timestamp": 123.4
    }
  ]
}
```

`accepted` 只代表该点是否通过格点门限和卡尔曼门限。原始点仍然保留，因而可以同时检查点云离散程度、误检位置和滤波结果。每个格点最多保留 120 个点，避免 DDS 地图消息无限增长。

## 诊断解释

图像同步指标显示 stitched 图像与最近检测消息的时间差。它只用于观察，不改变建图任务的时间同步判定。若频率图没有有效峰值，优先检查：

1. YOLO 分割掩膜是否为空或越界；
2. 左右图是否仍为同一立体帧；
3. `num_disparities`、`block_size` 与水下纹理是否匹配；
4. 深度范围、峰值比例和最小采样点数是否过严；
5. 机器人是否在观察期间发生明显位移。
