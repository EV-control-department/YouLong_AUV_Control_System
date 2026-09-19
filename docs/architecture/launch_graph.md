# 启动关系图 V1

启动文件的归属遵循 workspace 边界：

```text
workspace_auv
  uv_bringup/real.launch.py
    ├── auv_description/description.launch.py
    ├── uv_hm/hardware_launch.py
    ├── uv_localization/localization_launch.py
    ├── uv_control/control_launch.py
    ├── uv_camera/perception_launch.py
    ├── uv_planning/planning_launch.py
    └── uv_task/task_launch.py

workspace_sim
  uv_sim/sim.launch.py (public world/vehicle entry)
    └── uv_sim_bringup/sim.launch.py
    ├── uv_sim_description/description.launch.py
    ├── stonefish_ros2/*
    ├── uv_sim_bridge/bridge.launch.py
    ├── uv_sim_degradation/degradation.launch.py (optional)
    ├── uv_localization/localization_launch.py
    └── 与 AUV 相同的控制/感知/规划/任务启动文件
```

`hil.launch.py` 使用仿真描述和场景、真实 ZIT6 传输/代理，以及相同的 AUV
下游软件包。`uv_bringup` 不包含 Stonefish 或 `uv_sim` 运行时依赖。
`uv_sim_bringup` 保留旧 `scenario_desc` 转发入口；新的用户启动命令使用
`uv_sim sim.launch.py world:=... vehicle:=youlong`。

## TF 归属

`robot_state_publisher` 是唯一的固定变换发布者，并重映射到
`/auv/tf_static`。`uv_localization` 是唯一的动态
`odom -> base_link` 发布者，并重映射到 `/auv/tf`。
