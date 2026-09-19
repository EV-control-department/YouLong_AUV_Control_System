# YouLong AUV 系统架构 V1

本仓库采用两个 workspace：`workspace_auv` 是不依赖 Stonefish 的真机/通用
控制栈，`workspace_sim` 只提供 Stonefish、SIL/HIL 适配器和实验工具。
两个环境在 canonical ROS 2 接口之后共享控制、任务、感知和定位代码。

```text
真实传感器 / Stonefish 传感器
            ↓
       L1 适配器
            ↓  /auv/sensors/*
       uv_perception
            ↓  /auv/perception/*
      uv_localization
            ↓  /auv/state/odom, /auv/state/twist, /auv/tf
   uv_planning / uv_control / uv_task
            ↓
     /auv/hardware/* 或 /auv/sim/*
```

Stonefish 的 `/auv/sim/ground_truth/*` 与估计状态严格分离，只可被
`uv_sim_evaluation`、调试可视化和离线记录器使用。它不是
定位、规划、任务或控制的输入。

当前增量重构保留 `uv_nav` 作为规划兼容包、`uv_camera` 作为相机与感知兼容
包。仿真 bridge 的实现归属 `uv_sim_bridge`；`uv_sim` 提供稳定的
`world`/`vehicle` public launch wrapper，`uv_sim_bringup` 保留实现编排和旧
`scenario_desc` 兼容入口。
新边界已经通过
`auv_protocol`、`uv_localization`、`uv_sim_evaluation` 和
`uv_sim_degradation` 固化，后续可逐个拆分内部实现。

## 运行模式

- `real`：`auv_description` 的真实 URDF、`uv_hm`、`uv_localization` 和真机
  传感器/控制节点。
- `sim`：`uv_sim_description` 的仿真 URDF、Stonefish、仿真传感器适配器、bootstrap 定位器
  和相同的下游节点。
- `hil`：sim world + 真实 ZIT6 控制核/MCU，通过 `/auv/hardware/zit6/*`
  交换状态和命令。

所有新增 topic、service、action 均位于 `/auv`；旧的无 `/auv` 名称只能留在
兼容桥中。
