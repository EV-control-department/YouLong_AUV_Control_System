# 仿真与真机边界 V1

`workspace_auv` 包含与硬件无关的 AUV 逻辑，可以在不安装 Stonefish 和
`uv_sim` 的环境中构建。`workspace_sim` 提供 Stonefish、SIL/HIL
适配器，以及 `uv_sim_bringup` 启动入口。

Stonefish 只发布以下仿真专用数据：

```text
/auv/sim/ground_truth/odom
/auv/sim/raw/camera/*
/auv/sim/raw/dvl/*
```

启用 `uv_sim_degradation` 时，DVL、IMU、拼接视觉和 USBL 会从 canonical
输入复制到显式的 `/auv/sim/degraded/*` 输出；可配置丢帧/丢波束、底锁丢失、
偏置、随机游走、噪声、延迟、视觉亮度/模糊和 USBL 离群点。SIL launch
通过 remap 让定位/视觉消费这些输出。默认关闭退化时，canonical 链路不经过
该节点，因此不会形成同一话题的反馈环。

仿真真值只用于评测和调试。`uv_sim` 不订阅
`/auv/sim/ground_truth/odom`，也不会把 Stonefish 位姿伪装成硬件遥测。
SIL/HIL 先由标准 DVL/IMU 输入 `uv_localization` 的 bootstrap
航位推算状态，正式控制核心只消费 `/auv/state/odom` 和
`/auv/state/twist`。未来 FGO 可以在这个包内替换 bootstrap 实现，而不
改变下游接口。

唯一允许订阅仿真真值的运行时包是 `uv_sim_evaluation`（另加明确的
调试/离线记录器）。它只能计算 ATE/RPE 等评测指标，不发布控制状态、
不发布 TF，也不向控制器回灌数据。

真机和仿真下游节点使用相同的 `/auv/sensors/*`、
`/auv/perception/*`、`/auv/state/*` 和 `/auv/control/*` 接口。
