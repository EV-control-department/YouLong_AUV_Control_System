# 仿真真值隔离验收

仿真真值话题：

```text
/auv/sim/ground_truth/odom
/auv/sim/ground_truth/twist
```

允许订阅者只有：

- `uv_sim_evaluation`；
- 调试可视化；
- 离线记录器。

正式定位、控制、规划和任务必须通过：

```text
/auv/state/odom
/auv/state/twist
/auv/tf
```

验收命令：

```bash
ros2 topic info -v /auv/sim/ground_truth/odom
```

输出中不应出现 `uv_localization`、`uv_control`、`uv_planning`、`uv_task`
或 ZIT6 控制适配器。代码级检查应确认 `uv_sim` 不导入
`SIM_GT_ODOM`，并且 `uv_sim_evaluation` 是仿真真值唯一的运行时订阅者。

偏差注入验收：在 `/auv/state/odom` 注入约 2 m 的估计偏差后，控制轨迹必须
表现出相应偏差；若仍与 Stonefish 真值完全重合，说明存在真值泄漏路径。
