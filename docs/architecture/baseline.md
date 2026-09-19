# V1 重构基线

基线分支为 `robotcup`，协议基线文件为
`docs/architecture/baseline_manifest.yaml`。基线的目的是冻结接口和可观测
行为，而不是把大型 rosbag 放入代码仓库。

## 记录建议

在具备完整 ROS 环境后，先启动 `uv_sim_bringup`，再记录至少一段包含
传感器、估计状态、控制命令和真值评测的 rosbag：

```bash
ros2 bag record -o results/baseline \
  /auv/sensors/imu/data \
  /auv/sensors/dvl/velocity \
  /auv/sensors/pressure \
  /auv/state/odom /auv/state/twist \
  /auv/control/trajectory /auv/sim/ground_truth/odom
```

真值只被记录/评测，不应被控制、规划或任务订阅。记录完成后把 bag 路径、
commit 和运行参数写入实验目录的 `metadata.json`。

## 当前自动验收

```bash
git diff --check
python3 -m py_compile ...
ros2 topic info -v /auv/sim/ground_truth/odom
```

另外，`uv_sim/test/test_ground_truth_isolation.py` 对 SIL bridge 做静态真值
隔离检查，`uv_sim_evaluation` 的运行时订阅是允许的唯一评测路径。
