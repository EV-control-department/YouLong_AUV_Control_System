# YouLong AUV 实验运行器

实验输出统一放在：

```text
results/<run_id>/
├── config.yaml
├── metadata.json
├── rosbag/
├── trajectory.csv
├── metrics.json
└── log/
```

`run_experiment.py` 负责建立可重复的目录和 metadata；ROS launch 命令通过
`--command` 传入，`--seed` 会同时写入配置与 metadata。这样 nominal、DVL
丢失、视觉退化和组合退化实验使用相同的结果格式。
