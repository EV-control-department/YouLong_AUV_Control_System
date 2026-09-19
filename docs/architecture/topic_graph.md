# 标准话题关系图 V1

```text
Stonefish / 真实设备
          │
          ▼
  适配器层
          │ /auv/sensors/*
          ▼
  uv_camera / uv_perception
          │ /auv/perception/*
          ▼
  uv_localization
          │ /auv/state/odom, /auv/state/twist, /auv/tf
          ├───────────────┬────────────────┐
          ▼               ▼                ▼
       规划            控制              任务
          │               │                │
          └────── /auv/control/* ─────────┘
```

仿真专用数据单独隔离：

```text
Stonefish ──► /auv/sim/ground_truth/* ──► 仅供 uv_sim_evaluation
Stonefish ──► /auv/sim/raw/* ──► uv_sim_bridge ──► /auv/sensors/*
```

旧的 `/task/*`、`/basic_motion*` 和 `/zit6/*` 名称仅用于兼容接口。新的发布者、
订阅者、服务和 Action 都通过 `auv_protocol.topics` 注册，并使用 `/auv` 前缀。
