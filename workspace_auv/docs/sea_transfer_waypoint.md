# 海参运输可选中间航点

在 `src/uv_task/config/tasks/grab_sea_cucumber.yaml` 的 `params` 中设置：

```yaml
transfer_waypoint:
  enabled: false
  pose: []
  timeout_seconds: 30.0
```

默认关闭，坐标留空待现场测量。启用时把 `enabled` 改为 `true`，
`pose` 填为 `[x米, y米, z米, yaw度]`，使用本次 START 后的 odom 坐标系，z 向下为正。
修改后重启 task_runner。competition.yaml 中的海参任务会继承此设置。

启用后的路线：

- 抓取并离底回位后 → 中间航点 → delivery.pose → 原有对准/释放。
- 投放完成并上浮后 → 同一中间航点 → search.pose；通过中间航点后恢复途中海参识别接管。

首次从外部任务航路进入搜索区、抓后局部回位，以及收集框对准失败后的局部兜底不重复经过中间点。
航点使用 fixed_yaw 移动：原地调深归向，再以不超过0.4米的步长分段直线平移。
现场必须确认两段直线都绕开九宫格；单个航点不提供自动避障。

关闭时不检查坐标和超时。启用但坐标未填、值无效、未到达或动作异常时，
只记录 warning，跳过可选点并继续尝试原目的地，不因此返回海参任务失败。
因此航点失败后的直达路线可能重新穿过九宫格，应明确了解这一继续执行行为。
人工停止、定位等原有保护及总超时继续生效；不会清除停止状态或延长现有总预算。
timeout_seconds 是每次尝试该点的局部预算，往返分别计时，均计入海参及全链时间。

此接口不影响 competition.yaml 中三段外部 world_waypoints 航路。
