# competition 录制修正与 session 报告

已检查 `/home/laurie/AUV_2026_Robocup/sessions` 中两份会话。
competition 原本排除 DDS 图像消息，像素通过 HTTP MJPEG → FFmpeg → HLS/TS 单独保存。
20261010 会话的前、下视子进程各重启 751 次，两份会话均没有实际 `.ts`/`.mjpg` 文件。
旧程序把 showinfo 解码到的一帧计为 recording，吞掉非 showinfo 的 FFmpeg 错误，
无完整分段时提前返回，留下错误的 recording 状态，还删除中断 `.ts.tmp`。
所以“录制中”和帧索引不是像素落盘的证据。现场具体编码器/输入错误无法由旧日志确认。

当前修正：

- 比赛默认 5 秒 HLS 分段，首段落盘且帧新鲜才通过启动检查。
- 编码错误完整进入子进程日志，状态保存最近诊断、真实字节数、完整分段数和失败原因。
- H.264 失败自动保存可恢复 JPEG 分块；同样限制帧率/宽度，容量保护继续有效，但原 H.264 码率预算不再适用。
- 正常收尾通过 FFmpeg 专用管道发 `q`；本地测试发现信号退出可造成收尾超时和空清单，`q` 保留完整尾段和清单。
- 异常尾段保留，重启跳过其编号；连续短时失败采用退避。
- 直接 MJPEG 不关联另一条 go2rtc 编码流的 PTS，明确保留接收时钟近似同步。
- 保存任务变化、建图观测和 AprilTag 诊断；最终地图和状态变化绕过 1 Hz 限流。
- bag 补录原始 odom、目标位置/观测和 setpoint/servo/light，比赛默认不存全量分割 mask；回放不发送控制指令。

新增 `uv_record/uv_record/analyze.py`，安装入口为 `ros2 run uv_record analyze`。
输出独立 HTML、地图/路径/任务/AprilTag SVG、JSON 和 CSV、MP4 播放副本及识别角点图片。
读取原 session，不修改原文件；SQLite 缺失索引补读使用临时 DB/WAL/SHM 副本。
任务结束若由下一任务推断会明确标记；未结束的录制尾部不冒充任务成功。

已生成的真实会话报告：

- [20260519 报告](20260519_001406_report/report.html)：三轮地图，日志恢复最终分类；位置/协方差来自末次过程快照。在线解码记录 ID 18。
- [20261010 报告](20261010_180812_report/report.html)：轨迹、四轮任务共八个任务条目、315 条 AprilTag 诊断；没有建图快照，解码 ID 为空。

两份真实会话都缺少像素，报告不能产生现场视频或现场 AprilTag 角点。
已有在线识别/融合坐标可以可视化，但不等同于图像证据。

验证：uv_record 包构建成功；39 项回归通过，其中三项使用本机模拟 MJPEG 相机和真实
FFmpeg：前视 H.264、下视 H.264、无效编码器触发 JPEG 兜底，均验证完整保存、正常停止、
MP4 播放副本和 ID 18 像素角点图片。另验证实际 ROS 消息结构、最终地图日志恢复、
无像素和截断索引报告、异常重启及尾段编号保护。尚未进行实艇录制和一小时容量实测。

```bash
ros2 run uv_record record --profile competition
ros2 run uv_record analyze /绝对路径/session --output /绝对路径/新报告目录
```

直接 Python 用法、采样参数和输出文件说明见 [uv_record README](../../../uv_record/README.md)。
