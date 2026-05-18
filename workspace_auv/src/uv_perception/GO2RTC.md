# go2rtc 集成

`vision` 节点会自动启动 go2rtc，并将前、下相机的原始视频注册为 `front` 和
`down`，将 YOLO 识别后带框的视频注册为 `front_annotated` 和
`down_annotated`。go2rtc 的 HTTP/WebRTC 服务默认监听 `1984` 端口。

实机视频链路为：

```text
V4L2 → OpenCV（进程内）→ YOLO
                    ├→ 本地 MJPEG :8090 → go2rtc :1984
                    └→ DetectionArray / LineState / ArUco（ROS 2）
```

实机模式不会把原始或标注图像封装成 `sensor_msgs/Image` 发布到 DDS；ROS 2
只传输检测和状态元数据。vision 默认使用实机 V4L2 输入，仿真启动文件会显式
设置 `sim_mode:=true`，通过 `/auv/*/stitched` 接收模拟器图像。

## 部署

在目标机器上执行：

```bash
cd /path/to/YouLong_AUV_Control_System
bash third_party/go2rtc/download_go2rtc.sh
cd workspace_auv
colcon build --packages-select uv_perception --symlink-install
source install/setup.bash
```

下载脚本会根据 `uname -m` 选择 `amd64`、`arm64` 或 `armv7` 二进制。
二进制保存在仓库的 `third_party/go2rtc/go2rtc`。vision 会优先查找
`GO2RTC_BIN`、系统 PATH 和仓库中的该文件；不依赖某个用户的 home 目录。
go2rtc 不作为 Python 包的 `data_files` 打包，以避免 `colcon
--symlink-install` 跨出工作区创建外部链接。

也可以通过环境变量指定已有二进制：

```bash
export GO2RTC_BIN=/opt/go2rtc/go2rtc
```

或通过 ROS 参数指定：

```bash
-p gortc_executable:=/opt/go2rtc/go2rtc
-p gortc_http_port:=1984
-p mjpeg_port:=8090
-p enable_gortc:=true
```

## 查看视频

```text
http://设备IP:1984/stream.html?src=front
http://设备IP:1984/stream.html?src=down
http://设备IP:1984/stream.html?src=front_annotated
http://设备IP:1984/stream.html?src=down_annotated
```

vision 本地 MJPEG 源为 `8090` 端口。go2rtc 不可用时，原始视觉检测仍能运行，
也可以直接访问 `http://设备IP:8090/front`、`/down`、`/front_annotated` 或
`/down_annotated`。如需关闭标注流，可设置 ROS 参数
`-p stream_annotated:=false`。
