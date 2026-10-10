# uv_image_transport

独立的 Python 图像传输库，使用 iceoryx2 Python binding 在进程间传递相机帧。它不启动 ROS 节点，也不定义 ROS 图像话题。

公共接口位于 `uv_image_transport.iceoryx2`：

- `Iceoryx2Publisher` / `Iceoryx2Reader`
- `FrameHeader` / `FramePacket` / `Iceoryx2Error` / `InvalidFrameError`
- `CAMERA_FRONT`、`CAMERA_DOWN`、`ENCODING_BGR8=1`、`ENCODING_JPEG=2`

帧使用 iceoryx2 user header 类型名 `YoulongCameraFrameHeader`。头部字段顺序、宽度和对齐、BGR8 编码值、相机组标识及服务名约定沿用原实现；服务名由调用方从 `auv_protocol.topics` 取得。

运行环境必须预先安装仓库使用的 iceoryx2 Python binding；本包不会构建或捆绑该 binding。

新相机发布 JPEG：`width`、`height` 为压缩图像的实际尺寸，`stride=0`，payload 为完整的
JPEG 字节；长度来自 Iceoryx2 slice，不再使用 `height * stride`。帧头仍为 48 字节。
Reader 和 Publisher 校验 SOI/EOI、帧头尺寸与编码，不在传输阶段解码像素；
`FramePacket.bgr()` 在消费端解码一次，并校验解码尺寸。旧 BGR8 的 stride/payload
规则继续支持。无效帧抛出 `InvalidFrameError`，消费者丢弃该帧并限频记录日志。
Publisher 的初始 slice 容量为 1 MiB，使用 PowerOfTwo 按需增长。

真机在发布前已完成 JPEG 压缩域 180° 旋转，仿真 JPEG 保持仿真原方向。
所有新消费者无需再旋转。更新时统一重启相机、感知、录制与推流；旧消费者不识别 JPEG。
