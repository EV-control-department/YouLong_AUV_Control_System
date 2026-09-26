# uv_image_transport

独立的 Python 图像传输库，使用 iceoryx2 Python binding 在进程间传递相机帧。它不启动 ROS 节点，也不定义 ROS 图像话题。

公共接口位于 `uv_image_transport.iceoryx2`：

- `Iceoryx2Publisher` / `Iceoryx2Reader`
- `FrameHeader` / `FramePacket` / `Iceoryx2Error`
- `CAMERA_FRONT`、`CAMERA_DOWN`、`ENCODING_BGR8`

帧使用 iceoryx2 user header 类型名 `YoulongCameraFrameHeader`。头部字段顺序、宽度和对齐、BGR8 编码值、相机组标识及服务名约定沿用原实现；服务名由调用方从 `auv_protocol.topics` 取得。

运行环境必须预先安装仓库使用的 iceoryx2 Python binding；本包不会构建或捆绑该 binding。
