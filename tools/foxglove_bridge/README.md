# Foxglove Bridge

这是可选的调试服务。默认镜像不安装 Bridge，默认 Compose 不启动 Bridge。
Foxy 使用固定的官方 0.2.2 commit
`80e6a977c773cbc6018db891c4ca504ab2b8293a` 加本目录的兼容补丁；其他 ROS
发行版使用 `ros-${ROS_DISTRO}-foxglove-bridge` 二进制包。

## 构建和启动

在工程根目录执行：

```bash
INSTALL_FOXGLOVE_BRIDGE=1 docker compose build auv
# 启动 AUV，并等待日志中的工作空间准备完成。
docker compose up -d auv
docker compose logs -f auv
# 准备完成后单独启动调试服务。
docker compose --profile tools up -d foxglove_bridge
docker compose logs -f foxglove_bridge
```

客户端使用 Foxglove WebSocket 连接 `ws://<AUV_IP>:8765`。
Bridge 使用 host 网络，与 AUV 使用同一 `ROS_DOMAIN_ID`；主服务和 Bridge 的
`ROS_DOMAIN_ID` 默认均为 0，可以在 `.env` 或命令环境中统一覆盖。
`FOXGLOVE_BRIDGE_PORT` 和 `FOXGLOVE_BRIDGE_ADDRESS` 分别覆盖端口和监听地址。

```bash
FOXGLOVE_BRIDGE_PORT=8766 docker compose --profile tools up -d foxglove_bridge
docker compose stop foxglove_bridge
```

Bridge 启动只加载已构建的消息接口，不触发主工作空间的构建。
`workspace_auv/install` 包含 `uv_msgs` 和 `zit6_interfaces`；如有单独的
ZIT6 install，也会加载。未安装 Bridge 或消息包未就绪时，启动脚本会报错退出。
无需添加 `ENABLE_FOXGLOVE_BRIDGE`：运行开关就是 Compose 的 `tools` profile。

## Foxy 兼容补丁

官方 0.2.2 直接在 Foxy 上编译会缺少 `rclcpp::GenericSubscription`、
`GenericPublisher`、节点上的创建方法和较新的 QoS getter。
`foxy-0.2.2.patch` 使用 Foxy 的 `SubscriptionBase` / `PublisherBase` 和
`rosbag2_cpp` 类型加载器补齐动态 CDR 收发，并使用 RMW QoS profile 读取配置。
类型支持库的生命周期覆盖对应订阅/发布对象。不会替换系统的 rclcpp。

动态收发实现参考 ROS 官方
[rosbag2 Foxy generic_subscription.cpp](https://github.com/ros2/rosbag2/blob/foxy/rosbag2_transport/src/rosbag2_transport/generic_subscription.cpp)
和 [generic_publisher.cpp](https://github.com/ros2/rosbag2/blob/foxy/rosbag2_transport/src/rosbag2_transport/generic_publisher.cpp)，
保留原 Apache 2.0 版权声明。

## 容器验证结果

在独立 Foxy 容器中验证了：

- Release 编译成功。
- Foxglove v1 WebSocket 握手、话题发现和非空消息 schema。
- `std_msgs/msg/String`、`uv_msgs/msg/PidGains`、`zit6_interfaces/msg/ZitPid`
  的 ROS → WebSocket 和 WebSocket → ROS 实际数据一致。
- reliable、best-effort、transient-local 订阅及取消订阅。

验证使用隔离 ROS domain、只读工程挂载和测试端口 18765。
Jazzy 的二进制安装分支尚未做运行验证。
