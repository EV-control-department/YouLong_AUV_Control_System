# AUV 接口

V1 接口协议维护在
[`docs/architecture/topic_contract.md`](../architecture/topic_contract.md)；
程序化接口注册表位于
`workspace_auv/src/auv_protocol/auv_protocol/topics.py`。

真实端和仿真适配器共用的消息定义位于 `uv_msgs`。固件专用消息仍位于
`zit6_interfaces`，并限制在硬件/仿真适配器边界内使用。
