# uv_mapping（已弃用）

弃用日期：2026-09-26。

本包当前没有 mapping 节点、launch、算法或运行入口；uv_mapping.contracts 只从 auv_protocol.topics 重新导出三个常量：

- MAPPING_LANDMARKS
- MAPPING_KEYFRAMES
- MAPPING_LOCALIZATION_OPPORTUNITY

新代码请直接从 auv_protocol.topics 导入这些常量。仓库内没有发现其他包依赖此 shim；确认仓库外的工作区和部署脚本也不再导入 uv_mapping 后，可以删除本包。实际 mapping 功能需要落地时，再根据节点、数据模型和接口重新确定包边界。
