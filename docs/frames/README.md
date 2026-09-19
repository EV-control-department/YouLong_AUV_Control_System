# AUV 坐标系

权威的坐标系和单位约定见
[`frame_convention.md`](../architecture/frame_convention.md)：机体采用 FRD，
相机使用 optical 坐标轴，坐标树为 `map -> odom -> base_link`，单位分别为米、
弧度和秒。固定传感器几何关系发布到 `/auv/tf_static`；估计得到的动态位姿由
`uv_localization` 发布到 `/auv/tf`。
