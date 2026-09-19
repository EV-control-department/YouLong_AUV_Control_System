# YouLong AUV 坐标系约定 V1

固定坐标系树由 `auv_description` 通过
`robot_state_publisher` 发布：

```text
map -> odom -> base_link
                 ├── front_camera_link
                 │    ├── front_left_camera_optical_frame
                 │    └── front_right_camera_optical_frame
                 ├── downward_camera_link
                 │    ├── downward_left_camera_optical_frame
                 │    └── downward_right_camera_optical_frame
                 ├── imu_link
                 └── usbl_link
```

`docs/auv元件说明和标定数据_第四版.pdf` 没有给出 DVL 的安装位置和姿态，
因此真实 profile 当前不会发布未经标定的 `dvl_link`。收到实测 DVL 外参后，
应先补充 `auv_description/config/real_components.yaml`，再把该固定 link 加入
真实 URDF；仿真 profile 的 `dvl_link` 仍可按 Stonefish 场景使用。

固定变换只发布到 `/auv/tf_static`。动态的
`odom -> base_link` 变换属于定位器，并发布到 `/auv/tf`；Stonefish
真值不得作为 TF 的来源。

距离单位为米（m），线速度单位为 m/s，角速度和内部角度单位分别为
rad/s 和 rad。相机光学坐标轴遵循 ROS 约定（`x` 向右、`y` 向下、`z`
向前）；`base_link` 遵循 AUV 的 FRD 约定。
