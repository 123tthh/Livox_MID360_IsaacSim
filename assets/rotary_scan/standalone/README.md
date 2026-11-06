# Livox MID-360 Rotary Scan standalone asset

Import or reference `Livox_MID360_Rotary_Scan.usd` in Isaac Sim 6.2.
It combines the common MID-360 CAD assembly with the deterministic 40-line
ROTARY approximation at 10 Hz and approximately 200,000 points/s.

The wrapper references `../../common/Livox_MID360_CAD.usd`; keep the repository
directory structure when copying it. The default prim is `/MID360`, and the
sensor prim is `/MID360/torso_link/mid360_link/mid360_native_approx`.

This profile has no compressed petal trajectory and needs no runtime pattern
driver. Use the common `scripts/publish_mid360_ros2_isaacsim62.py` when ROS 2
PointCloud2 output is required.

## IMU 使用

独立资产内置 `/MID360/torso_link/mid360_imu`，类型为 `IsaacImuSensor`，
周期 `0.005 s`（200 Hz）。LiDAR 与 IMU 在组件原点同位同向；安装或引用
`/MID360` 时无需再单独调整 IMU 位姿。

6.2 发布模块同时发布 `/mid360/imu`，`frame_id=mid360_link`，单位为 m/s²、rad/s。
这是旋转近似配置；完整 LIO 融合性能未在本仓库验收。
