# Livox MID-360 standalone asset for Isaac Sim 6.2

Import or reference `Livox_MID360_Petal_Scan.usd`. It contains:

- the MID-360 CAD assembly from `mid-360-asm.stp` (22 source meshes, 77,565 triangles);
- an Isaac Sim 6.2 `OmniLidar` configured for 200,000 points/s and 10 Hz;
- all 800,000 directions from the official Livox four-second non-repetitive scan table.

Keep the repository structure because the wrapper references
`../../common/Livox_MID360_CAD.usd` relatively.
The wrapper stage uses metres and Z-up; the CAD millimetre coordinates are
scaled by 0.001. Its default prim is `/MID360`, and the sensor is at
`/MID360/torso_link/mid360_link/mid360_native_approx`.

All 40 states are native 6.2 emitter states. Create a `MID360Runtime` reader
or run `scripts/publish_mid360_ros2_isaacsim62.py` after enabling the ROS bridge,
then press Play. No angle-swapping driver is needed. See the root README for
startup environment and 200 Hz physics settings.

The scan table is redistributed under the Livox SDK2 BSD-3-Clause license; see
the source repository's
`assets/common/mid360_official_pattern/LICENSE.livox_laser_simulation`.
The CAD geometry is derived from the repository owner's supplied STEP assembly
and is included here as the standalone Isaac Sim import asset requested for
this project.

## IMU 使用

独立资产内置 `/MID360/torso_link/mid360_imu`，类型为 `IsaacImuSensor`，
周期 `0.005 s`（200 Hz）。独立组件的 LiDAR 与 IMU 都位于组件原点且方向一致；
把 `/MID360` 安装到机器人时，两者会随组件一起变换。

6.2 发布模块同时发布 `/mid360/imu`，`frame_id=mid360_link`，单位为 m/s²、rad/s。
点云与 IMU 使用单位相对外参；完整 LIO 融合性能未在本仓库验收。
