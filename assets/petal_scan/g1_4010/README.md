# Unitree G1 4010 + Livox MID-360 Petal Scan

`Unitree_G1_4010_MID360_Petal_Scan.usd` is the self-contained 29-DoF G1 4010 asset
(`mode_machine=5`). It has 29 revolute joints and no external USD references.
Its Isaac Sim 6.2 `OmniLidar` contains all 40 native states of the official
four-second Livox trajectory at 200,000 emitted rays/s and 10 Hz.
RTX advances the states directly; no runtime angle-array swapping is needed.
The sensor prim is:

```text
/g1_29dof_rev_1_0/torso_link/mid360_link/mid360_native_approx
```

The asset intentionally contains no saved ROS or motion-control Action Graph.
Use `scripts/publish_mid360_ros2_isaacsim62.py` from the repository root in Isaac Sim's Script
Editor. Exact USD and sensor-contract hashes are in `manifest.json`.

## IMU 使用

- Prim：`/g1_29dof_rev_1_0/torso_link/mid360_imu`
- 类型：`IsaacImuSensor`，周期 `0.005 s`（200 Hz）
- 位姿：与 `mid360_link` 完全同位同向，平移
  `[0.0002835, 0.00003, 0.41618]` m、滚转 180°
- ROS 2：读取该 Prim 并发布 `/mid360/imu`，`frame_id=mid360_link`
- FAST-LIO：点云和此 IMU 已在同一坐标系，使用 `extrinsic_T=[0,0,0]`、`extrinsic_R=I`
