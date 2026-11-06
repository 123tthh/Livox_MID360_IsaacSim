# Unitree G1 5010 Mode13 + Livox MID-360 Rotary Scan

`Unitree_G1_5010_Mode13_MID360_Rotary_Scan.usd` combines Unitree's official Mode13 5010
wrist geometry/inertias/limits, the InstinctLab torso-rooted training collision
contract, and the repository's MID360 approximation.

The flattened USD has 29 revolute joints, no external USD references, and no
embedded ROS or motion-control Action Graph. Its sensor prim is:

```text
/g1_29dof_mode_13_5010_mid360/torso_link/mid360_link/mid360_native_approx
```

Refresh the sensor profile with the Isaac Sim 6.2 Python environment:

```bash
python scripts/migrate_assets_isaacsim62.py
python scripts/validate_assets.py
```

Source revisions, file hashes, the 5010 wrist contract, MID360 parameters and
licenses are pinned in `manifest.json` and `assets/common/g1_5010_support/`.

## IMU 使用

- Prim：`/g1_29dof_mode_13_5010_mid360/torso_link/mid360_imu`
- 类型：`IsaacImuSensor`，周期 `0.005 s`（200 Hz）
- 位姿：与 Rotary LiDAR 同位同向，平移
  `[0.0002835, 0.00003, 0.41618]` m、滚转 180°
- ROS 2：发布 `/mid360/imu`，`frame_id=mid360_link`
- LiDAR/IMU：使用单位相对外参 `T=[0,0,0]`、`R=I`
