# MID-360 测试场景

本目录提供三份可直接用 Isaac Sim 6.2 打开的测试 Stage：

| Stage | 阶高 | 环境 | 默认传感器 |
| --- | --- | --- | --- |
| `MID360_Test_H06cm.usda` | 0.06 m | 坡道、平台、4 级上下阶梯、围墙 | Petal Scan 独立 MID-360 |
| `MID360_Test_H10cm.usda` | 0.10 m | 坡道、平台、4 级上下阶梯、围墙 | Petal Scan 独立 MID-360 |
| `MID360_Test_H15cm.usda` | 0.15 m | 8 级上下阶梯 | Petal Scan 独立 MID-360 |

`stairs/` 内是移植后的自包含碰撞环境，不引用 Lidar_Hiking，也不需要额外网格、材质或纹理。
顶层 Stage 添加了灯光、PhysicsScene，以及位于 `(0, 0, 1.2)` m、绕 X 轴旋转 180°
的独立 MID-360，因此克隆仓库后可以直接测试点云。

## Isaac Sim 6.2

1. 打开本目录任意 `MID360_Test_H*.usda`。
2. 按根 README 用内置 Jazzy 启动 6.2，并启用 `isaacsim.ros2.bridge`。
3. 在 Script Editor 运行仓库根目录的
   `scripts/publish_mid360_ros2_isaacsim62.py`。
4. 按 Play，通过 `/mid360/points` 查看点云；IMU Prim 为
   `/World/MID360/torso_link/mid360_imu`。
5. RViz2 中将 Fixed Frame 设为 `mid360_link`，添加 PointCloud2，话题选择
   `/mid360/points`。

Petal Scan 直接使用 RTX 的 40 个原生发射状态，无需角度数组切换脚本。
只需 Python 点云时使用 `scripts/mid360_runtime_isaacsim62.py`。

## 切换雷达资产

顶层 Stage 的 `/World/MID360` 默认引用：

```text
../../assets/petal_scan/standalone/Livox_MID360_Petal_Scan.usd
```

要测试普通旋转扫描，将该引用替换为：

```text
../../assets/rotary_scan/standalone/Livox_MID360_Rotary_Scan.usd
```

要测试 G1 4010/5010，在 Stage 中删除 `/World/MID360`，引用资产矩阵中的机器人 USD 到
`/World/G1`，将机器人放在起始平面上方，并使用自己的关节保持或控制器。测试场景本身不固化
任何机器人控制或建图组件；这些组件应由使用者在仓库外自行配置。

## 自动验证

```bash
make check
/path/to/isaac-sim/python.sh scripts/smoke_test_assets_isaacsim62.py
```

`make check` 验证场景哈希、阶高、碰撞、默认 Prim 与相对引用；Isaac Sim 冒烟测试会解析三个
组合 Stage，并在 10 cm 场景创建一次 RTX render product。
