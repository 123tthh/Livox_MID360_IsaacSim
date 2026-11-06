# Livox MID-360 · Isaac Sim 6.2

[English](README.md) | **简体中文**

**`main` 对应 Isaac Sim 6.2**。5.1 的完整资产和运行脚本保存在 [`isaac-sim-5.1`](https://github.com/123tthh/Livox_MID360_IsaacSim/tree/isaac-sim-5.1) 分支：

```bash
# Isaac Sim 6.2
git clone https://github.com/123tthh/Livox_MID360_IsaacSim.git
# Isaac Sim 5.1
git clone --branch isaac-sim-5.1 https://github.com/123tthh/Livox_MID360_IsaacSim.git
```

仓库直接包含 **8 个 USD 资产**：独立 MID-360、G1 4010、G1 5010 Mode13、G1 5010 Mode15，分别提供 Petal 与 Rotary 两种扫描配置。USD 二进制、CAD、官方轨迹和自包含测试场景均随 Git 上传，克隆即可取得。

## 资产入口

| 配置 | 独立雷达 | G1 4010 | G1 5010 Mode13 | G1 5010 Mode15 |
| --- | --- | --- | --- | --- |
| Petal 花瓣扫描 | `assets/petal_scan/standalone/Livox_MID360_Petal_Scan.usd` | `assets/petal_scan/g1_4010/Unitree_G1_4010_MID360_Petal_Scan.usd` | `assets/petal_scan/g1_5010_mode_13/Unitree_G1_5010_Mode13_MID360_Petal_Scan.usd` | `assets/petal_scan/g1_5010_mode_15/Unitree_G1_5010_Mode15_MID360_Petal_Scan.usd` |
| Rotary 旋转近似 | `assets/rotary_scan/standalone/Livox_MID360_Rotary_Scan.usd` | `assets/rotary_scan/g1_4010/Unitree_G1_4010_MID360_Rotary_Scan.usd` | `assets/rotary_scan/g1_5010_mode_13/Unitree_G1_5010_Mode13_MID360_Rotary_Scan.usd` | `assets/rotary_scan/g1_5010_mode_15/Unitree_G1_5010_Mode15_MID360_Rotary_Scan.usd` |

机器人 USD 是扁平化的单文件资产。独立雷达相对引用 `assets/common/Livox_MID360_CAD.usd`，复制时保留目录结构。各目录的 `manifest.json` 与 `assets/catalog.json` 记录版本和文件哈希。

Mode13 与 Mode15 分别来自官方对应 URDF，髋部减速比分别为 14.3/22.5 与 22.5/22.5，均采用 5010 腕部电机、解锁腰部和 29 个旋转关节。此次迁移沿用已有机器人几何、关节与雷达安装变换。

## 6.2 扫描实现

| 参数 | Petal | Rotary 近似 |
| --- | --- | --- |
| `scanType` | `SOLID_STATE` | `ROTARY` |
| 原生发射状态 | 40 | 1 |
| 每状态发射器 | 20,000 | 40 |
| `patternFiringRateHz` | 10 | 5,000 |
| 完整扫描帧率 | 10 Hz | 10 Hz |
| 标称发射点数 | 200,000/s | 200,000/s |
| 方向分布 | Livox 官方 80 万有序方向，4 秒循环 | 固定 40 仰角通道 |

6.2 的花瓣资产将 **40 个状态直接写入 OmniLidar**，无需 Python 定时替换角度数组。RTX 方位角符号已转换，以保持官方 Livox XYZ 方向。5.1 的压缩轨迹 Scope 和数组切换脚本仅在 5.1 分支保留。

两种配置均输出传感器坐标系中的笛卡尔点，累积为完整 10 Hz 帧；量程截断为 0.1–70 m，使用理想单回波，没有实测标定的距离/角度噪声和反射率探测概率模型。标称发射点数不等于有效回波点数。花瓣轨迹在 4 秒后重放；Rotary 用于基线调试，不代表真机扫描方式。

## 启动固定 G1 物体场景

在未 source 系统 ROS 的新终端中使用已安装 Isaac Sim **6.2** 的 Python。启动脚本会选择该安装自带的 Jazzy 库，不安装软件、不替用户接受 EULA。

```bash
# 若当前 python3 不是 Sim 6.2 环境，可显式指定：
export ISAAC_SIM_PYTHON=/你的/6.2环境/bin/python
export ROS_DOMAIN_ID=76
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
bash scripts/run_object_field_validation.sh petal

# 旋转近似；或运行 10 秒后退出的无界面测试：
bash scripts/run_object_field_validation.sh rotary
bash scripts/run_object_field_validation.sh petal 10 --headless
```

另开系统 ROS 2 Jazzy 终端，选择相同 ROS 域：

```bash
export ROS_DOMAIN_ID=76
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
bash scripts/run_rviz2_mid360.sh petal
```

RViz 的 Fixed Frame 使用正立的 `G1`。Petal 累积 4.2 秒，Rotary 保留一个 0.1 秒周期。场景内 G1 双脚接地、通过 World FixedJoint 固定，29 个关节锁零；5 m 内有十组接地目标。不加载运控策略或 SLAM。

## Python 与 Script Editor

`SimulationApp` 启动后导入 `scripts/mid360_runtime_isaacsim62.py`。在开始仿真前建立传感器输出，物理步长使用 0.005 秒以取得 200 Hz IMU；RTX motion BVH 开关见启动脚本。

若父场景时间范围为空，读取模块会在 session layer 设置一小时范围，避免 Kit 在单帧上循环播放；长期任务可自行设置场景时间范围。

```python
from mid360_runtime_isaacsim62 import MID360Runtime
sensor = MID360Runtime(stage, root_path='/World/G1')  # 也可指定独立雷达根路径
# 每次 sim.step(render=True) 之后：
cloud = sensor.read_cloud()              # 新 10 Hz 帧，或 None
imu = sensor.read_imu(sim.current_time)   # physics_dt=0.005 时为 200 Hz
# 结束时调用 sensor.close()
```

点云字典包含 `xyz`、`intensity`、`timestamp_ns`、`time_offset_ns`、`emitter_id`、`state_id`。逐点时间为 `timestamp_ns + time_offset_ns`，过滤无回波点后仍保留原始 5 μs 发射槽位。本机 6.2 的原始 `tickStates` 始终为零，读取模块先从实际 XYZ 建立初始相位，再按采集时间推进；验收另外遍历全部 40 个官方状态独立匹配，避免信任错误标签。

Script Editor 用法：在已正确加载内置 ROS 库并启用 `isaacsim.ros2.bridge` 的 6.2 会话中打开/引用资产，运行完整 `scripts/publish_mid360_ros2_isaacsim62.py`。脚本用物理步后回调发布 IMU、渲染后回调发布点云。多实例时先设置 `MID360_ROBOT_ROOT_PATH='/World/G1_A'`；换场景或重启前调用 `cleanup_mid360_ros2()`。

| 话题 | 类型／坐标系 | 约定 |
| --- | --- | --- |
| `/mid360/points` | `PointCloud2`，`mid360_link` | 完整 10 Hz 点云；26 字节 XYZ/intensity/tag/line/timestamp |
| `/mid360/imu` | `Imu`，`mid360_link` | m/s²、rad/s；不提供绝对姿态 |
| `/tf` | `TFMessage` | 资产根坐标系 → `mid360_link` |
| `/clock` | `Clock` | 仿真时间；订阅端设置 `use_sim_time=true` |

PointCloud2 的 `timestamp` 为 float64 **绝对纳秒**，与 `livox_ros_driver2` 布局一致，不能当作秒。Petal 的 `line=emitter_id % 4` 只是兼容近似，官方方向表未提供物理激光 ID；Rotary 的 `line` 为 40 个合成通道编号。

机器人安装约定沿用 torso 平移 `[0.0002835, 0.00003, 0.41618]` m、滚转 180°。IMU 与雷达仍共点且同向（`T=0`、`R=I`）；这是已有仿真约定，不是 MID-360 内部真实 IMU 外参。独立雷达安装滚转为 0°。

## 重建与验收

```bash
# 普通 Python：文件哈希、轨迹及场景测试
make check

# 在 Sim 6.2 Python 环境中：更新全部 8 个传感器配置和清单
python scripts/migrate_assets_isaacsim62.py
python scripts/smoke_test_assets_isaacsim62.py
bash scripts/run_isaacsim62.sh scripts/validate_runtime_isaacsim62.py --all --ros2 --callbacks
```

需要从机器人传感器重新生成独立雷达时：

```bash
python scripts/build_mid360_standalone_assets.py \
  assets/common/Livox_MID360_CAD.usd \
  assets/petal_scan/standalone/Livox_MID360_Petal_Scan.usd \
  --sensor-source assets/petal_scan/g1_4010/Unitree_G1_4010_MID360_Petal_Scan.usd
python scripts/update_asset_metadata.py
```

Rotary 版替换成对应路径。传感器迁移直接沿用机器人 USD，不重新导入 URDF。可选 URDF 重建器需要配套 Isaac Lab；新版导入器的圆柱/胶囊转换可能与历史版本不同。

报告：[6.2 资产／场景检查](docs/validation/isaacsim62_structure.json)、[6.2 点云、IMU 与 ROS 验收](docs/validation/isaacsim62_runtime.json)、[5.1 分支复核](https://github.com/123tthh/Livox_MID360_IsaacSim/blob/isaac-sim-5.1/docs/validation/isaacsim51_20261106.json)。运行测试使用静态封闭房间和固定关节的机器人，不据此宣称运控、动态畸变、FAST-LIO 或 SLAM 性能。

2026-11-06 在本机 6.2.0.0 / RTX 5090 上，8 个资产全部通过：花瓣 40 状态及回绕、200 Hz IMU、244/244 帧点云和 4,880/4,880 条 IMU 的 ROS 收发。[逐项迁移比对](docs/validation/isaacsim62_migration.json)确认几何和机器人动力学配置一致；完整物体场景启动入口也通过：[Petal](docs/validation/isaacsim62_object_field_petal.json)、[Rotary](docs/validation/isaacsim62_object_field_rotary.json)。

`docs/validation/screenshots/` 保留原有 **5.1** 截图和文件名，不作为新 6.2 截图。[角度分布比较图](docs/validation/MID360_Petal_vs_Rotary_Angular_Pattern.png) 与场景版本无关，可用 `python scripts/generate_scan_pattern_comparison.py` 重建。

原始脚本采用 MIT 许可证。Livox 官方轨迹、Unitree G1、InstinctLab 和 CAD 的来源与保留许可证见 [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) 及 `assets/common/`。仓库提供资产和传感器读取／发布模块，不包含运控策略、建图系统、硬件设备发现或无限时长光机模型。
