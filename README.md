# Livox MID-360 for Isaac Sim 6.2

**English** | [简体中文](README_CN.md)

**`main` targets Isaac Sim 6.2.** The complete 5.1 assets and runtime are maintained on [`isaac-sim-5.1`](https://github.com/123tthh/Livox_MID360_IsaacSim/tree/isaac-sim-5.1):

```bash
# Isaac Sim 6.2
git clone https://github.com/123tthh/Livox_MID360_IsaacSim.git
# Isaac Sim 5.1
git clone --branch isaac-sim-5.1 https://github.com/123tthh/Livox_MID360_IsaacSim.git
```

This repository includes eight ready-to-use USD assets: standalone MID-360 and Unitree G1 4010 / 5010 Mode13 / 5010 Mode15, each with Petal and Rotary profiles. Binary USDs, CAD, the official trajectory, and portable test scenes are tracked in Git; cloning downloads the assets directly.

## Asset matrix

| Profile | Standalone | G1 4010 | G1 5010 Mode13 | G1 5010 Mode15 |
| --- | --- | --- | --- | --- |
| Petal | `assets/petal_scan/standalone/Livox_MID360_Petal_Scan.usd` | `assets/petal_scan/g1_4010/Unitree_G1_4010_MID360_Petal_Scan.usd` | `assets/petal_scan/g1_5010_mode_13/Unitree_G1_5010_Mode13_MID360_Petal_Scan.usd` | `assets/petal_scan/g1_5010_mode_15/Unitree_G1_5010_Mode15_MID360_Petal_Scan.usd` |
| Rotary approximation | `assets/rotary_scan/standalone/Livox_MID360_Rotary_Scan.usd` | `assets/rotary_scan/g1_4010/Unitree_G1_4010_MID360_Rotary_Scan.usd` | `assets/rotary_scan/g1_5010_mode_13/Unitree_G1_5010_Mode13_MID360_Rotary_Scan.usd` | `assets/rotary_scan/g1_5010_mode_15/Unitree_G1_5010_Mode15_MID360_Rotary_Scan.usd` |

The six robot assets are flattened, single-file robot USDs. Standalone assets reference `assets/common/Livox_MID360_CAD.usd` relatively; preserve the directory structure. Each asset has a `manifest.json`, and `assets/catalog.json` records file hashes and target version.

Mode13 and Mode15 remain distinct official configurations: hip reductions 14.3/22.5 and 22.5/22.5 respectively, 5010 wrist motors, unlocked waist, and 29 revolute joints. This migration preserves their existing geometry, joints and mounting transforms.

## Native 6.2 scanning

| Parameter | Petal | Rotary approximation |
| --- | --- | --- |
| `scanType` | `SOLID_STATE` | `ROTARY` |
| Native emitter states | 40 | 1 |
| Emitters per state | 20,000 | 40 |
| `patternFiringRateHz` | 10 | 5,000 |
| Full scan rate | 10 Hz | 10 Hz |
| Nominal emitted rays | 200,000/s | 200,000/s |
| Pattern | Official 800,000 ordered Livox directions; four-second replay | Fixed 40 elevation channels |
| Output | Cartesian sensor-frame coordinates, accumulated full scans | Same |

6.2 stores all 40 Petal states directly on `OmniLidar`. **No Python angle-array swapping driver is needed.** RTX azimuth signs are converted to preserve the official Livox XYZ convention. The old 5.1 compressed Scope and swap scripts are confined to the 5.1 branch.

Both profiles use ideal single returns, 0.1–70 m clipping, and no calibrated angle/range noise or reflectivity detection model. The emitted ray count is not a guarantee of 200,000 valid returns/s. Petal's four-second table repeats; Rotary is a conventional baseline, not the physical MID-360 pattern.

## Run the fixed G1 object field

Use a fresh terminal with a Python environment containing **Isaac Sim 6.2**. The launcher selects its matching built-in ROS 2 Jazzy libraries; do not source a system ROS workspace in this terminal. No installation or EULA acceptance is performed by the scripts.

```bash
# Optional if the active python3 is not your Sim 6.2 interpreter:
export ISAAC_SIM_PYTHON=/path/to/your/6.2/environment/bin/python
export ROS_DOMAIN_ID=76
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
bash scripts/run_object_field_validation.sh petal

# Rotary profile, or a bounded headless run:
bash scripts/run_object_field_validation.sh rotary
bash scripts/run_object_field_validation.sh petal 10 --headless
```

In a separate system ROS 2 Jazzy terminal, use the same domain:

```bash
export ROS_DOMAIN_ID=76
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
bash scripts/run_rviz2_mid360.sh petal
```

RViz uses upright `G1` as its Fixed Frame. Petal display accumulation is 4.2 s; Rotary uses one 0.1 s revolution. The self-built scene fixes both feet on the ground with a world anchor and locks 29 joints; ten ground-contacting object groups lie within 5 m. No locomotion policy or SLAM stack is loaded.

## Python and Script Editor

Import `scripts/mid360_runtime_isaacsim62.py` **after** `SimulationApp` starts. Create a rendering product before playing; use physics time steps of 0.005 s for the 200 Hz IMU and enable RTX motion BVH as in the supplied launchers.

If the parent scene has an empty time range, the reader supplies a one-hour range in the session layer to avoid Kit's single-frame playback loop. Set a suitable scene range for longer runs.

```python
from mid360_runtime_isaacsim62 import MID360Runtime
sensor = MID360Runtime(stage, root_path='/World/G1')  # or your standalone root
# After sim.step(render=True):
cloud = sensor.read_cloud()                # new 10 Hz frame, or None
imu = sensor.read_imu(sim.current_time)     # 200 Hz at physics_dt=0.005
# Finish with sensor.close().
```

Clouds include `xyz`, `intensity`, `timestamp_ns`, `time_offset_ns`, `emitter_id` and `state_id`. Per-point acquisition time is `timestamp_ns + time_offset_ns`. Petal offsets retain the 5 μs firing slots even after invalid returns are removed. Raw `tickStates` in the tested 6.2 build stays zero; the reader establishes phase from returned XYZ and advances it using acquisition timestamps. Runtime tests independently search all 40 official states.

For Script Editor, enable `isaacsim.ros2.bridge` in a correctly started 6.2 session, open/reference an asset, and run the whole `scripts/publish_mid360_ros2_isaacsim62.py` file. It starts post-physics IMU and post-render cloud callbacks. Set `MID360_ROBOT_ROOT_PATH='/World/G1_A'` first when multiple assets are present. Call `cleanup_mid360_ros2()` before replacing the scene or restarting the publisher.

| Topic | Message / frame | Contract |
| --- | --- | --- |
| `/mid360/points` | `sensor_msgs/PointCloud2`, `mid360_link` | Full 10 Hz scans; packed 26-byte XYZ/intensity/tag/line/timestamp |
| `/mid360/imu` | `sensor_msgs/Imu`, `mid360_link` | m/s² and rad/s; no absolute orientation |
| `/tf` | `tf2_msgs/TFMessage` | Asset root frame → `mid360_link` |
| `/clock` | `rosgraph_msgs/Clock` | Simulation time; consumers should use `use_sim_time=true` |

`timestamp` in PointCloud2 is float64 **absolute nanoseconds**, matching `livox_ros_driver2`; it is not seconds. `line=emitter_id % 4` for Petal is a synthetic compatibility field because the official table lacks physical laser IDs. Rotary exposes its 40 synthetic channel indices. Do not interpret either as calibrated hardware channels.

Robot mounting is preserved at torso translation `[0.0002835, 0.00003, 0.41618]` m with 180° roll. The bundled IMU remains colocated and aligned with LiDAR (`T=0`, `R=I`), an existing simulation mounting contract rather than the physical sensor's internal IMU offset. Standalone mounting roll is 0°.

## Rebuilding and validation

Run offline checks with:

```bash
make check
```

Using the 6.2 Python interpreter, refresh all eight native sensor configurations and manifests:

```bash
python scripts/migrate_assets_isaacsim62.py
python scripts/smoke_test_assets_isaacsim62.py
bash scripts/run_isaacsim62.sh scripts/validate_runtime_isaacsim62.py --all --ros2 --callbacks
```

Rebuild a standalone wrapper from a robot sensor when needed:

```bash
python scripts/build_mid360_standalone_assets.py \
  assets/common/Livox_MID360_CAD.usd \
  assets/petal_scan/standalone/Livox_MID360_Petal_Scan.usd \
  --sensor-source assets/petal_scan/g1_4010/Unitree_G1_4010_MID360_Petal_Scan.usd
python scripts/update_asset_metadata.py
```

Use the corresponding Rotary paths for that profile. Native configuration migration preserves robot geometry rather than reimporting robot URDFs. The optional URDF source builder requires a compatible Isaac Lab installation; its legacy cylinder/capsule conversion may differ under the newer importer.

Reports: [6.2 USD/scene checks](docs/validation/isaacsim62_structure.json), [6.2 runtime and ROS checks](docs/validation/isaacsim62_runtime.json), and [5.1 branch verification](https://github.com/123tthh/Livox_MID360_IsaacSim/blob/isaac-sim-5.1/docs/validation/isaacsim51_20261106.json). Runtime validation uses a static closed room and anchored robots; it does not establish locomotion, dynamic distortion, FAST-LIO, or SLAM performance.

Local 6.2.0.0 / RTX 5090 validation on 2026-11-06 passed all eight assets, all 40 Petal states and wraparound, 200 Hz IMU, and 244/244 clouds plus 4,880/4,880 IMU ROS messages. [Exact migration comparison](docs/validation/isaacsim62_migration.json) preserves geometry and robot physics. Both complete object-field launchers also passed: [Petal](docs/validation/isaacsim62_object_field_petal.json), [Rotary](docs/validation/isaacsim62_object_field_rotary.json).

The historical screenshots under `docs/validation/screenshots/` were captured in **5.1** and retain their original names. They are not new 6.2 screenshots. The version-independent [angular pattern figure](docs/validation/MID360_Petal_vs_Rotary_Angular_Pattern.png) can be regenerated with `python scripts/generate_scan_pattern_comparison.py`.

## Sources and licensing

Original scripts use the root MIT license. Official trajectory, Unitree G1, InstinctLab resources and CAD provenance are recorded in [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) and `assets/common/`. Preserve their licenses when redistributing. The repository contains sensor and robot assets, not a locomotion policy, mapping implementation, hardware discovery stack, or infinite-duration optical model.
