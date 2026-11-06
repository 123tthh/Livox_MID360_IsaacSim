# Unitree G1 5010 Mode15 + MID-360 Rotary Scan

目标版本：**Isaac Sim 6.2 (`main`)**。运行入口见仓库根 README。

这是明确标注的官方 `g1_29dof_mode_15` 派生资产，不是 Mode13 的改名副本。

- 资产：`Unitree_G1_5010_Mode15_MID360_Rotary_Scan.usd`
- Machine mode：15
- 髋部减速比：22.5 / 22.5
- 手腕：5010；腰部未锁定
- 关节：29 个 RevoluteJoint
- 雷达：40 线普通旋转近似，200,000 points/s，10 Hz
- IMU：与 `mid360_link` 同位同向，200 Hz
- Default Prim：`/g1_29dof_mode_15_5010_mid360`
- 外部 USD 引用：无
- 内置策略、运控、建图 Action Graph：无

官方来源提交、URDF SHA-256、模型参数和资产 SHA-256 见 `manifest.json`。
