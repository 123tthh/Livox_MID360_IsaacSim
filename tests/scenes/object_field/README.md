# Isaac Sim 6.2 自建物体场验证

`MID360_Object_Field.usda` 是完全自建且自包含的 12 m × 12 m 地面场景。机器人周围
5 m 内放置 Box、Sphere、Cylinder、Cone、Capsule、ThinWall、Pyramid、Wedge、Arch
九类物体，并在 4.75 m 处增加一块 5.5 m × 3.2 m 的 `ProjectionWall`。十个目标的
包围盒最低点均为 z=0。

当前两个可运行 Stage 明确使用 G1 5010 Mode13：

- `MID360_G1_5010_Petal_Object_Field.usda`
- `MID360_G1_5010_Rotary_Object_Field.usda`

机器人 29 个旋转关节锁为 0，`torso_link` 通过 World FixedJoint 固定，双脚位于地面。
Stage 不包含策略、运控、定位或建图图节点。

分别在两个终端运行：

```bash
./scripts/run_object_field_validation.sh petal
./scripts/run_rviz2_mid360.sh petal
```

Rotary 版将两条命令的 `petal` 都改为 `rotary`。RViz2 Fixed Frame 必须是 `G1`；
传感器物理安装滚转 180°，若错误地用 `mid360_link` 作 Fixed Frame，地面会显示在上方。

- Petal：`config/mid360_object_field_petal_4s.rviz`，累积 4.2 s；
- Rotary：`config/mid360_object_field_rotary_0p1s.rviz`，累积 0.1 s。

这些窗口只影响 RViz 显示，不改变 `/mid360/points` 发布数据。
