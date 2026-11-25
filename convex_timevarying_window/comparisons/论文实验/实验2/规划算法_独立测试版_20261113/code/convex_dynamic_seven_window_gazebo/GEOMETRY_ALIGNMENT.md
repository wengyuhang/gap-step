# 安全几何口径

最终安全验收统一使用 Gazebo 的实体碰撞几何：官方 `x500/model.sdf` 合并的 `x500_base/model.sdf` 碰撞盒和旋翼碰撞盒，以及七扇门的 `*_sleeve.stl` 碰撞网格。窗口按 `course_spec.json` 的周期平移和 RPY 旋转。运行 `px4_kinematic_replay.py` 时，每个 4 ms 物理步直接写入 x500 状态；首个门框接触才释放模型。因此接触日志是这套最终几何口径的安全判定。

`x500_togt/experiment.py` 中的 `X500_FRAME_RADIUS=0.385944720 m` 保留，但只作为优化与候选筛选的外接圆代理。它包住四个旋翼碰撞盒在平面上的最坏投影，忽略机身真实形状和姿态方向，所以比实际 SDF 碰撞更保守。它的失败不能再写成 Gazebo 实体碰撞失败。

在这一统一口径下，2026-09-11 的直接回放结果为：

| 轨迹 | 参考时间 | 官方 x500 + sleeve 门框接触 |
|---|---:|---|
| TOGT x500 baseline | 24.660806190 s | 0 |
| Conditional Dual-Constraint CEM candidate 364 | 28.822503627 s | 0 |

对应结果为 `results/px4_kinematic_replay/20260911_045752/result.json` 和 `20260911_045907/result.json`。这两个运行不经过 PX4，不能用于控制器跟踪结论；此前 PX4 中 W2 的碰撞保留为闭环跟踪失败。
