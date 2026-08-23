# CWB-SC-DynaTOGT 算法说明

## 方法边界

本方法复用 SC-DynaTOGT 的动态窗口、SC 前向映射、degree-7 MINCO、四旋翼微分平坦性和
TOGT 代价，但不修改这些模块。算法仍优化 `x=[K,D]`，yaw 固定为零；整机用机体系长方体表示。

## 几何与区间

`body_model.CuboidBody` 固定生成 8 个符号位顶点和只改变一个符号位的 12 条边。
`gate_frame.frame_at` 从原 `state_at` 构造 `normal` 和 `normal_dot`。
`plane_section.gate_local_vertex_coordinates` 实现

\[
\xi^3=n^T(v-c).
\]

`find_planned_crossing_interval` 从 `t_i` 向两侧寻找无截面括区间并二分边界，只返回包含
`t_i` 的连通分量。`plane_section_from_vertices` 对 12 条机体边求交、处理接触/共面退化、
去重并按局部质心极角排序；`topology_key` 只由来源机体边编号组成。

## SC 安全验证

`sc_inverse.inverse_sc_map` 使用实二维 Newton、回溯线搜索和开圆盘投影。目标多边形外的点在
Newton 前由真实非凸多边形守卫拒绝。安全裕度唯一采用

\[
m(q)=r^2-z^Tz,\qquad z=\Psi^{-1}(q).
\]

`whole_body_safety.verify_whole_body_trajectory` 先做拓扑稳定时间分段，再对截面多边形每条边
建立 `(t, lambda)` 单元。每个单元采样时间端点、四分点、中点与边参数端点/中点，估计
`V_t`、`V_lambda` 和 `||J_Psi^-1||`，乘 `velocity_inflation` 后尝试接受；否则沿贡献更大的维度二分。
明确负裕度或 SC 定义域外点产生 `SafetyWitness`；数值失败、预算不足与明确不安全严格分开。

该上界来自数值采样而非严格区间包围，所以通过结果只能叫 `NUMERICALLY_VERIFIED`。

## 活动约束生成

`constraint_generation.WholeBodyConstrainedObjective` 根据 witness 的
`(segment, normalized_time, source_edges, lambda)` 在当前 `[K,D]` 下重建危险点。拓扑改变使来源边
失效时，本轮约束标记 stale，优化后由验证器重新找 witness。

窗口内点的安全罚为

\[
w[\max(\|\Psi^{-1}(q)\|^2-r^2,0)]^2.
\]

窗口外点从安全 SC 圆周辅助原像池选择最近图像点，使用等式残差平方提供恢复方向。
外层按“基础优化 → 完整验证 → 最危险 witness → 温启动优化”循环，并始终保留原 `[K,D]` 维度。

## 未完成项

- V2 Bernstein/区间算术严格证书；
- 连续 SC 导数的严格网格上界；
- 将 V1 活动罚的中心差分替换为计划中的 PyTorch float64 反向模式；
- 连续辅助变量 `[K,D,U]` 增广拉格朗日版本（当前为离散辅助原像池）。

因此论文或报告不得把当前结果写成 certified continuous safety。
