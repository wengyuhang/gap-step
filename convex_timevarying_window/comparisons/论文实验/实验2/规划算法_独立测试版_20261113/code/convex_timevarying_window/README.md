# Convex Time-Varying Window

该问题族使用七个周期三维平移和 RPY 旋转的凸窗口，起点等于终点。

- [`continuation_racing/`](continuation_racing/README.md)：两窗加可停车末段联合优化，检查整个局部可执行计划，自适应少量检查点、连续参考和统一平台预览。2026-09-19冻结16条件均完赛且零接触，15次严格通过、1次缺少真值证据；原3个失败条件通过，最大完整周期90.614 ms、最小采样净空0.022683 m。新布局/独立补测见方法目录；不宣称连续硬安全。

- [`togt/`](togt/README.md)：发布版 TOGT standard 目标、C++ 手写动力学梯度和 MINCO 伴随回传的名义基线及历史结果。
- [`conditional_dual_constraint_cem/`](conditional_dual_constraint_cem/README.md)：先求名义 TOGT，再按安全违反量条件激活手写安全梯度和 Dual-Constraint CEM。
- [`joint_feasible_time_refinement/`](joint_feasible_time_refinement/README.md)：从已通过完整审计的恢复解出发，联合优化飞行时间、动力学和整机安全，仅接受更短且重新通过审计的候选。
- [`comparisons/conditional_cem_vs_togt/`](comparisons/conditional_cem_vs_togt/README.md)：冻结的多难度凸时变窗口基准，对比 TOGT 与 Conditional Dual-Constraint CEM 的联合可行率、碰撞率、飞行时间、安全净空、动力学峰值和规划时间。
- [`receding_gate_racing/`](receding_gate_racing/README.md)：统一三窗口在线时空优化与 PX4 执行；不读离线轨迹、无逐门特例。含房间约束的 v10 原赛道 33.640 s，相位/布局变体 34.416/32.356 s 均通过；最长规划周期 77.415 ms。中心对照更快及历史越界的负结果见方法报告。

公用的凸窗口映射与周期位姿定义保留在 [`geometry.py`](geometry.py)，问题口径见
[`PROBLEM_DEFINITION.md`](PROBLEM_DEFINITION.md)。根目录的 `experiment.py`、`native_backend.py`
和 `native_objective.py` 仅作旧导入路径的兼容入口。
