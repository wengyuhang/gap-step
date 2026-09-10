# Convex Time-Varying Window

该问题族使用七个周期三维平移和 RPY 旋转的凸窗口，起点等于终点。

- [`togt/`](togt/README.md)：发布版 TOGT standard 目标、C++ 手写动力学梯度和 MINCO 伴随回传的名义基线及历史结果。
- [`conditional_dual_constraint_cem/`](conditional_dual_constraint_cem/README.md)：先求名义 TOGT，再按安全违反量条件激活手写安全梯度和 Dual-Constraint CEM。

公用的凸窗口映射与周期位姿定义保留在 [`geometry.py`](geometry.py)，问题口径见
[`PROBLEM_DEFINITION.md`](PROBLEM_DEFINITION.md)。根目录的 `experiment.py`、`native_backend.py`
和 `native_objective.py` 仅作旧导入路径的兼容入口。
