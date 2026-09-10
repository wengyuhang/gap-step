# Dual-Constraint CEM SC-DynaTOGT

该方法从原始 SC-DynaTOGT 的最终 `[K,D]` 直接启动完整协方差 CEM，不含相位模板、共同时间伸缩或任何按窗口形状编写的结构化前端。

每个候选同时计算两个非负软约束积分：

1. TOGT 复现代码的动力学积分：`smoothedL1`（`mu=0.01`）、每段按时长取 8–32 个区间、梯形积分；约束数值取发布复现包 `standard_planning.yaml` 的 `60 m/s`、`10 rad/s` 和单旋翼 `0.25–5 N`。
2. 带 15 mm 规划安全余量的球体与零厚度有限门框安全积分。局部边界距离用点到曲线稠密样本平方距离的 Gibbs 加权平滑最小值，残差为 `rho_plan^2-z^2-d_soft^2`，再使用同一 `smoothedL1` 和 8–32 区间梯形积分，其中 `rho_plan = r_body + 0.015 m`。最终碰撞验收单独使用真实无人机外接球 `r_body`，不把这 15 mm 规划余量算进碰撞。

搜索排序先在 `(J_dyn, J_safe)` 上做 Pareto 分层，同一层按 `J_aug = T + max(J_dyn-epsilon_dyn,0) + 100000 max(J_safe-epsilon_safe,0)` 排序。固定系数只补偿安全距离残差与 TOGT 推力/角速度残差的量纲数量级差异。Pareto 比较和同层目标使用整次实验不变的 `epsilon_dyn=epsilon_safe=1e-6` 作为数值分辨率平台，没有动态 epsilon；它不决定合格。中间合格仍严格要求原始 `J_dyn=0` 且原始 `J_safe=0`。合格与失败候选都会参与均值和完整协方差更新，合格点也不会退出协方差更新，并由飞行时间打破同层并列。

软积分只用于搜索。首次双零后固定再运行 5 轮。返回结果必须另行通过全程 1 ms 动力学密集检测，以及七个窗口全部真实外接球接触区间的 0.2 ms/0.05 ms 安全密集检测；双零候选按飞行时间升序验收，第一条硬检测通过后立即停止。

可微距离场的思路参考 CHOMP 的 signed-distance collision cost、GPMP2 的 signed-distance-field obstacle factors，以及 Madan 与 Levin 对离散几何平滑距离的构造。当前实现采用相关的指数核软最近点权重，并使用预处理后满足 1 mm 弦误差和 1 cm 最大弦长的曲线折线；所有直边也补采样至 1 cm。它是优化辅助场，不是原始曲线的连续安全证书。

- CHOMP: <https://www.cs.cmu.edu/~mzucker/icra09-chomp.pdf>
- GPMP2: <https://arxiv.org/abs/1707.07383>
- Smooth Distance Functions for Co-Dimensional Geometry: <https://www.dgp.toronto.edu/projects/smooth-distances/>

正式比较入口：

```bash
conda run -n wyh python -m nonconvex_timevarying_window.comparisons.seven_unique_dual_constraint_cem.experiment \
  --outdir nonconvex_timevarying_window/comparisons/seven_unique_dual_constraint_cem/results/formal_final_post5_sphere_only_20260909 \
  --population 64 --maximum-rounds 50 --hard-audit-limit 96 \
  --baseline-result-dir nonconvex_timevarying_window/comparisons/seven_unique_dual_constraint_cem/results/formal_20260909 \
  --candidate-jsonl nonconvex_timevarying_window/comparisons/seven_unique_dual_constraint_cem/results/formal_fixed_epsilon_20260909/candidates.jsonl
```

`--candidate-jsonl` 会逐轮校验并精确恢复冻结前缀的随机数、均值、完整协方差和精英记忆，只计算尚缺的后续轮次。入口在运行前后计算所依赖 Python 源文件的 SHA-256；若实验期间代码发生变化，运行会标记为无效并失败。

## 七异形闭合赛道结果

2026-09-09 的冻结候选复放从原 SC-DynaTOGT 解出发，CEM 每轮 64 个候选，在第 42 轮首次得到严格双零候选，并按协议完成第 43–47 轮后停止，共保留 3072 条随机候选。严格双零集合按飞行时间排序后，第 1 条硬验收候选 `id=2701` 通过，因此没有继续验收后续候选。

该轨迹飞行时间为 `24.709999069 s`，动力学软积分和含 15 mm 规划余量的安全软积分均为零；全程 1 ms 动力学检测与七窗口真实外接球检测均通过，最小真实球体余量为 `13.878932 mm`。完整三方法表、逐窗检测和冻结代码清单见[正式结果](../comparisons/seven_unique_dual_constraint_cem/results/formal_final_post5_sphere_only_20260909/REPORT.md)。这是名义模型的密集采样数值证据，不是连续域证书。
