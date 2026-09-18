# TOGT vs Conditional Dual-Constraint CEM

这是凸时变窗口的冻结批量对比：每个赛道都是闭环七窗、精确圆/凸多边形开口、三维平移和 RPY 周期运动。
默认生成 `4 难度 × 6 实例 = 24` 条赛道。难度以冻结的开口尺度、窗口运动幅值/频率和独立的相位/周期扰动变化。每对比例使用完全相同的赛道、`[K,D]`、TOGT C++ 动力学和最大 1 ms 审计器。
输出包括：

- `per_instance.csv`：全部逐实例原始指标，含失败；
- `benchmark_overview.png`：联合可行率（Wilson 95% CI）、碰撞率、双方可行时的飞行时间、规划时间；
- `safety_dynamics.png`：最小安全净空、单旋翼推力、XY 机体角速度和倾角分布；
- `REPORT.md`和`summary.json`：组别汇总。

运行正式 24 场景批次：

```bash
conda run --no-capture-output -n wyh python -m \
  convex_timevarying_window.comparisons.conditional_cem_vs_togt.benchmark \
  --outdir convex_timevarying_window/comparisons/conditional_cem_vs_togt/results/formal_run
```

安全与动力学结论是密集采样数值证据，不是连续域认证。
