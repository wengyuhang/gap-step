# 条件式安全恢复 Dual-Constraint CEM：七窗口结果

分支：`joint`；名义 TOGT 安全软积分：`0.531650780152`。

| 方法 | 飞行时间 (s) | 完整动力学 | 完整安全 |
|---|---:|---|---|
| TOGT | 24.660803582 | 满足 | 不满足 |
| Conditional Dual-Constraint CEM | — | 无合格轨迹 | 无合格轨迹 |

## 阶段耗时

| 阶段 | 时间 (s) |
|---|---:|
| scene_and_objective_setup | 0.006234 |
| safety_geometry_setup | 0.000279 |
| nominal_togt_lbfgs | 0.600215 |
| conditional_joint_lbfgs | 70.410803 |
| cem_search | 378.510735 |
| baseline_final_audits | 8.599266 |
| method_final_audits | 0.000000 |
| total_wall | 458.537950 |

安全优化积分使用外接球半径加 300 mm 规划余量；完整安全验收只使用真实外接球半径。
动力学和安全最终验收采用最大 1 ms 步长的密集采样，是数值证据，不是连续域认证。
