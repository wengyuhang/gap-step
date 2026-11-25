# 条件式安全恢复 Dual-Constraint CEM：七窗口结果

分支：`joint`；名义 TOGT 安全软积分：`0.0408022930684`。

| 方法 | 飞行时间 (s) | 完整动力学 | 完整安全 |
|---|---:|---|---|
| TOGT | 24.660804761 | 满足 | 不满足 |
| Conditional Dual-Constraint CEM | 28.822503627 | 满足 | 满足 |

## 阶段耗时

| 阶段 | 时间 (s) |
|---|---:|
| scene_and_objective_setup | 0.001569 |
| safety_geometry_setup | 0.000235 |
| nominal_togt_lbfgs | 0.056256 |
| conditional_joint_lbfgs | 72.134656 |
| cem_search | 78.391326 |
| baseline_final_audits | 8.522787 |
| method_final_audits | 10.495146 |
| total_wall | 170.353639 |

安全优化积分使用外接球半径加 30 mm 规划余量；完整安全验收只使用真实外接球半径。
动力学和安全最终验收采用最大 1 ms 步长的密集采样，是数值证据，不是连续域认证。
