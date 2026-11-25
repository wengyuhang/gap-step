# 条件式安全恢复 Dual-Constraint CEM：七窗口结果

分支：`joint`；名义 TOGT 安全软积分：`0.114643893981`。

| 方法 | 飞行时间 (s) | 完整动力学 | 完整安全 |
|---|---:|---|---|
| TOGT | 24.660803582 | 满足 | 不满足 |
| Conditional Dual-Constraint CEM | — | 无合格轨迹 | 无合格轨迹 |

## 阶段耗时

| 阶段 | 时间 (s) |
|---|---:|
| scene_and_objective_setup | 0.006449 |
| safety_geometry_setup | 0.000278 |
| nominal_togt_lbfgs | 0.536768 |
| conditional_joint_lbfgs | 260.483027 |
| cem_search | 2.212107 |
| baseline_final_audits | 8.645214 |
| method_final_audits | 0.000000 |
| total_wall | 272.308172 |

安全优化积分使用外接球半径加 100 mm 规划余量；完整安全验收只使用真实外接球半径。
动力学和安全最终验收采用最大 1 ms 步长的密集采样，是数值证据，不是连续域认证。
