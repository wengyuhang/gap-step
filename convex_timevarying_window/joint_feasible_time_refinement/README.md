# Joint Feasibility-Preserving Time Refinement

该方法是 `conditional_dual_constraint_cem/` 之后的独立时间优化方法，不修改或覆盖
原条件式安全恢复结果。输入必须是已经通过完整动力学与整机安全审计的
`[K,D]` 候选。

每轮先以联合目标

```text
flight time + native TOGT dynamic penalty + 100000 * whole-body safety penalty
```

做一次激进的解析梯度 L-BFGS。该局部解只用于提供缩短时间的方向，不直接作为
输出。随后在上一个已验收解与激进解之间对完整 `[K,D]` 做 homotopy 搜索，
候选必须依次通过：

1. 原生 TOGT 动力学软积分和整机安全软积分筛选；
2. 最大 1 ms 步长的发布版 C++ 四旋翼动力学审计；
3. 最大 1 ms 步长的真实外接球—动态有限门框安全审计。

只有飞行时间严格改善且两项完整审计都通过的候选才替换当前解。因此方法
联合处理时间、动力学和安全，但最终结论仍是密集采样数值证据，不是连续域证书或
全局时间最优性证明。

运行：

```bash
conda run --no-capture-output -n wyh python \
  -m convex_timevarying_window.joint_feasible_time_refinement.experiment \
  --outdir convex_timevarying_window/joint_feasible_time_refinement/results/new_run
```
