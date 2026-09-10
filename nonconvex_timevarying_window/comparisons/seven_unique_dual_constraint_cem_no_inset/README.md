# 无内缩 SC 双约束 CEM

该消融实验沿用七异形分散闭环赛道和 Dual-Constraint CEM，只取消 SC 建图前的几何内缩：

- SC 映射拟合 Chang 重采样后的物理开口；严格共线直线段只保留端点，不改变窗口几何，曲线采样点保留。
- `rho = r_body + 15 mm` 不变，但不用于构造 SC 多边形，只用于安全软积分。
- 严格双零仍指原始动力学软积分和安全软积分都精确为零。
- 最终动力学检测覆盖全程，最终碰撞检测只使用真实无人机外接球。
- 首次严格双零后继续 5 轮；候选按飞行时间升序逐条硬验收，首个成功立即停止。

这种分离允许 SC 参数域保持原物理开口的单连通拓扑，不要求内缩结果仍是单个无洞多边形。安全可通行性由独立软约束和最终硬检测决定。

正式入口：

```bash
conda run -n wyh python -m nonconvex_timevarying_window.comparisons.seven_unique_dual_constraint_cem_no_inset.experiment \
  --outdir nonconvex_timevarying_window/comparisons/seven_unique_dual_constraint_cem_no_inset/results/formal_20260910 \
  --baseline-result nonconvex_timevarying_window/comparisons/seven_unique_dual_constraint_cem/results/formal_final_post5_sphere_only_20260909/result.json \
  --population 64 --maximum-rounds 50 --hard-audit-limit 96
```

运行前后会核对所依赖 Python 源文件的 SHA-256；正式运行期间不得修改代码。



