# 曲线自旋三窗口：SC-DynaTOGT vs Fixed-WP

这是一条新设计的开放赛道，依次穿越利马松、五瓣波浪曲线和直线–三次 Bézier 混合边界。窗口中心和平面固定，窗口只绕自身法向匀速旋转，厚度为零。

对比使用同一条四段 degree-7 MINCO 轨迹、同一原始 SC-DynaTOGT 目标、动力学参数和 L-BFGS 设置。Fixed-WP 将三个穿越点固定为各安全内缩区的最大内切圆中心，只优化四个时间变量；SC-DynaTOGT 从 Fixed-WP 最终解精确热启动，再开放三个二维 SC 穿越点。

```bash
conda run -n wyh python -m nonconvex_timevarying_window.comparisons.curved_rotating_sc_fixed_wp.experiment \
  --outdir nonconvex_timevarying_window/comparisons/curved_rotating_sc_fixed_wp/results/baselines

conda run -n wyh python -m nonconvex_timevarying_window.comparisons.curved_rotating_sc_fixed_wp.three_way \
  --baseline-result nonconvex_timevarying_window/comparisons/curved_rotating_sc_fixed_wp/results/baselines/result.json \
  --outdir nonconvex_timevarying_window/comparisons/curved_rotating_sc_fixed_wp/results/three_way
```

`three_way` 在两个原始基线上增加 Feasibility-Guided CEM 新算法。新算法先做共同时间伸缩前端，再用现有全协方差 CEM 联合搜索原生 K/D。中间筛查对每扇窗只检查球体接触平面的穿越区间，并检查全程采样动力学。最终候选再做真实姿态长方体审计。只有全部硬要求都通过的轨迹才进入时间排名。全部结论都是名义模型密集采样证据，不是连续域证书。
