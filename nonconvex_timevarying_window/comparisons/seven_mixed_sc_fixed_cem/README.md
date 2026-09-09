# 七窗口混合赛道：Fixed-WP、SC-DynaTOGT 与 Feasibility-Guided CEM

固定中心和平面的开放赛道依次使用均衡 U、利马松、五角星、五瓣波浪、L、直线–三次 Bézier 和均衡 U。各窗口只绕法向自旋。第一扇高速均衡 U 用于检验 Fixed-WP 的整机碰撞。

```bash
conda run -n wyh python -m nonconvex_timevarying_window.comparisons.seven_mixed_sc_fixed_cem.experiment \
  --outdir nonconvex_timevarying_window/comparisons/seven_mixed_sc_fixed_cem/results/baselines

conda run -n wyh python -m nonconvex_timevarying_window.comparisons.seven_mixed_sc_fixed_cem.cem \
  --baseline-result nonconvex_timevarying_window/comparisons/seven_mixed_sc_fixed_cem/results/baselines/result.json \
  --outdir nonconvex_timevarying_window/comparisons/seven_mixed_sc_fixed_cem/results/three_way
```

三个方法使用同一场景、动力学限制和碰撞口径。失败轨迹保留但不进入合格时间排名。球体、动力学与真实姿态长方体检查均为密集采样证据，不是连续域证书。
