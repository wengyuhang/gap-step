# 验证记录

日期：2026-08-18。

执行：

```bash
pytest -q nonconvex_timevarying_window/exact_area_sc_dynatogt/tests
python -m compileall -q nonconvex_timevarying_window/exact_area_sc_dynatogt
python -m nonconvex_timevarying_window.exact_area_sc_dynatogt.experiments \
  --suite B \
  --outdir nonconvex_timevarying_window/exact_area_sc_dynatogt/results/experiment_b
```

结果：8 个测试全部通过，编译检查通过，实验 B 生成 PNG、GIF、CSV 和 JSON。

反例关键数值：

| State | Time [s] | World margin [m] | Center-plane distance [m] | E [m²] | Collision |
|---|---:|---:|---:|---:|---:|
| Old nominal center crossing | 8.800000 | 0.315 | 0 | 0 | False |
| Old first detected dynamic collision | 8.073768 | 0.315 | 0.307969 | 1.952e-3 | True |
| Ours at the same collision time | 8.073768 | 0.315 | 0.265769 | 0 | False |

Old 的名义穿越点世界边界距离为 0.316 m，满足固定 0.315 m 裕度，且 (E(t_i)=0)。机体为 `0.530 × 0.530 × 0.118 m` 的正方形底面扁平长方体，机体 x 轴沿轨迹速度指向机头。动态窗口运动使机头/前缘在质心到达平面前先与门框接触。Old 在首次接触时终止；Ours 在同一时刻安全并返回共同规划终点。这些时间只描述动画执行，不是优化飞行时间，也没有据此声称 Ours 更快。
