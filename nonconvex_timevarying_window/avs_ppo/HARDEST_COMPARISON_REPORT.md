# AVS-PPO 最难 comparisons 赛道转移实验

## 场景与机体

实验直接读取
`comparisons/sc_sip_fast_closed_loop/results/wide_scrambled_certified_final`
的 `wide_scrambled_fast_closed_loop_6` 场景。六个窗口的真实直线、圆弧、Bézier
和 B-spline 边界，以及平移、全 RPY 旋转和缩放运动均未简化。

无人机使用原比较实验的姿态长方体：

```text
half_extents = (0.26504, 0.26504, 0.05890) m
net_clearance = 0.015 m
```

每个动作都通过动作后全机距离、参考时相管、位置/速度可恢复管和靠近窗框时的
名义恢复动作检查。穿越事件使用移动窗口平面与姿态长方体的实际截面，截面所有顶点必须位于
真实非凸开口内。

## 结果

seed 17 的严格盾版运行完成六窗闭环：

- 成功率 `1/1`，几何安全违规 `0`；
- 完赛时间 `16.15 s`；
- 局部连续数值细化后的最小长方体—边界距离 `20.5539 mm`，比 `15 mm`
  要求多 `5.5539 mm`；
- 六次平面截面均在指定非凸开口内；
- 平均可行动作比例仅 `0.08764`，相当于 13 个动作中平均只有 `1.139`
  个可选。

因此这不是“PPO 在最难赛道上学会了更快飞行”的正结果。两次 PPO 更新前后都是
`100%` 完成，masked entropy 为零，说明严格安全盾在关键区域几乎完全接管了策略。
该结果证明当前 AVS 架构可安全重放此场景，但不证明它在该场景上有有效的学习自由度。

安全结论是实验模型下的高密度数值审计和局部优化结果，不是对 AVS 闭环轨迹的 Arb
连续域证书，也不包含真机感知、执行器和模型误差。

## 复现

```bash
python -m nonconvex_timevarying_window.avs_ppo.train_hardest \
  --config nonconvex_timevarying_window/avs_ppo/configs/hardest_comparison.yaml \
  --outdir nonconvex_timevarying_window/avs_ppo/results/hardest_comparison_strict_v5_seed17

python -m nonconvex_timevarying_window.avs_ppo.hardest_evaluate \
  --config nonconvex_timevarying_window/avs_ppo/configs/hardest_comparison.yaml \
  --checkpoint nonconvex_timevarying_window/avs_ppo/results/hardest_comparison_strict_v5_seed17/final.pt \
  --episodes 1 --dense-audit

python -m nonconvex_timevarying_window.avs_ppo.visualize_hardest \
  --config nonconvex_timevarying_window/avs_ppo/configs/hardest_comparison.yaml \
  --checkpoint nonconvex_timevarying_window/avs_ppo/results/hardest_comparison_strict_v5_seed17/final.pt \
  --learning-curve nonconvex_timevarying_window/avs_ppo/results/hardest_comparison_strict_v5_seed17/learning_curve.csv \
  --outdir nonconvex_timevarying_window/avs_ppo/results/hardest_comparison_strict_v5_seed17/visualization
```
