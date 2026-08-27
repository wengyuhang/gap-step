# AVS-PPO：安全动作掩码的时变非凸窗口竞速

AVS-PPO（Action-masked Viability-Shielded PPO）是在本目录中独立实现的安全强化学习方法。它面向三维平移质点模型：球形无人机以连续时间恒加速度离散控制，按 `G1 -> G2 -> G3` 顺序穿过平移、面内旋转和各向异性缩放的无洞非凸窗口，并最小化完赛时间。

核心分工是：PPO 只负责在安全动作支持集内学得更快；预测安全盾牌负责训练和部署时的硬几何约束。安全不是负奖励的同义词。

## 已实现范围

- 状态为三维位置、速度、时间、当前顺序索引和全部窗口的特权动态状态；
- 控制是 13 个参数化加速度指令，包括一个门前制动备份动作；
- 窗口包含星形、U 形和波浪形简单非凸多边形，不能用凸包替代；
- 每个候选动作都接一段备份制动 rollout，只有整段保持可恢复才进入 PPO 的 categorical support；
- 恒加速度轨迹与门平面的交时刻通过二次方程求解，交点在该连续时刻由真实动态多边形和 `0.16 m + 0.035 m` 裕度验证；
- PPO 用显式旧策略采样，更新新策略，掩码与动作一起进入 clipped objective；
- 提供独立种子评估、学习曲线、checkpoint 和回归测试。

这里的“安全”严格限定为本仿真模型内：已知确定性动力学、零厚度门平面、球形 clearance 模型和配置中的窗口运动族。它不是对真实四旋翼、模型误差、感知误差或任意连续时间障碍的硬件级证明。

## 快速复现

```bash
pytest -q nonconvex_timevarying_window/avs_ppo/tests

python -m nonconvex_timevarying_window.avs_ppo.train \
  --config nonconvex_timevarying_window/avs_ppo/configs/smoke.yaml \
  --outdir nonconvex_timevarying_window/avs_ppo/results/smoke

python -m nonconvex_timevarying_window.avs_ppo.train \
  --config nonconvex_timevarying_window/avs_ppo/configs/train.yaml \
  --outdir nonconvex_timevarying_window/avs_ppo/results/formal_seed7_final

python -m nonconvex_timevarying_window.avs_ppo.evaluate \
  --config nonconvex_timevarying_window/avs_ppo/configs/train.yaml \
  --checkpoint nonconvex_timevarying_window/avs_ppo/results/formal_seed7_final/best.pt \
  --episodes 200 --seed 50000 \
  --output nonconvex_timevarying_window/avs_ppo/results/formal_seed7_final/audit_200.json
```

算法、文献选择和实验结论分别见 [ALGORITHM.md](ALGORITHM.md)、[LITERATURE_REVIEW.md](LITERATURE_REVIEW.md) 和 [TEST_RESULTS.md](TEST_RESULTS.md)。

## comparisons 最难六窗赛道

`hardest_comparison.py` 使用 `wide_scrambled_fast_closed_loop_6` 的原始六窗几何与动力学时间线，
并将机体替换为比较实验的姿态长方体。详细设置、复现命令和负责任结论见
[HARDEST_COMPARISON_REPORT.md](HARDEST_COMPARISON_REPORT.md)。

本次严格盾版在数值审计下完成六窗且无几何违规，但 13 个动作中平均仅有
`1.139` 个可行，PPO 更新前后的确定性策略表现相同。因此它是“安全盾成功、学习自由度几乎消失”
的压力测试，不应写成 PPO 在该场景上获得了新的竞速能力。
