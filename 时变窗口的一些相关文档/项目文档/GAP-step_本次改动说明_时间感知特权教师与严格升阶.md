# GAP-Step 本次改动说明：时间感知特权教师与严格升阶

## 背景

上一轮 adaptive full run 在 C3 达到 hard max 后停止，最终 checkpoint 的 deterministic evaluation 连 C1/C2 也明显退化。主要判断是：teacher 只看当前 ray 几何，缺少动态窗口未来时机信息；同时 PPO 后期容易把不稳定策略训得过于确定。

本次改动目标是让训练流程真正按能力升阶，并让 teacher 直接获得低维窗口时序特权信息。仍然只训练 PPO privileged teacher，不引入学生策略、RNN、视觉输入、BC 或 SITT。

## 本次主要改动

1. 特权观测从 39D 扩展为 161D。
   - 保留原 39D 前缀：robot state、relative goal、32 rays。
   - 追加全局时间相位 `sin/cos`。
   - 追加最多 10 个窗口的固定长度摘要，每个窗口 12 维。
   - 窗口摘要包含 relative center、distance、normal、current safe、width clearance、angle error、time_to_next_safe、time_to_close 和 wait cost。
2. PPO 增加探索下限。
   - `TeacherActorCritic` 新增 `min_log_std` / `max_log_std`。
   - 默认 `min_log_std = -1.0`，防止 std 塌缩到约 0.1。
   - `train_metrics.csv` 增加 `log_std_mean`、`std_mean`。
3. adaptive curriculum 升阶改为双条件。
   - rollout 最近成功率 >= 70%。
   - deterministic train-validation 成功率 >= 60%。
   - hard max 仍然停止，不保存 best/intermediate checkpoint。
4. full 配置安全阀放宽。
   - soft max: 5M steps。
   - hard max: 10M steps。
5. checkpoint/evaluate/visualize 链路适配新观测维度。
   - checkpoint 保存 `obs_dim`、`min_log_std`、`max_log_std`。
   - `evaluate.py` 根据 checkpoint 自动加载对应模型维度。

## 配置变化

新增或更新的关键配置：

```yaml
promotion_eval_success_rate: 0.60
promotion_eval_episodes: 50
promotion_eval_interval_updates: 10
soft_max_steps_per_stage: 5000000
hard_max_steps_per_stage: 10000000
min_log_std: -1.0
max_log_std: 2.0
env:
  max_gate_obs: 10
```

smoke 配置使用更小的 `promotion_eval_episodes`，只用于快速验证流程。

## 诊断字段

`train_metrics.csv` 新增：

```text
promotion_eval_success_rate
promotion_eval_episodes
obs_dim
log_std_mean
std_mean
```

这些字段用于判断升阶是否同时满足 stochastic rollout 和 deterministic evaluation，以及策略探索是否再次塌缩。

## 已完成验证

已完成快速链路验证：

```bash
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
python -m gap_step.evaluate --checkpoint checkpoints/teacher_final.pt --episodes 20 --stages C1,C2,C3,C4,C5
```

结果：

- `pytest -q`：25 passed
- smoke 训练成功写出 `obs_dim=161` 的 checkpoint 和 train metrics
- stage-wise evaluation 可以正常读取新 checkpoint 并完成 C1-C5 评估

注意：smoke 会覆盖 `checkpoints/teacher_final.pt` 和 `results/*.csv`，只验证代码链路，不代表正式训练质量。

## 下一步

重新运行 full training：

```bash
python -m gap_step.train --config gap_step/configs/train_teacher_full.yaml
```

重点观察：

1. C3 是否仍出现 success 先升后塌。
2. `std_mean` 是否长期贴近下限。
3. deterministic promotion eval 是否阻止过早升阶。
4. 是否能继续推进到 C4/C5。
