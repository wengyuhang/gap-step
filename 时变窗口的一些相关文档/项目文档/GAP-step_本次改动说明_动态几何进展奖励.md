# GAP-Step 本次改动说明：动态几何进展奖励

## 背景

此前 PPO privileged teacher 在稀疏奖励下训练效果不稳定，容易出现碰撞或超时，成功信号不足。由于环境是连续二维几何环境，进展奖励不能依赖 grid cell 或 `open_edges` 作为状态/路径依据。

## 本次主要改动

1. 在 `ContinuousMazeEnv` 中加入可选的连续几何动态进展奖励。
2. 保留 strict v4.6 稀疏奖励为环境默认行为。
3. 在 teacher smoke/full 训练配置中启用 `progress_mode: dynamic_geometry` 和 `reward_progress: 2.0`。
4. 进展势函数使用连续几何 roadmap 估计当前位置到目标的剩余时间。
5. 时变窗口 crossing cost 纳入未来等待代价：当前不可通过但未来可安全通过时，势函数仍给出有限等待代价。
6. 不改变 observation 维度、PPO 模型结构、碰撞规则、成功判定和课程生成逻辑。
7. 文档中明确：reward potential 不使用 cell；full 训练的 C1-C5 阶段切换由 `train.py` 根据 `global_steps` 和 `steps_per_stage` 控制。

## 实现要点

动态进展奖励形式：

```text
progress_reward = reward_progress * (prev_remaining_time - current_remaining_time)
```

每步 reward 组合：

```text
- time_penalty
- action_penalty
+ progress_reward
+ goal_reward if success
+ collision_penalty if collision
+ timeout_penalty if truncated
```

连续几何 roadmap 由以下节点/边组成：

- 目标点
- 时变窗口两侧 approach point
- 膨胀障碍矩形周围的可行转角点
- 不穿过膨胀障碍的连续线段 visibility edge
- 只通过窗口 approach point 连接的 gate crossing edge

窗口边代价包含：

```text
wait_until_future_safe(arrival_time) + crossing_length / max_speed
```

如果 lookahead 范围内没有安全时刻，则给大代价而不是直接删除路径，避免“必须过窗”场景下 reward 信号完全断掉。

## 已完成验证

已完成：

- `pytest -q`：15 passed
- full teacher 训练：完成 C1-C5，生成 `checkpoints/teacher_final.pt`
- evaluation：完成 ID、OOD-size、OOD-dynamics 三组评估，生成 `results/eval_metrics.csv`

评估结果：

| split | success rate | collision rate | timeout rate |
|---|---:|---:|---:|
| `id_test` | 5.0% | 57.5% | 37.5% |
| `ood_size_test` | 5.5% | 44.0% | 50.5% |
| `ood_dynamics_test` | 5.0% | 17.5% | 77.5% |

阶段训练聚合显示：

| stage | 后段 success | 后段 collision | 后段 timeout |
|---|---:|---:|---:|
| C1 | 60.0% | 18.4% | 21.5% |
| C2 | 20.8% | 42.7% | 36.5% |
| C3 | 13.4% | 43.9% | 42.7% |
| C4 | 14.9% | 36.2% | 48.8% |
| C5 | 8.0% | 55.6% | 36.4% |

## 当前结论

本次改动证明了动态几何进展奖励可以端到端运行，并且 C1 能明显学到一定导航能力。但完整 teacher 仍未训练成功，C2-C5 成功率明显下降，最终评估成功率约 5%。

因此，当前问题不是“环境完全不可学”，而是 reward、PPO 动作概率、课程推进和动态窗口难度之间仍未调顺。

## 已发现的主要风险点

1. PPO 中采样 action 后进行了 clamp，但 log probability 可能与实际执行 action 不完全一致。
2. progress potential 可能因为 visibility roadmap 或窗口等待代价产生跳变，导致 progress reward 噪声偏大。
3. 当前训练指标缺少 progress reward、gate wait、gate use、碰撞类型等诊断字段。
4. 课程按固定步数从 C1 切到 C5，C2 以后可能还未学稳就继续升级。
5. feedforward teacher 没有显式时间/窗口相位或记忆，对旋转时变窗口策略可能偏难。

## 下一步建议

优先做小范围、可验证优化：

1. 修正 PPO action/log probability 一致性。
2. 在训练 CSV 中加入 progress reward、progress delta、gate wait/use、closed gate collision、wall collision 等诊断字段。
3. 对 progress delta 或 progress reward 做裁剪/归一化，降低 roadmap 跳变影响。
4. 增加 stage-wise evaluation，分别评估 C1-C5，定位失败从哪个课程开始。
5. 优化课程推进方式：固定步数切换改为成功率阈值或混合旧课程复习。

暂不建议优先改 observation 维度、模型结构或引入学生策略；应先确认 reward/PPO/curriculum 的基础训练信号是否稳定。
