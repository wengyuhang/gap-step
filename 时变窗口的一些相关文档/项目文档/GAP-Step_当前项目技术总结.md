# GAP-Step 当前项目技术总结

## 1. 项目定位

GAP-Step 当前只做一件事：在连续二维时变窗口迷宫中训练 PPO 特权教师。

教师看到完整模拟器图状态，包括迷宫拓扑、窗口动力学、当前窗口安全状态和全局状态。教师直接输出连续二维加速度，不执行 A*/MPC/waypoint，也不使用专家演示或行为克隆。

## 2. 当前代码结构

所有运行入口都在 `gap_step/`：

```text
env.py          连续迷宫、碰撞、奖励、图观测、渲染
graph.py        GraphObs、GraphBatch、图拼接
curriculum.py   C1/C1_5/C2A/C2B/C3/C4/C5 课程
model.py        GNN PPO 教师
ppo.py          旧策略 rollout、GAE、PPO update、策略同步
train.py        逐课程训练入口
evaluate.py     ID/OOD 和分课程评估
visualize.py    GIF 可视化
```

旧的 `trainers/`、`scripts/`、`gap_step/envs/`、`gap_step/models/`、`gap_step/teachers/` 不再作为入口。

## 3. 当前训练方式

训练改为逐课程：

```text
C1 -> C1_5 -> C2A -> C2B -> C3 -> C4 -> C5
```

规则：

- 后一个课程继承前一个课程的模型参数
- 每个课程重新建立优化器
- 每个课程单独保存最终模型和训练指标
- 不再保存 `teacher_best.pt`
- 训练时不显示进度条，改为中文实时指标
- PPO 使用显式旧策略：`model_old` 采样，`model` 更新，更新后同步

输出示例：

```text
checkpoints/C1/teacher_final.pt
results/C1/train_metrics.csv
checkpoints/C5/teacher_final.pt
results/C5/train_metrics.csv
```

## 4. 本次失败原因判断

上一轮训练一直卡在 C1，说明主要问题不是复杂动态门，而是基础连续迷宫导航没有稳定学会。

核心原因：

- 全局 goal 向量不是路径答案，迷宫里直冲目标通常会撞墙
- 只靠全图池化会稀释 agent cell 和 goal cell 信息
- 训练后期 entropy/std 变大，策略退化为高噪声动作
- 撞墙前的进展可能仍得到正 progress reward
- 原 PPO 主 KL 指标可能为负，采样策略和更新策略语义不够清楚

## 5. 本次简化修正

- `entropy_coef` 降到 `0.0001`
- `log_std_init` 改为 `-1.0`
- `max_log_std` 降到 `0.0`
- `reward_progress` 提高到 `4.0`
- `reward_timeout` 改为 `-20.0`
- 撞墙或撞门时，正的 `progress_reward` 置零
- GNN 读出加入 agent 所在 cell 和 goal cell 表示
- 训练日志改成中文实时输出
- PPO 改为显式旧策略流程
- KL 改为标准非负近似，并记录裁剪率和解释方差

## 6. 运行命令

```bash
cd /home/jack/wyh/时变窗口
source /home/jack/anaconda3/etc/profile.d/conda.sh
conda activate wyh
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
python -m gap_step.train --config gap_step/configs/train_teacher_full.yaml
```

评估：

```bash
python -m gap_step.evaluate --checkpoint checkpoints/C5/teacher_final.pt
python -m gap_step.evaluate --checkpoint checkpoints/C5/teacher_final.pt --episodes 20 --stages C1,C1_5,C2A,C2B,C3,C4,C5
```

## 7. 下一步

先看 C1 是否稳定成功。如果 C1 仍不稳定，优先看 KL、裁剪率、解释方差、熵、std、超时率和碰撞率，再决定是否继续调超参数；不要急着分析 C5。
