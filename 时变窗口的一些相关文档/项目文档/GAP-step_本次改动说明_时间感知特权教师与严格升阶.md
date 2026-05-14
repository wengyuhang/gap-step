# GAP-Step 本次改动说明

## 背景

上一轮训练虽然已经使用完整图特权观测，但训练一直卡在 C1。C1 没有复杂动态门，说明主要问题不是“动态门信息不足”，而是基础连续导航、奖励诱导和 PPO 稳定性不够。

## 本次改动目标

先做小而有效的修改：

- 训练更容易观察
- 输出更清楚
- 降低探索噪声
- 避免撞墙也拿正进展奖励
- 让模型显式看到 agent cell 和 goal cell
- 按标准 PPO 流程拆清旧策略采样和当前策略更新

## 训练流程改动

训练改成逐课程：

```text
C1 -> C1_5 -> C2A -> C2B -> C3 -> C4 -> C5
```

每个课程：

- 继承前一课程的模型参数
- 重置优化器
- 单独保存最终模型
- 单独保存训练 CSV

输出：

```text
checkpoints/<课程>/teacher_final.pt
results/<课程>/train_metrics.csv
```

不再保存 `teacher_best.pt`。

PPO 训练流程同步改为：

```text
model_old 采样 rollout -> model 做 PPO 更新 -> model_old <- model
```

回报估计保留 GAE，动作仍是 tanh-squashed Gaussian。

## 日志改动

去掉进度条，改为中文实时指标：

```text
课程 C1 | 更新 12 | 成功率 0.10 | 碰撞率 0.80 | 超时率 0.10 | 平均回报 -8.42 | 熵 2.91 | KL 0.012
```

现在日志还会输出裁剪率和解释方差，用来判断 PPO 是否真的在有效更新。

## 稳定性改动

配置默认改为：

```yaml
entropy_coef: 0.001
max_log_std: 0.5
```

原因：之前训练后期 entropy/std 过大，策略变成高噪声动作。

PPO 主 KL 指标改为标准非负近似，避免使用可能为负的 `old_logp - new_logp` 均值误导 early stop。

## 奖励改动

如果当前 step 发生碰撞，并且 progress reward 为正，则置零。

这不会改变碰撞判定和成功判定，只是避免“冲墙也有收益”。

## 模型改动

GNN 主体不变。actor/critic 输入现在包含：

```text
global_h
mean_pool
max_pool
agent_cell_h
goal_cell_h
```

这样模型不只依赖全图平均，也能直接看到当前位置和目标位置。

## 验证命令

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh
conda activate wyh
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
```
