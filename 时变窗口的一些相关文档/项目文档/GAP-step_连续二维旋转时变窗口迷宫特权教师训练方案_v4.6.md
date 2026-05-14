# GAP-Step 连续二维时变窗口迷宫特权教师训练方案

## 1. 目标

训练 PPO 特权教师，让圆盘机器人在连续二维时变窗口迷宫中到达目标。

教师输入是完整图观测，输出是连续二维加速度。教师不执行 A*/MPC/waypoint，不使用专家动作标签。

## 2. 环境

机器人状态：

```text
位置 p
速度 v
动作 a
```

动力学：

```text
v = clip(v + a * dt, v_max)
p = p + v * dt
```

迷宫由网格拓扑生成，再转成连续墙体和窗口。窗口是否可通行由开度和旋转角共同决定。

## 3. 课程

训练顺序：

```text
C1      静态安全窗口
C1_5    高开放占空比动态窗口
C2A     单动态窗口，可能需要等待
C2B     小迷宫，1-2 个动态窗口
C3      加入旋转窗口
C4      中等多窗口迷宫
C5      最终异步多窗口迷宫
```

当前采用逐课程训练。后一课程继承前一课程模型参数，但优化器重置。

## 4. 观测

教师观测为：

```text
GraphObs(
  global_features: [16],
  node_features: [num_nodes, 32],
  node_type: [num_nodes],
  edge_index: [2, num_edges],
  edge_features: [num_edges, 20],
)
```

节点：

- cell node
- gate node

边：

- cell-cell 拓扑边
- gate-cell 边
- self-loop

## 5. 模型

模型是纯 PyTorch GNN actor-critic。

当前读出：

```text
global_h
+ mean_pool(node_h)
+ max_pool(node_h)
+ agent_cell_h
+ goal_cell_h
```

加入 agent 和 goal cell 是为了避免当前位置和目标位置被全图池化稀释。

## 6. PPO 动作

动作使用 tanh-squashed Gaussian。PPO log probability 必须对应实际执行的 squashed action。

不要改回 `Normal -> clamp(action)`。

PPO 训练使用显式旧策略流程：

```text
model_old 负责采样
model 负责更新
每次 update 后同步 model_old <- model
```

回报估计使用 GAE。KL early stop 使用标准非负近似，并记录裁剪率和解释方差。

## 7. 奖励

基础奖励：

```text
+20 成功
-20 碰撞
-0.01 每步
-0.001 * ||action||^2
```

训练时启用连续几何 progress shaping。

当前新增规则：

```text
如果当前 step 撞墙或撞门，正的 progress_reward 置零。
```

这不改变碰撞规则，也不改变成功条件。

## 8. 训练输出

每个课程单独保存：

```text
checkpoints/<课程>/teacher_final.pt
results/<课程>/train_metrics.csv
```

不保存 `teacher_best.pt`。

训练日志为中文实时指标，不显示进度条。

## 9. 配置重点

```yaml
curriculum_mode: stagewise
steps_per_stage: 300000
rollout_steps: 4096
minibatch_size: 512
update_epochs: 4
learning_rate: 0.0001
target_kl: 0.2
entropy_coef: 0.0001
log_std_init: -1.0
min_log_std: -2.0
max_log_std: 0.0
checkpoint_dir: checkpoints
results_dir: results
log_interval_updates: 1
normalize_advantage: true
```

环境：

```yaml
progress_mode: dynamic_geometry
reward_progress: 4.0
reward_timeout: -20.0
suppress_positive_progress_on_collision: true
```

## 10. 实验顺序

先跑：

```bash
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
```

再跑完整训练：

```bash
python -m gap_step.train --config gap_step/configs/train_teacher_full.yaml
```

分析时先看 C1 是否稳定，不要直接从 C5 失败推断全部问题。
