# GAP-Step 实验实施文档

## 1. 实验目标

训练 PPO 特权教师，使其在连续二维时变窗口迷宫中直接输出连续加速度动作。

当前只做 teacher，不做视觉学生、BC、专家演示、A*/MPC 或 3D 动力学。

## 2. 当前实验流程

训练采用逐课程方式：

```text
C1 -> C1_5 -> C2A -> C2B -> C3 -> C4 -> C5
```

每个课程：

- 继承前一课程的模型参数
- 重置优化器
- 保存最终模型
- 保存独立训练指标
- PPO rollout 使用 `model_old`，更新使用 `model`，每次更新后同步旧策略

输出：

```text
checkpoints/<课程>/teacher_final.pt
results/<课程>/train_metrics.csv
```

不再保存 `teacher_best.pt`。

## 3. 当前关键设置

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

环境奖励：

```yaml
progress_mode: dynamic_geometry
reward_progress: 4.0
reward_timeout: -20.0
suppress_positive_progress_on_collision: true
```

含义：动态几何 progress 是主 shaping；超时和碰撞一样是失败；撞墙或撞门时，不允许保留正的 progress reward。

C5 tune 配置另有放宽口径：

```yaml
config: gap_step/configs/train_teacher_c5_tune.yaml
robot_radius: 0.1
safe_margin: 0.0
max_steps: 800
```

该配置只用于 C5 快速诊断，不等同于 full 配置。

## 4. 模型说明

GNN 主体保持简单。actor/critic 输入包括：

```text
global_h
mean_pool(node_h)
max_pool(node_h)
agent_cell_h
goal_cell_h
```

这样可以直接利用 agent 所在 cell 和 goal cell 信息。

当前 `GraphObs.global_features` 为 26 维，包含动态几何引导、门等待、下一路标、前方关闭门信息和动作先验。actor 可将动作先验作为 tanh-squashed Gaussian 的 raw mean，PPO 学残差。

## 5. 运行命令

```bash
cd /home/jack/wyh/时变窗口
source /home/jack/anaconda3/etc/profile.d/conda.sh
conda activate wyh
pytest -q
```

烟测：

```bash
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
```

完整训练：

```bash
python -m gap_step.train --config gap_step/configs/train_teacher_full.yaml
```

C5 tune：

```bash
python -m gap_step.train --config gap_step/configs/train_teacher_c5_tune.yaml
```

## 6. 训练日志

训练时没有进度条，每次 PPO update 输出中文实时指标：

```text
课程 C1 | 更新 12 | 阶段步数 1536 | 回合 8 | 成功率 0.125 | 滚动成功率 0.100 | 碰撞率 0.750 | 超时率 0.125 | 平均回报 -8.42 | 动作范数 2.10 | 熵 2.91 | KL 0.0120 | 裁剪率 0.125 | 解释方差 0.500 | PPO更新 4
```

## 7. 评估命令

```bash
python -m gap_step.evaluate --checkpoint checkpoints/C5/teacher_final.pt
python -m gap_step.evaluate --checkpoint checkpoints/C5/teacher_final.pt --episodes 20 --stages C1,C1_5,C2A,C2B,C3,C4,C5
python -m gap_step.evaluate --checkpoint checkpoints/C5/teacher_final.pt --episodes 50 --stages C5 --output results/eval_c5_tune.csv
```

也可以只评估某个课程模型：

```bash
python -m gap_step.evaluate --checkpoint checkpoints/C1/teacher_final.pt --episodes 20 --stages C1
```

## 8. 实验判断顺序

优先看 C1：

- 成功率是否稳定上升
- 碰撞率是否下降
- 超时率是否下降
- entropy/std 是否不再持续膨胀
- KL 是否使用非负近似且没有长期触发 early stop
- 裁剪率是否没有长期接近 0 或 1
- 解释方差是否没有长期明显为负
- PPO 更新次数是否没有长期掉到 1

C1 稳定后，再继续看 C1_5、C2A、C2B。

## 9. 当前 C5 tune 结果

正式 50 回合 C5 ID 评估最好结果：

```text
success_rate = 0.68
closed_gate_collision_rate = 0.16
wall_collision_rate = 0.10
timeout_rate = 0.06
```

训练 rollout 中可达到 75%-80%，但泛化评估尚未真实超过 70%。后续优先降低 closed gate collision，再处理 wall collision。

50 回合泛化评估：

```text
id_test          success_rate = 0.68
ood_size_test    success_rate = 0.64
ood_dynamics     success_rate = 0.40
```

GIF 输出：

```text
results/typical_success.gif
results/typical_wait.gif
results/typical_collision.gif
```

当前 checkpoint 下三个默认可视化 seed 都成功，`typical_collision.gif` 是旧案例名。
