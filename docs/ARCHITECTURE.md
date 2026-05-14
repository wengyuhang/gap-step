# Architecture

## 主流程

```text
gap_step/configs/train_teacher_*.yaml
    -> python -m gap_step.train
    -> C1 -> C1_5 -> C2A -> C2B -> C3 -> C4 -> C5
    -> checkpoints/<课程>/teacher_final.pt
    -> results/<课程>/train_metrics.csv

checkpoints/C5/teacher_final.pt
    -> python -m gap_step.evaluate
    -> results/eval_metrics.csv

checkpoints/C5/teacher_final.pt
    -> python -m gap_step.visualize
    -> results/*.gif
```

不再保存或依赖 `teacher_best.pt`。

## 模块

- `curriculum.py`：生成 C1、C1_5、C2A、C2B、C3、C4、C5 课程迷宫
- `env.py`：连续迷宫、碰撞、奖励、图观测、渲染
- `graph.py`：`GraphObs`、`GraphBatch`、图拼接
- `model.py`：纯 PyTorch GNN actor-critic
- `ppo.py`：旧策略 rollout、GAE、PPO update、策略同步
- `train.py`：逐课程训练、中文实时日志、分课程保存
- `evaluate.py`：ID/OOD 和分课程评估
- `visualize.py`：GIF 可视化

## 环境

每局采样一个连续方形迷宫。迷宫先生成离散拓扑，再转换为连续墙段和时变窗口。

奖励基础项：

```text
+20.0 成功
-20.0 碰撞
-0.01 每步
-0.001 * ||action||^2
```

训练配置启用连续几何 progress shaping。现在如果当前 step 发生碰撞，并且 progress reward 为正，则该正奖励会被置零，避免“撞墙前进”得到收益。

## 图观测

教师看到完整特权图：

- cell node：cell 中心、相对 agent/goal、start/goal/agent 标记
- gate node：窗口中心、开度、旋转、安全状态、time-to-open/time-to-close、动力学参数
- cell-cell edge：wall/open/gate 类型
- gate-cell edge：gate 与相邻 cell 连接
- self-loop

## 模型

模型仍是纯 PyTorch GNN。

当前读出方式：

```text
global_h
+ mean_pool(node_h)
+ max_pool(node_h)
+ agent_cell_h
+ goal_cell_h
```

这样 actor/critic 不只依赖全图池化，也能直接看到 agent 所在 cell 和 goal cell 的表示。

## 训练

训练模式为 `stagewise`：

- 每个课程训练固定 `steps_per_stage`
- 下一课程继承上一课程模型参数
- 每个课程重置 Adam 优化器
- 每个 PPO update 输出中文性能指标
- 每个课程单独保存最终模型和 CSV 指标

PPO 使用显式旧策略流程：

- `model_old` 只用于 rollout 采样
- `model` 只用于梯度更新
- 每次 PPO update 后同步 `model_old <- model`
- KL 使用标准非负近似，并记录裁剪率和解释方差

当前稳定性默认：

```text
learning_rate = 0.0001
rollout_steps = 4096
update_epochs = 4
target_kl = 0.2
entropy_coef = 0.0001
log_std_init = -1.0
max_log_std = 0.0
reward_progress = 4.0
reward_timeout = -20.0
```

## 边界

不引入 A*/MPC/waypoint 执行，不使用专家动作标签，不改变碰撞规则、成功条件或迷宫生成语义。
