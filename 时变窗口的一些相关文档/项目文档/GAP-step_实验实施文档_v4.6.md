# GAP-step 实验实施文档 v4.6

## 0. 实验目标

本实验实现连续二维旋转时变窗口迷宫，并训练 PPO 特权教师。当前教师使用完整拓扑图和 gate 动力学作为 privileged observation，由纯 PyTorch GNN actor-critic 输出连续加速度动作。

本阶段不实现学生模型，不使用启发式专家演示，不做行为克隆，也不在推理时运行 A*/MPC/waypoint 跟踪。

## 1. 项目结构

```text
gap_step/
  graph.py
  curriculum.py
  env.py
  model.py
  ppo.py
  train.py
  evaluate.py
  visualize.py
  tests/
checkpoints/
  teacher_final.pt
  teacher_best.pt
results/
  train_metrics.csv
  eval_metrics.csv
```

所有可运行代码直接位于 `gap_step/`，不再使用旧的 `trainers/`、`scripts/`、`gap_step/envs/`、`gap_step/models/`、`gap_step/teachers/`。

## 2. 连续环境

环境空间：

```text
Omega_S = [0, S] x [0, S]
```

机器人为圆盘，状态和动作均连续：

```text
state = (pos, vel)
action = (ax, ay)
v = clip(v + action * dt, -v_max, v_max)
p = p + v * dt
```

默认参数：

```text
robot_radius = 0.25
dt = 0.1
v_max = 2.0
a_max = 3.0
max_steps = 500
```

## 3. 时变窗口

每个 gate 包含位置、方向、slot 宽度、宽度变化参数、旋转变化参数，以及它对应的 topology edge。

当前宽度：

```text
d(t) = d_min + 0.5 * (d_max - d_min) * (1 + sin(omega_d * t + phi_d))
```

当前角度：

```text
theta(t) = theta0 + theta_amp * sin(omega_theta * t + phi_theta)
```

安全通行：

```text
safe_width = d(t) >= 2 * robot_radius + safe_margin
safe_angle = abs(wrap(theta(t) - theta_ref)) <= theta_safe
gate_safe = safe_width and safe_angle
```

碰撞和成功判定仍使用连续几何，不使用图上的离散移动。

## 4. GNN 特权观测

`ContinuousMazeEnv.reset()` 和 `step()` 返回 `GraphObs`：

```text
global_features: [16]
node_features:   [num_nodes, 32]
node_type:       [num_nodes]
edge_index:      [2, num_edges]
edge_features:   [num_edges, 20]
```

节点：

- cell node：每个迷宫 cell 一个节点，包含中心坐标、start/goal/agent flags、到 agent/goal 的相对关系。
- gate node：每个窗口一个节点，包含中心坐标、方向、slot 宽度、当前宽度、safe、time-to-open、time-to-close、宽度/旋转动力学参数。

边：

- cell-cell edge：所有相邻 cell 都建有向边，并标注 wall/open/gate；
- gate-cell edge：gate node 与两侧 cell 相连；
- self-loop：每个节点保留自身信息。

图大小随迷宫大小变化，不做 padding。`gap_step.graph.collate_graph_obs` 在 PPO minibatch 中拼接图并自动 offset `edge_index`。

## 5. 课程

当前阶段顺序：

```text
C1   静态 always-safe gate
C1_5 动态宽度 gate，高 open 占空比
C2A  单动态 gate，可能需要等待
C2B  当前小型动态 gate 任务，1-2 个 gate
C3   加入旋转 gate
C4   中型多 gate
C5   最终异步多 gate 普通迷宫
```

`C2` 仅作为旧命令兼容别名，内部映射到 `C2B`；正式文档和实验使用 `C2B`。

## 6. PPO 与 checkpoint

教师模型：

```text
GraphObs
  -> global/node/edge encoders
  -> GNN message passing
  -> mean + max graph pooling
  -> actor/value heads
```

动作分布：

```text
raw_action ~ Normal(raw_mean, std)
action = tanh(raw_action) * a_max
```

训练保存：

```text
checkpoints/teacher_final.pt
checkpoints/teacher_best.pt
```

`teacher_best.pt` 根据 deterministic promotion eval 更新。评估默认使用 best checkpoint。

## 7. 命令

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh && conda activate wyh
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
python -m gap_step.evaluate --checkpoint checkpoints/teacher_best.pt --episodes 20 --stages C1,C1_5,C2A,C2B,C3,C4,C5
python -m gap_step.evaluate --checkpoint checkpoints/teacher_best.pt
```

smoke 训练只验证流程，不代表策略质量。
