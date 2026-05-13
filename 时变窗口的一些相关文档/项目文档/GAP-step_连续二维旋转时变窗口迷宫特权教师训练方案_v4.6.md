# GAP-step：连续二维旋转时变窗口迷宫 GNN 特权教师训练方案 v4.6

## 0. 文档定位

本文档定义当前阶段方案：在连续二维迷宫中训练一个 PPO GNN 特权教师策略。环境包含连续墙体、连续状态、连续动作，以及同时具有开闭变化和旋转变化的时变窗口。

当前阶段只训练特权教师，不训练学生模型，不引入行为克隆、启发式演示、SITT、主动感知或视觉输入。

关键修订：

- 教师不再使用局部向量观测；
- 教师接收完整 topology graph 与 gate 动力学；
- GNN 图只是 privileged state，不把运动改成离散格子；
- 策略推理时不运行 A*/MPC/waypoint 跟踪；
- 动作仍为二维连续加速度。

## 1. 任务定义

智能体在连续二维平面中运动，从起点区域出发，到达目标区域。迷宫由连续矩形墙体构成，墙体上存在若干时变窗口。窗口的通行状态由两个因素决定：

1. 窗口开口宽度随时间变化；
2. 窗口方向角随时间旋转。

智能体必须在不碰撞墙体、关闭窗口或环境边界的情况下到达目标。

教师策略为：

```text
pi_T(a_t | G_t^priv)
```

其中 `G_t^priv` 是完整特权图观测，`a_t` 是二维连续加速度动作。

## 2. 连续环境

迷宫空间：

```text
Omega_S = [0, S] x [0, S]
```

机器人动力学：

```text
v_{t+1} = clip(v_t + a_t * dt, -v_max, v_max)
p_{t+1} = p_t + v_{t+1} * dt
```

固定参数：

```text
dt = 0.1
robot_radius = 0.25
a_max = 3.0
v_max = 2.0
max_steps = 500
```

所有碰撞和成功判定都在连续几何中完成。

## 3. 时变窗口

窗口宽度：

```text
d_i(t)=d_min + 0.5 * (d_max - d_min) * (1 + sin(omega_d * t + phi_d))
```

窗口角度：

```text
theta_i(t)=theta0 + theta_amp * sin(omega_theta * t + phi_theta)
```

安全通行：

```text
safe_width = d_i(t) >= 2 * robot_radius + safe_margin
safe_angle = abs(wrap(theta_i(t) - theta_ref)) <= theta_safe
gate_safe = safe_width and safe_angle
```

不可通行窗口在连续碰撞检测中按动态障碍处理；可通行窗口按连续开口处理。

## 4. GNN 特权观测

教师观测为 `GraphObs`：

```text
global_features: [16]
node_features:   [num_nodes, 32]
node_type:       [num_nodes]
edge_index:      [2, num_edges]
edge_features:   [num_edges, 20]
```

### 4.1 Global Features

包含机器人归一化位置/速度、目标相对位置、目标距离、迷宫尺寸、rows/cols、时间相位、episode 进度、gate 数量和当前 safe gate 比例。

### 4.2 Cell Nodes

每个生成拓扑 cell 一个节点。cell node 表达：

- cell center 连续坐标；
- 是否 start cell；
- 是否 goal cell；
- 是否 agent 当前所在 cell；
- 到 agent 和 goal 的相对连续位置与距离。

cell 只用于表达拓扑结构，不代表机器人离散移动。

### 4.3 Gate Nodes

每个窗口一个 gate node。gate node 表达：

- gate center；
- horizontal/vertical orientation；
- slot width；
- 当前 width；
- width clearance；
- 当前 safe；
- angle error；
- time-to-open；
- time-to-close；
- 宽度和旋转动力学参数。

### 4.4 Edges

图边包括：

- cell-cell edge：所有相邻 cell 之间的有向边，标注 wall/open/gate；
- gate-cell edge：每个 gate node 连接其两侧 cell；
- self-loop：每个节点保留自身信息。

edge feature 包含类型 one-hot、方向、距离，以及 gate 边上的安全状态和时间信息。

## 5. GNN PPO 教师

网络结构：

```text
global encoder
node encoder
edge encoder
GNN message passing
mean + max graph pooling
actor head / critic head
```

动作分布：

```text
raw_action ~ Normal(raw_mean, std)
action = tanh(raw_action) * a_max
```

PPO log probability 按实际 squashed action 计算。

## 6. 课程学习

当前课程：

```text
C1   静态 always-safe gate
C1_5 动态宽度 gate，高 open 占空比
C2A  单动态 gate，可能需要等待
C2B  小迷宫，1-2 个动态 gate
C3   加入旋转 gate
C4   中型多 gate
C5   最终异步多 gate 普通迷宫
```

这样避免从 C1 直接跳到完整 C2，使 teacher 先单独学会等待和择时过门。

## 7. 奖励

环境默认奖励：

```text
+20 goal
-20 collision
-0.01 per step
-0.001 * ||action||^2
```

训练配置可启用 continuous-geometry progress shaping。该 shaping 使用连续 visibility geometry 和 gate future safety 估计潜在距离，但不作为 policy 推理时的规划器，也不输出动作目标。

## 8. 评估

训练输出：

```text
checkpoints/teacher_final.pt
checkpoints/teacher_best.pt
results/train_metrics.csv
results/eval_metrics.csv
```

评估默认使用 `teacher_best.pt`：

```bash
python -m gap_step.evaluate --checkpoint checkpoints/teacher_best.pt --episodes 20 --stages C1,C1_5,C2A,C2B,C3,C4,C5
```

首要成功标准是 C2A/C2B deterministic success 明显提升；C2B 稳定后再推进 C3-C5 full training。
