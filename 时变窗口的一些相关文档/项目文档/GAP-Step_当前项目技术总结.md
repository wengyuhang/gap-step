# GAP-Step 当前项目技术总结

## 1. 当前定位

当前项目是连续二维旋转时变窗口迷宫中的 PPO 特权教师训练系统。

当前主线为：

```text
连续二维迷宫
    -> 完整拓扑图 + gate 动力学特权观测
    -> 纯 PyTorch GNN PPO 教师
    -> ID / OOD-size / OOD-dynamics 评估
```

迷宫、墙体、窗口、碰撞和动作仍然是连续的。GNN 图只是 teacher 的 privileged state，不把机器人运动离散化，也不在推理时执行 A*/MPC/路径规划。

## 2. 代码结构

所有当前可运行代码都放在 `gap_step/`：

```text
env.py          # 连续迷宫、碰撞、GraphObs、奖励、渲染
graph.py        # GraphObs、GraphBatch、图 batch collation
curriculum.py   # C1/C1_5/C2A/C2B/C3/C4/C5 在线课程
model.py        # GNN PPO 教师 Actor-Critic
ppo.py          # graph rollout、GAE、PPO update
train.py        # 教师训练入口
evaluate.py     # ID/OOD 和 stage-wise 评估
visualize.py    # GIF 可视化入口
utils.py        # 通用工具
```

旧目录 `gap_step/envs/`、`gap_step/models/`、`gap_step/teachers/`、`trainers/`、`scripts/` 不再作为代码入口使用。

## 3. 环境与任务

环境是连续方形区域：

```text
Omega_S = [0, S] x [0, S]
```

智能体是圆盘机器人，状态为连续位置和速度，动作是二维连续加速度：

```text
v = clip(v + a * dt, -v_max, v_max)
p = p + v * dt
```

迷宫先由 randomized DFS 生成网格拓扑，再转换为连续墙段和时变窗口槽位。窗口是否可通行由宽度和旋转角共同决定：

```text
safe_width = d(t) >= 2 * robot_radius + safe_margin
safe_angle = abs(wrap(theta(t) - theta_ref)) <= theta_safe
gate_safe = safe_width and safe_angle
```

不可通行窗口按障碍处理；可通行窗口作为连续开口处理。

## 4. GNN 特权教师观测

teacher 当前观测为 `GraphObs`：

```text
global_features: [16]
node_features:   [num_nodes, 32]
node_type:       [num_nodes]
edge_index:      [2, num_edges]
edge_features:   [num_edges, 20]
```

节点包括：

- 每个迷宫 cell 一个 cell node；
- 每个时变窗口一个 gate node。

边包括：

- 相邻 cell 之间的有向 cell-cell 边，标注 wall/open/gate；
- gate node 到两侧 cell 的有向 gate-cell 边；
- 每个节点的 self-loop。

gate 节点和 gate 边提供当前 safe、宽度、余量、角度误差、time-to-open、time-to-close、宽度/旋转周期参数等完整特权信息。

这不是规划算法。GNN 只是可学习的图编码器，策略仍然一次前向输出连续动作。

## 5. 课程学习

当前课程顺序为：

```text
C1   静态 always-safe gate
C1_5 动态宽度 gate，高 open 占空比
C2A  单动态 gate，可能需要等待
C2B  小迷宫，1-2 个动态 gate
C3   加入旋转 gate
C4   中型多 gate
C5   最终异步多 gate 普通迷宫
```

训练和测试仍由程序化生成器在线生成，不保存训练集。

## 6. PPO 教师

教师网络为 GNN actor-critic：

```text
GraphObs -> global/node/edge encoder -> message passing -> graph pooling -> actor/value
```

动作分布为 tanh-squashed Gaussian：

```text
action = tanh(raw_action) * a_max
```

PPO update 中 log probability 与实际执行动作一致。训练支持 target-KL early stop，并保存：

```text
checkpoints/teacher_final.pt
checkpoints/teacher_best.pt
```

默认评估使用 `teacher_best.pt`。

## 7. 验证状态

当前已通过：

```bash
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
python -m gap_step.evaluate --checkpoint checkpoints/teacher_best.pt --episodes 2 --stages C1,C1_5,C2A,C2B,C3,C4,C5 --output /tmp/gap_gnn_eval_smoke.csv
```

smoke 训练只验证流程，不代表策略质量。下一步应集中训练到 C2B，并以 deterministic success 判断 GNN privileged teacher 是否真正学会等待和过门。
