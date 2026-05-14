# Roadmap

## 当前主线

- 连续二维时变窗口迷宫
- PPO 特权教师
- 纯 PyTorch GNN actor-critic
- 图观测包含完整拓扑和 gate 动力学
- 逐课程训练：`C1 -> C1_5 -> C2A -> C2B -> C3 -> C4 -> C5`
- 每个课程只保存最终模型
- C5 目前有独立 tune 配置，正式 50 回合 ID 评估最好为 `0.68`，尚未达到 `0.70`
- 50 回合泛化评估：ID `0.68`，OOD size `0.64`，OOD dynamics `0.40`

## 近期任务

1. 优先压 OOD dynamics 的 `closed_gate_collision_rate`。
2. 用 GIF 可视化逐个看 closed gate collision、wall collision 和 timeout 的轨迹。
3. 如果继续使用 PPO，避免长训过拟合；当前短训 5 更新好于 10 更新。
4. 回到原始 full 口径前，需要确认 C5 tune 已稳定超过 `0.70`。

## 后续可能工作

- 如果 agent/goal 读出仍不够，再考虑更强的图读出或 attention。
- 增加更细的 gate 等待、穿越速度和门前刹停诊断。
- 只有 teacher 稳定解决 C5 后，再考虑学生策略。
