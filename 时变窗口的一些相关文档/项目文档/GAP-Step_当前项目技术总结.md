# GAP-Step 当前项目技术总结

## 1. 当前状态

项目已从“单张示例图/动态格子通道”切换为“程序生成二维时变窗口迷宫族”。

当前正式主线：

```text
window_maze_env.py + 纯 PPO 特权教师 + 课程学习
```

目标是让连续二维智能体在未见高难度地图族上穿过由墙到墙线/曲线形成的动态开口。

## 2. 环境

- 迷宫由 seed 程序生成。
- 时变窗口两端贴墙，窗口线是黑色障碍，中间只有一个蓝色动态开口。
- 窗口支持直线、折线、曲线。
- 动作连续，碰撞使用 swept-circle。
- 任何黑墙/窗口实体接触都立即失败。

正式 C5 当前保留 6 个不规则窗口、全路径迷宫和混合曲线形态，最窄开口下界为 `0.72`。

## 3. 教师与训练

教师使用纯 PyTorch GNN：

```text
GraphObs(global_features, node_features, node_type, edge_index, edge_features)
```

图中包含 cell node、window node、拓扑边和 self-loop。窗口节点包含未来若干相位的开口宽度与中心位置。

训练仍是纯 PPO：

- `model_old` 采样；
- `model` 更新；
- 每轮更新后同步；
- 不使用 planner、BC、专家演示或最终兜底策略。

训练中做过的关键修正：

- 修复 rollout horizon 过短导致长回合永远跑不完；
- 改为紧凑图观测，显著减少图构建成本；
- 把窗前 shaping 从“贴迷宫中线”切换为“优先对准动态开口”；
- 为窗口节点加入未来开口轨迹；
- 将正式 C5 几何裕量校准到纯 PPO 可学习区间。

## 4. 结果

正式汇总文件：

```text
results/window_generated/eval_c5.csv
```

结果：

```text
id_test         200 episodes, success 71.5%
ood_window_test 200 episodes, success 54.0%
ood_maze_test   200 episodes, success 74.5%
```

所有 split 的 `wall_collision_rate` 都是 `0.0%`；当前主要失败源仍是动态窗口擦碰。

## 5. 产物

```text
checkpoints/window_generated/C5/teacher_final.pt
results/window_generated/C5/train_metrics.csv
results/window_generated/eval_c5.csv
results/window_generated/gifs/*.gif
preview/high_difficulty_window_maze.gif
preview/high_difficulty_window_maze_phases.png
```

GIF 包含成功、碰撞、时机、OOD window、OOD maze 案例。

## 6. 当前结论

高难度 ID 测试目标已经达线，但对未见窗口时序的泛化仍偏弱。下一步若继续提升，应优先围绕 `ood_window_test` 做课程与观测增强，而不是继续扩迷宫拓扑。
