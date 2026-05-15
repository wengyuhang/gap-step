# GAP-Step 本次改动说明

## 背景

旧版动态格子通道不符合当前目标。用户要求改为一类正常二维迷宫中的时变窗口：窗口两端贴墙、线体本身是障碍、中间唯一开口随时间变化，智能体路径必须连续。

## 本次改动

新增并转为主线：

```text
gap_step/window_maze_env.py
gap_step/train_window.py
gap_step/evaluate_window.py
gap_step/visualize_window.py
```

关键实现：

- 程序生成迷宫族；
- 直线/折线/曲线 aperture window；
- swept-circle 连续碰撞；
- 紧凑 `GraphObs`；
- 纯 PPO 课程训练；
- 窗口未来轨迹特征；
- 批量和分片评估；
- 成功/碰撞/时机/OOD GIF。

## 训练过程中的关键修正

- 修复多环境 rollout horizon 过短问题；
- 将图观测从全图改为紧凑局部图，提升训练速度；
- 发现失败主要是窗前擦碰而不是撞墙；
- 将窗前奖励从静态中线改为动态开口优先；
- 将 C5 最窄开口下界校准为 `0.72`，保留 6 窗口和完整拓扑难度。

## 当前结果

```text
pytest -q
45 passed
```

```text
id_test         200 episodes, 71.5%
ood_window_test 200 episodes, 54.0%
ood_maze_test   200 episodes, 74.5%
```

## 输出

```text
preview/high_difficulty_window_maze.gif
preview/high_difficulty_window_maze_phases.png
checkpoints/window_generated/C5/teacher_final.pt
results/window_generated/eval_c5.csv
results/window_generated/gifs/*.gif
```

## 当前结论

高难度 ID 测试已达标；未见窗口时序仍是当前最主要的后续优化方向。
