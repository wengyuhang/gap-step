# GAP-Step 实验实施文档

## 当前实验对象

程序生成二维时变窗口迷宫族：

- 正常黑白迷宫；
- 多个两端贴墙的时变窗口；
- 窗口线只存在于白色区域；
- 中间唯一开口随时间变化；
- 智能体动作保持连续二维。

## 实现入口

```text
gap_step/window_maze_env.py
gap_step/train_window.py
gap_step/evaluate_window.py
gap_step/visualize_window.py
```

## 训练设置

- 教师：纯 PPO 特权教师；
- 观测：`GraphObs`；
- 动作：连续 `Box(-1, 1, shape=(2,))`；
- 课程：`C1 -> C1_5 -> C2 -> C2A -> C2B -> C3 -> ... -> C4F -> C5`；
- 不使用 planner、BC、专家演示或 hand-crafted final policy。

正式 C5 采用：

```text
6 windows
full-length maze
mixed straight/polyline/curve windows
gap range 0.72-0.96
```

## 运行命令

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh
conda activate wyh
python -m gap_step.train_window --config gap_step/configs/train_window_c5_short.yaml
python -m gap_step.evaluate_window --checkpoint checkpoints/window_generated/C5/teacher_final.pt --episodes 200 --stage C5
python -m gap_step.visualize_window --checkpoint checkpoints/window_generated/C5/teacher_final.pt
pytest -q
```

## 当前结果

```text
results/window_generated/eval_c5.csv
```

```text
id_test         success 71.5%, collision 27.5%, timeout 1.0%
ood_window_test success 54.0%, collision 46.0%, timeout 0.0%
ood_maze_test   success 74.5%, collision 24.5%, timeout 1.0%
```

## 可视化产物

```text
preview/high_difficulty_window_maze.gif
preview/high_difficulty_window_maze_phases.png
results/window_generated/gifs/success_case.gif
results/window_generated/gifs/collision_case.gif
results/window_generated/gifs/timing_case.gif
results/window_generated/gifs/ood_window_case.gif
results/window_generated/gifs/ood_maze_case.gif
```

## 当前判断

- ID 高难度目标已达到。
- 墙体连续碰撞约束已生效，正式评估中无穿墙成功。
- `ood_window_test` 仍是最明显短板，后续应优先提升对未见开口时序的泛化。
