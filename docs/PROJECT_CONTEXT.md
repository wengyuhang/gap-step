# Project Context

## Current Focus

The repository mainline remains a generated family of continuous 2D time-varying window mazes trained with a pure privileged PPO teacher. The current active research extension is the multi-method non-convex time-varying window traversal project under `nonconvex_timevarying_window/`.

```text
gap_step/window_maze_env.py
gap_step/train_window.py
gap_step/evaluate_window.py
gap_step/visualize_window.py

nonconvex_timevarying_window/PROBLEM_DEFINITION.md
nonconvex_timevarying_window/atlas_dynatogt/
nonconvex_timevarying_window/sc_dynatogt/
```

## Environment Contract

- Static black walls are hard obstacles.
- Each aperture window is a wall-to-wall line/polyline/curve with one dynamic gap.
- The agent moves with continuous 2D actions.
- Collision is swept-circle and terminal for walls, window bodies, boundary contact, or post-phase overlap.
- Blue overlays visualize current openings only.

## Current Results

```text
id_test         200 episodes, 71.5% success
ood_window_test 200 episodes, 54.0% success
ood_maze_test   200 episodes, 74.5% success
```

The ID target is met. Unseen window timing is the current generalization weakness.

## Key Outputs

```text
gap_step/checkpoints/window_generated/C5/teacher_final.pt
gap_step/results/window_generated/eval_c5.csv
gap_step/results/window_generated/gifs/
gap_step/preview/high_difficulty_window_maze.gif
gap_step/preview/high_difficulty_window_maze_phases.png
```

## TOGT Reproduction Context

`复现/TOGT-Planner-reproduction/` contains the source-level reproduction package and notes for arXiv:2309.06837v3.

`togt_timevarying_window/` has been rebuilt as **DynaTOGT**, an independent dynamic time-varying window traversal experiment. It keeps the TOGT paper idea of choosing traversal points inside gate geometry, but changes the constraint from static `p(t_i) in G_i` to dynamic/deformable `p(t_i) in G_i(t_i)`.

Current DynaTOGT facts:

- independent from `gap_step/` PPO and the old maze environment;
- supports moving, rotating, scaling/deforming 3D windows;
- supports arbitrary ordered traversal task sequences, including repeated visits to the same window;
- default canonical order remains `G1 -> G6 -> G3 -> G2 -> G5 -> G4`;
- repeated demo example uses `G1 -> G6 -> G1 -> G3 -> G2 -> G5 -> G4 -> G2`;
- exports Chinese presentation-style PNG/GIF plus trajectory CSV under `togt_timevarying_window/results/`;
- traversal evidence is recorded per crossing with `contains`, `plane_error`, and `gate_margin`.

## Non-Convex Time-Varying Window Research Context

`nonconvex_timevarying_window/` 是一个独立的非凸时变窗口研究总目录，不属于 `gap_step/` PPO 主线，也不替代已有的凸窗口 `togt_timevarying_window/` 子项目。

研究目标是在论文 *Time-Optimal Gate-Traversing Planner for Autonomous Drone Racing*（`arXiv:2309.06837v3`）的 TOGT 问题上，将原有静态凸窗口扩展为非凸且随时间平移、旋转和缩放的窗口。

当前通用问题范围：

- 窗口是无洞、无自交的简单闭合非凸区域；
- 折线、光滑曲线和混合边界都可以通过有序边界点表示；
- 无人机按给定顺序穿越窗口，当前任务不要求重复穿越同一窗口；
- 穿越点必须位于穿越时刻的真实非凸区域内，不能用凸包代替真实窗口验证；
- 目标是在窗口几何、时变运动、指定顺序和轨迹动力学约束下尽量减小总飞行时间。

总目录和方法目录的边界为：

```text
nonconvex_timevarying_window/
  README.md                 总任务与方法索引
  PROBLEM_DEFINITION.md     与具体算法无关的问题定义
  atlas_dynatogt/           已实现的 AtlasDynaTOGT 方法
  sc_dynatogt/              已实现的 SC-DynaTOGT 方法
  <algorithm_name>/        后续方法的并列目录
```

当前有两个相互独立的方法：

- `AtlasDynaTOGT`：将非凸区域用 ear clipping 剖分成三角 chart atlas，使用 softmax 重心坐标生成 chart 内穿越点；
- `SC-DynaTOGT`：Chang 等人的工作只用于边界均匀重采样和角点保留，内部取点严格使用圆盘 Schwarz--Christoffel 映射，并接入原 TOGT 的时间变量、degree-7 MINCO 和动力学代价。

两种方法不共享内部参数化代码，各自从本目录的 `experiments.py` 进入。

当前验证状态：

```text
default suite: 14 scenarios, 14 successes
pytest -q nonconvex_timevarying_window/atlas_dynatogt/tests  # 7 passed
SC-DynaTOGT smoke: E0--E5 all passed
SC-DynaTOGT mapping: 1,000,000 / 1,000,000 legal, no NaN/Inf/degenerate Jacobian
pytest -q nonconvex_timevarying_window/sc_dynatogt/tests  # 85 passed (2026-07-14)
```
