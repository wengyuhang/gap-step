# Architecture

## Active Runtime Line

```text
window_maze_env.py -> GraphObs -> model.py -> ppo.py
train_window.py    -> curriculum training
evaluate_window.py -> ID/OOD evaluation
visualize_window.py -> GIF rollouts
```

## Environment

- Procedural maze generator.
- `WallRect` for static walls.
- `ApertureWindow` for straight/polyline/curve dynamic window bodies.
- Swept-circle continuous collision.
- Compact graph construction around the route, local cells, and window nodes.

## Observation

`GraphObs` keeps the project-wide graph contract:

```text
global_features
node_features
node_type
edge_index
edge_features
```

Window nodes include current gap state plus future gap width/center features.

## Training

- Pure PPO teacher.
- Explicit `model_old` sampling and `model <- model_old` sync.
- Internal bridge curriculum up to `C5`.
- No planner, BC, expert demonstrations, or fallback controller.

## Offline DynaTOGT Extension

```text
复现/TOGT-Planner-reproduction/   source-level reproduction package
togt_timevarying_window/          standalone DynaTOGT dynamic time-varying window experiment
```

`togt_timevarying_window/` 是独立子项目，不使用旧迷宫环境，也不接入 `gap_step/` PPO 主线。它基于 TOGT 论文的 gate 几何约束思想，把静态 `p(t_i) in G_i` 扩展为动态窗口 `p(t_i) in G_i(t_i)`。

当前 DynaTOGT 架构：

```text
geometry.py      2D/3D 窗口几何、局部点映射、裕度计算
environment.py   DynamicWindow / WindowTrack / MotionProfile / 场景生成
optimizer.py     DynaTOGT warm start + L-BFGS-B 连续优化
trajectory.py    Hermite 连续轨迹和速度/加速度/jerk 采样
baselines.py     WaypointCenter / StaticTOGT / DiscreteDynamic / DynaTOGT
experiments.py   smoke/default 实验套件与 summary.csv
visualize.py     中文组会展示风格 PNG/GIF
```

关键语义：

- `ordered_dynamic` / `static` 的 `--order` 是穿越任务序列，不是 permutation；同一窗口可以出现多次。
- `shuffled_dynamic` 保留为一次性 permutation 搜索对照。
- 导出 CSV 中 `contains=True`、`plane_error≈0`、`gate_margin>0` 是穿越成功的数值证据。
