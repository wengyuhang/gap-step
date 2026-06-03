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

## Offline TOGT-Style Prototype

```text
复现/TOGT-Planner-reproduction/   source-level reproduction package
togt_timevarying_window/          standalone paper-style 3D dynamic gate/window planner
```

The TOGT prototype is a separate project and does not use the old maze environment. It models an ordered 3D race track with dynamic gates/windows `G_i(t)` and searches over gate order, arrival time, and gate-interior candidate points.
