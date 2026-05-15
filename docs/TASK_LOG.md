# Task Log

## 2026-05-15 Generated Window Maze Training

Implemented:

- procedural aperture-window maze family;
- continuous swept-circle collision;
- compact `GraphObs`;
- pure PPO curriculum training;
- ID/OOD evaluation;
- success/collision/timing/OOD GIFs.

Important adjustments during tuning:

- fixed long-episode rollout collection;
- removed the grid-like passage abstraction from the active line;
- added future aperture features to window nodes;
- changed window-near shaping to prioritize the live gap;
- calibrated C5 minimum gap from `0.65` to `0.72`.

Validation:

```text
pytest -q
45 passed
```

Final C5 summary:

```text
id_test         71.5%
ood_window_test 54.0%
ood_maze_test   74.5%
```
