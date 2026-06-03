# Project Context

## Current Focus

The active project line is a generated family of continuous 2D time-varying window mazes trained with a pure privileged PPO teacher.

```text
gap_step/window_maze_env.py
gap_step/train_window.py
gap_step/evaluate_window.py
gap_step/visualize_window.py
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

`复现/TOGT-Planner-reproduction/` contains the source-level reproduction package and notes for arXiv:2309.06837v3. `togt_timevarying_window/` is now an independent TOGT-style 3D dynamic gate/window project using paper-like ordered race gates, not the older maze environment.
