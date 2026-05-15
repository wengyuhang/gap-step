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
checkpoints/window_generated/C5/teacher_final.pt
results/window_generated/eval_c5.csv
results/window_generated/gifs/
preview/high_difficulty_window_maze.gif
preview/high_difficulty_window_maze_phases.png
```
