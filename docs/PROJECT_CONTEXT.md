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
