# Decisions

## 2026-05-11: Refactor to v4.6 PPO teacher only

Decision:

Replace the visual student / BC / heuristic teacher project with the v4.6 continuous 2D rotating time-varying window maze. Train only a PPO privileged teacher.

Reasoning:

The current research stage is to obtain a teacher policy in a procedurally generated continuous maze before introducing student learning.

## 2026-05-11: Keep all active code in `gap_step/`

Decision:

All runnable code now lives directly in the `gap_step/` package. Do not keep active code in `trainers/`, `scripts/`, or nested `gap_step/envs`, `gap_step/models`, `gap_step/teachers` packages.

Reasoning:

The user requested a clean single-folder code layout without backward compatibility.

## 2026-05-11: Use fixed-dimensional ray observations

Decision:

Use `N_ray = 32`, `ray_max_dist = 0.35 * S`, and `obs_dim = 39`.

Reasoning:

Window count should not enter the teacher observation dimension. Scaling ray distance by maze size makes ID and OOD-size evaluation better matched while preserving a fixed network input.

## 2026-05-11: Ignore generated outputs

Decision:

Ignore `data/`, `checkpoints/`, `logs/`, `runs/`, and `results/`.

Reasoning:

These are reproducible experiment artifacts and may become large.

## 2026-05-11: Generate ordinary maze topology before continuous walls

Decision:

Generate each maze as a randomized grid maze, add a small number of loop openings, then convert closed edges and selected passage edges into continuous `WallSegment` and `Gate` objects.

Reasoning:

This produces ordinary maze-like layouts with both horizontal and vertical corridors while keeping the implementation compact and readable.
