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

## 2026-05-12: Add continuous-geometry progress shaping for PPO training

Decision:

Keep strict v4.6 sparse rewards as the environment default, but enable `dynamic_geometry` progress shaping in teacher training configs.

Reasoning:

The sparse reward run collapsed to waiting for timeout with zero successes. The shaping term uses continuous visibility geometry and future gate safety sampling, so windows can be waited for or bypassed depending on estimated cost without changing the observation contract or collision semantics.

## 2026-05-12: Do not use grid cells for reward potentials

Decision:

Reward shaping potentials must be computed from continuous geometry: positions, wall rectangles, gate approach points, line-segment visibility, and future gate safety. Grid cells or `open_edges` may define maze generation, but they are not the state or path basis for progress reward.

Reasoning:

The environment dynamics, collision, and observations are continuous. A cell-based reward can give misleading progress signals around walls and time-varying windows, especially when a window may be optional, temporarily unsafe, or the only viable passage.

## 2026-05-12: Treat dynamic shaping as an experimental aid, not a solved teacher recipe

Decision:

Keep `dynamic_geometry` shaping available and enabled in current teacher configs, but do not consider the teacher training recipe solved after the first full C1-C5 run.

Reasoning:

The full run completed and evaluated successfully, but final success rates stayed near 5% across ID, OOD-size, and OOD-dynamics splits. The next iteration should tune or redesign reward scale, potential clipping, gate wait cost, and curriculum progression rather than adding new model families or changing the observation contract.

## 2026-05-12: Full curriculum stage comes from `train.py`

Decision:

For full teacher training, `train.py` determines C1-C5 progression from `global_steps` and `steps_per_stage` and passes the active stage into rollout resets. The `env.stage_name` YAML field remains a fallback/default when an environment is reset without an explicit stage override.

Reasoning:

This keeps single-environment smoke/manual runs simple while allowing one full config to train all five curriculum stages without duplicating environment config blocks.
