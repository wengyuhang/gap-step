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

Status:

Superseded on 2026-05-13 for the teacher policy. The 39D vector is now retained as a stable prefix inside the 161D time-aware privileged observation.

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

## 2026-05-12: Use adaptive curriculum for full teacher training

Decision:

Full teacher training now uses `curriculum_mode: adaptive`: each stage trains for at least 500k steps, promotes when the recent 100 completed episodes reach 70% success, emits a soft warning after 2M steps without promotion, and stops at 5M steps without forcing a promotion.

Reasoning:

The first full run showed C1 could learn but C2-C5 remained weak under fixed 1M-step stage switches. Promotion should be based on demonstrated stage competence; step limits are safety valves, not the main curriculum criterion.

Status:

Updated on 2026-05-13: full training now also requires deterministic promotion evaluation and uses 5M/10M soft/hard limits.

## 2026-05-12: Use squashed Gaussian PPO actions

Decision:

The teacher policy now samples from a tanh-squashed Gaussian and computes PPO log probabilities for the squashed action that is actually executed.

Reasoning:

The previous `Normal -> clamp(action)` path could store log probabilities for unclamped samples while the environment executed clamped actions, weakening PPO's importance-ratio update.

## 2026-05-12: Remove time-passing reward leakage from progress shaping

Decision:

Progress shaping now compares `potential(old_pos, current_time)` and `potential(new_pos, current_time)`, then clips the delta before scaling. It no longer directly rewards the decrease in future wait time caused only by time passing.

Reasoning:

Time-varying windows can make remaining-time potential decrease while the agent stands still. Waiting can be necessary, but it should not produce large dense reward unless the agent improves its spatial state.

## 2026-05-13: Expand teacher observation with time-aware gate summaries

Decision:

Break the old `obs_dim = 39` teacher contract. Keep the original 39D prefix, then add global time phase and fixed-length summaries for up to 10 gates. The default teacher observation is now 161D.

Reasoning:

The adaptive full run reached C3 and then collapsed; final evaluation showed the policy forgot even earlier stages. Reward shaping used future gate timing, but the policy could only observe current ray geometry. The teacher needs privileged timing features to learn whether to wait, cross, or bypass time-varying windows.

## 2026-05-13: Require deterministic validation for curriculum promotion

Decision:

Adaptive promotion requires both stochastic rollout rolling success and deterministic train-validation success. Hard max remains a stop condition rather than saving or rolling back to a best checkpoint.

Reasoning:

Training rollout success alone can overstate deployable policy quality. The project goal is to actually train through C5, not preserve the best intermediate checkpoint; if a stage cannot meet the stricter criterion before hard max, training should stop and expose the failing stage.
