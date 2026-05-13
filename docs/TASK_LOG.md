# Task Log

## 2026-05-11

Refactored GAP-Step to v4.6 PPO teacher only:

- moved active code into flat `gap_step/` modules
- removed old visual student, BC, heuristic teacher, `trainers/`, and `scripts/` paths
- added randomized grid-maze curriculum C1-C5 with continuous walls and time-varying windows
- added 39D privileged ray observation with `ray_max_dist = 0.35 * S`
- added PPO teacher training, evaluation, and visualization entrypoints
- updated tests for curriculum, environment, and actor-critic model

Validation:

- `pytest -q`
- `python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml`

## 2026-05-12

Implemented and validated continuous-geometry dynamic progress shaping for teacher training:

- added optional `dynamic_geometry` progress shaping in `ContinuousMazeEnv`
- kept strict v4.6 sparse rewards as the environment default
- enabled dynamic progress shaping in teacher smoke/full configs
- documented that reward potentials must use continuous geometry, not grid cells
- clarified that full training advances C1-C5 from `train.py` using `steps_per_stage`; `env.stage_name` in YAML is only the reset default when no stage override is passed

Validation:

- `pytest -q`: 15 passed
- `python -m gap_step.train --config gap_step/configs/train_teacher_full.yaml`: completed C1-C5 and saved `checkpoints/teacher_final.pt`
- `python -m gap_step.evaluate --checkpoint checkpoints/teacher_final.pt`: completed ID, OOD-size, and OOD-dynamics evaluation

Result:

- `id_test` success rate: 5.0%
- `ood_size_test` success rate: 5.5%
- `ood_dynamics_test` success rate: 5.0%

Open issues:

- Dynamic geometry shaping runs end-to-end but is not yet sufficient to train a strong teacher.
- C5 training remains collision-heavy; OOD dynamics evaluation is mostly timeout-heavy.
- Next work should inspect reward scale, potential clipping, gate wait cost, and curriculum smoothness before treating the teacher as solved.

## 2026-05-12 Follow-Up

Implemented the next conservative training fixes:

- replaced `Normal -> clamp(action)` with tanh-squashed Gaussian actions so PPO log probabilities match executed actions
- changed progress shaping to compare old/new positions at the same current time and clip `progress_delta`
- added adaptive curriculum mode with success-rate promotion, soft warning, and hard stop
- added training diagnostics for progress reward, gate usage/wait time, collision type, action norm, and curriculum status
- added stage-wise evaluation via `--stages C1,C2,C3,C4,C5`

Validation:

- `pytest -q`: 22 passed
- `python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml`: completed and wrote adaptive curriculum diagnostics
- `python -m gap_step.evaluate --checkpoint checkpoints/teacher_final.pt --episodes 20 --stages C1,C2,C3,C4,C5`: completed

Note:

- The smoke run overwrites `checkpoints/teacher_final.pt` and `results/*.csv`; current local generated outputs are smoke artifacts, not the previous full-run artifacts.
- A new adaptive full training run is still needed to judge policy quality.

## 2026-05-13

Implemented the next teacher-training iteration after the adaptive full run failed in C3:

- expanded teacher observation from the old 39D ray-only vector to a 161D time-aware privileged vector while preserving the original 39D prefix
- added fixed-length summaries for up to 10 dynamic gates, including safety, timing, clearance, orientation, and wait-cost features
- added `min_log_std` / `max_log_std` protection and PPO diagnostics for effective std
- tightened adaptive promotion so a stage requires both rollout rolling success and deterministic train-validation success before advancing
- kept hard max as a stop condition; training does not save best/intermediate checkpoints
- updated full config soft/hard max to 5M/10M steps per stage

Validation:

- `pytest -q`: 25 passed
- `python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml`: completed and wrote `obs_dim=161` metrics/checkpoint
- `python -m gap_step.evaluate --checkpoint checkpoints/teacher_final.pt --episodes 20 --stages C1,C2,C3,C4,C5`: completed

Open issues:

- The smoke checkpoint is only a plumbing check and has no policy-quality meaning.
- A new full adaptive run is needed to test whether time-aware gate features and std protection can progress beyond C3 and eventually solve C5.
