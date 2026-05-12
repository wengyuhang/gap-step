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
