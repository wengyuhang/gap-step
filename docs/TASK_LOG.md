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
