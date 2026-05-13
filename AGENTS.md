# AGENTS.md

Project-specific context for agents working on GAP-Step.

## Goal

GAP-Step trains a PPO privileged teacher in a continuous 2D rotating time-varying window maze. Current scope is teacher-only RL with low-dimensional observations: normalized robot state, relative goal features, and 32 ray distances.

Out of scope for this stage: visual students, BC, heuristic demos, SITT, world models, future video prediction, active perception, 3D simulators, and quadrotor dynamics.

## Code Layout

Runnable code lives directly under `gap_step/`:

- `env.py`: continuous maze, collision, ray observation, reward, rendering
- `curriculum.py`: C1-C5 procedural maze generation
- `model.py`: tanh-squashed Gaussian PPO teacher actor-critic
- `ppo.py`: rollout, GAE, PPO update
- `train.py`: fixed/adaptive curriculum training entrypoint
- `evaluate.py`: ID/OOD and stage-wise evaluation
- `visualize.py`: GIF rollout visualization

Do not reintroduce `trainers/`, `scripts/`, `gap_step/envs/`, `gap_step/models/`, or `gap_step/teachers/` as active paths.

## Rules

- Read relevant `docs/` before non-trivial work.
- Preserve observation contract: `N_ray = 32`, `ray_max_dist = 0.35 * S`, `obs_dim = 39`.
- Keep config in YAML when possible.
- Full training defaults to adaptive C1-C5 curriculum: 70% recent success over 100 episodes, 500k min steps, 2M soft warning, 5M hard stop.
- PPO actions use tanh-squashed Gaussian log probabilities matching executed actions; do not return to `Normal -> clamp(action)`.
- Progress reward must use continuous geometry, not cells; compare old/new positions at the same time and clip delta to avoid time-passing reward leakage.
- Do not change collision rules, success criteria, or maze generation unless explicitly asked.
- Preserve generated-output ignores for `data/`, `checkpoints/`, `logs/`, `runs/`, and `results/`; avoid committing large artifacts unless asked.
- Use `wyh` conda env. In non-interactive shells: `source /home/jack/anaconda3/etc/profile.d/conda.sh && conda activate wyh`.
- Run `pytest -q` after behavior changes when feasible.

## Commands

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh && conda activate wyh
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
python -m gap_step.evaluate --checkpoint checkpoints/teacher_final.pt --episodes 20 --stages C1,C2,C3,C4,C5
python -m gap_step.evaluate --checkpoint checkpoints/teacher_final.pt
```
