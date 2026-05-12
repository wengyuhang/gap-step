# AGENTS.md

This file gives coding agents the project-specific context needed to work on GAP-Step.

## Project Goal

GAP-Step is a continuous 2D rotating time-varying window maze project with mixed horizontal and vertical wall segments. The current stage trains only a PPO privileged teacher policy with low-dimensional observations:

- normalized robot position and velocity
- normalized relative goal features
- 32 ray distances against the current traversable geometry

The project intentionally excludes visual students, behavior cloning, heuristic demonstrations, full SITT, world models, future video prediction, active perception, 3D simulators, and quadrotor dynamics.

## Repository Layout

All runnable project code lives directly under `gap_step/`:

- `gap_step/env.py`: environment, collision, raycast observation, rendering
- `gap_step/curriculum.py`: C1-C5 procedural maze generation
- `gap_step/model.py`: PPO teacher actor-critic
- `gap_step/ppo.py`: rollout and PPO update logic
- `gap_step/train.py`: training entrypoint
- `gap_step/evaluate.py`: evaluation entrypoint
- `gap_step/visualize.py`: GIF visualization entrypoint
- `gap_step/utils.py`: shared utilities
- `gap_step/configs/`: teacher training YAML configs
- `gap_step/tests/`: focused curriculum, environment, and model tests
- `docs/`: persistent project context

Do not reintroduce `trainers/`, `scripts/`, `gap_step/envs/`, `gap_step/models/`, or `gap_step/teachers/` as active code paths.

## Working Rules

- Prefer small, runnable changes over broad rewrites unless the user explicitly asks for a full refactor.
- At the start of non-trivial tasks, inspect `docs/` and read the relevant project notes.
- Keep configuration in YAML when possible.
- Preserve the v4.6 observation contract: `N_ray = 32`, `ray_max_dist = 0.35 * S`, `obs_dim = 39`.
- Do not add 3D simulator dependencies unless explicitly requested.
- Do not implement students, BC, SITT, future video prediction, world models, or active camera control in this stage.
- Preserve generated-output ignores for `data/`, `checkpoints/`, `logs/`, `runs/`, and `results/`.
- Avoid committing large artifacts such as `.pt`, `.npz`, logs, or GIF/video rollouts unless the user explicitly asks.
- Use the `wyh` conda environment when running tests locally.
- In non-interactive shells, initialize conda before activation: `source /home/jack/anaconda3/etc/profile.d/conda.sh && conda activate wyh`.
- Run `pytest -q` after behavior changes when feasible.

## Common Commands

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh
conda activate wyh
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
python -m gap_step.evaluate --checkpoint checkpoints/teacher_final.pt --episodes 5
python -m gap_step.visualize --checkpoint checkpoints/teacher_final.pt
```

## Git Notes

The current remote is expected to use the SSH alias:

```bash
git@github-wengyuhang:wengyuhang/gap-step.git
```
