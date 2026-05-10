# AGENTS.md

This file gives coding agents the project-specific context needed to work on GAP-Step without rediscovering the same constraints.

## Project Goal

GAP-Step is a minimal 2D / 2.5D research project for dynamic gate crossing. It tests whether a visual student policy can learn gate selection and timing from:

- privileged heuristic teacher demonstrations
- visual behavior cloning
- auxiliary current gate-width and safety prediction
- compact PPO fine-tuning

The project intentionally excludes full 3D quadrotor dynamics, full SITT proxy-student machinery, world models, future video prediction, and active camera control.

## Repository Layout

- `gap_step/envs/`: 2D environment, gate dynamics, renderer
- `gap_step/teachers/`: heuristic privileged teacher
- `gap_step/models/`: CNN encoder and student policy
- `trainers/`: demo generation, BC training, PPO fine-tuning, evaluation
- `scripts/`: rollout visualization scripts
- `configs/`: environment and training YAML configs
- `tests/`: focused environment, dynamics, and teacher tests
- `docs/`: project context, architecture, roadmap, decisions, and task log

## Working Rules

- Prefer small, runnable changes over broad rewrites.
- Keep configuration in YAML when possible.
- Do not add 3D simulator dependencies unless explicitly requested.
- Do not implement full SITT, world models, future video prediction, or active camera control in this MVP.
- Preserve generated-output ignores for `data/`, `checkpoints/`, `logs/`, and `runs/`.
- Avoid committing large artifacts such as `.pt`, `.npz`, TensorBoard logs, or GIF/video rollouts unless the user explicitly asks.
- Use the existing conda workflow from `README.md` and `environment.yml`.
- Run `pytest` after behavior changes when feasible.

## Common Commands

```bash
conda activate isaac
pytest
python scripts/render_random_policy.py
python trainers/generate_demos.py
python trainers/train_bc.py
python trainers/train_ppo.py
python trainers/evaluate.py
```

## Git Notes

The current remote is expected to use the SSH alias:

```bash
git@github-wengyuhang:wengyuhang/gap-step.git
```

The project has generated experiment outputs locally, but those paths are ignored by Git.

