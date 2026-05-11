# GAP-Step

GAP-Step is now a compact continuous 2D maze project for training a PPO privileged teacher. The agent is a circular robot with double-integrator dynamics. Each episode samples a randomized grid-maze topology, converts it into continuous horizontal/vertical wall segments, and inserts time-varying windows whose usable state depends on both opening width and rotation angle.

This version does not include visual students, behavior cloning, heuristic teacher demonstrations, SITT, future prediction, world models, active camera control, or 3D quadrotor dynamics.

## Code Layout

All runnable project code lives directly under `gap_step/`:

- `gap_step/env.py`: continuous maze environment, ray observation, collision, rendering
- `gap_step/curriculum.py`: online C1-C5 procedural maze generator
- `gap_step/model.py`: MLP Gaussian actor-critic teacher
- `gap_step/ppo.py`: rollout collection and PPO update
- `gap_step/train.py`: teacher training entrypoint
- `gap_step/evaluate.py`: ID/OOD evaluation entrypoint
- `gap_step/visualize.py`: typical rollout GIF generation
- `gap_step/configs/`: teacher training YAML configs
- `gap_step/tests/`: curriculum, environment, and model tests

## Environment

Use the existing conda environment:

```bash
conda activate wyh
```

Or create a fresh environment from `environment.yml`.

## Commands

Smoke test:

```bash
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
python -m gap_step.evaluate --checkpoint checkpoints/teacher_final.pt --episodes 5
python -m gap_step.visualize --checkpoint checkpoints/teacher_final.pt
```

Full training:

```bash
python -m gap_step.train --config gap_step/configs/train_teacher_full.yaml
python -m gap_step.evaluate --checkpoint checkpoints/teacher_final.pt
python -m gap_step.visualize --checkpoint checkpoints/teacher_final.pt
```

## Outputs

- `checkpoints/teacher_final.pt`
- `results/train_metrics.csv`
- `results/eval_metrics.csv`
- `results/typical_success.gif`
- `results/typical_wait.gif`
- `results/typical_collision.gif`

Generated artifacts under `data/`, `checkpoints/`, `logs/`, `runs/`, and `results/` are ignored by Git.

## Observation

The privileged teacher observation is fixed-size:

```text
self_features + goal_features + ray_features
4 + 3 + 32 = 39 dimensions
```

`N_ray = 32` is fixed. The ray maximum distance scales with the current maze size:

```text
ray_max_dist = 0.35 * S
```
