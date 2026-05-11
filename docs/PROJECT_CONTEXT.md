# Project Context

## Summary

GAP-Step is a continuous 2D rotating time-varying window maze project. The current implementation trains a PPO privileged teacher in procedurally generated ordinary mazes. Each episode first samples a randomized grid-maze topology, then converts it into continuous horizontal/vertical walls with time-varying windows that open, close, and rotate.

## Scope

Included:

- continuous square mazes generated from randomized grid topology
- 2D double-integrator robot dynamics
- time-varying window width and rotation safety checks
- low-dimensional privileged observation with 32 ray distances
- PPO actor-critic teacher training
- ID, OOD-size, and OOD-dynamics evaluation
- rollout GIF visualization

Excluded:

- visual student policies
- behavior cloning and demonstration datasets
- heuristic teacher demonstrations
- SITT proxy-student machinery
- world models or future video prediction
- active camera control
- full 3D quadrotor physics

## Observation Contract

The teacher observation is:

```text
[self_features, goal_features, ray_features]
```

Dimensions:

```text
4 + 3 + 32 = 39
```

`N_ray = 32` is fixed. The ray maximum distance is not a fixed constant:

```text
ray_max_dist = 0.35 * S
```

where `S` is the current episode's sampled maze side length.

## Expected Outputs

- `checkpoints/teacher_final.pt`
- `results/train_metrics.csv`
- `results/eval_metrics.csv`
- `results/typical_success.gif`
- `results/typical_wait.gif`
- `results/typical_collision.gif`

Generated outputs are ignored by Git.
