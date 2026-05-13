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
- adaptive C1-C5 curriculum training
- ID, OOD-size, OOD-dynamics, and stage-wise evaluation
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

## Current Training Status

The project now has an end-to-end PPO teacher pipeline that can run the full C1-C5 curriculum and complete ID/OOD evaluation. The current dynamic-geometry reward shaping is a training aid, not a solved policy recipe.

Latest full-run result after enabling dynamic geometry shaping:

- ID success rate: 5.0%
- OOD-size success rate: 5.5%
- OOD-dynamics success rate: 5.0%

Follow-up changes have now addressed the first-order training signal issues: adaptive curriculum replaces fixed C1-C5 step switches, PPO actions use a squashed Gaussian instead of post-sampling clamp, and progress shaping no longer rewards pure time passing near future-opening gates. A new adaptive full run is needed before updating the success-rate baseline.
