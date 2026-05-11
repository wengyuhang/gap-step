# Architecture

## Flow

```text
gap_step/configs/train_teacher_*.yaml
    -> python -m gap_step.train
        -> gap_step.curriculum.sample_maze
        -> gap_step.env.ContinuousMazeEnv
        -> gap_step.model.TeacherActorCritic
        -> checkpoints/teacher_final.pt + results/train_metrics.csv

checkpoints/teacher_final.pt
    -> python -m gap_step.evaluate
    -> results/eval_metrics.csv

checkpoints/teacher_final.pt
    -> python -m gap_step.visualize
    -> results/*.gif
```

## Modules

- `gap_step/curriculum.py` defines C1-C5 procedural maze generation, time-varying gate parameters, and ID/OOD test distributions.
- `gap_step/env.py` implements the Gymnasium-style continuous maze, collision checks, 39D privileged observation, reward, and RGB rendering.
- `gap_step/model.py` defines the MLP Gaussian actor-critic teacher.
- `gap_step/ppo.py` handles rollout collection, GAE, and PPO updates.
- `gap_step/train.py` trains the teacher across the fixed curriculum order.
- `gap_step/evaluate.py` evaluates ID, OOD-size, and OOD-dynamics splits.
- `gap_step/visualize.py` saves typical GIF rollouts.

## Environment

Each episode samples a square maze with side length `S`. `curriculum.py` builds a randomized DFS grid maze, adds a few extra openings for loops, and converts grid edges into continuous axis-aligned wall segments. Some passage edges are replaced by window slots. A window is traversable only when both width and angle are safe.

Collision types:

- `wall`
- `closed_gate`
- `boundary`

Rewards:

```text
+20.0 on goal
-20.0 on collision
-0.01 per step
-0.001 * ||action||^2
```

There is no progress reward, gate reward, waypoint reward, or path-following reward.

## Code Boundary

All active code is directly under `gap_step/`. Legacy folders such as `trainers/`, `scripts/`, `gap_step/envs/`, `gap_step/models/`, and `gap_step/teachers/` are intentionally removed from the active architecture.
