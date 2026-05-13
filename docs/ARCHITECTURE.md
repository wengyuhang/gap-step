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
- `gap_step/env.py` implements the Gymnasium-style continuous maze, collision checks, 161D time-aware privileged observation, reward, and RGB rendering.
- `gap_step/model.py` defines the MLP actor-critic teacher with tanh-squashed Gaussian actions.
- `gap_step/ppo.py` handles rollout collection, GAE, and PPO updates.
- `gap_step/train.py` trains the teacher across C1-C5. Full training uses adaptive curriculum promotion by recent success rate; fixed stage stepping remains available for compatibility.
- `gap_step/evaluate.py` evaluates ID, OOD-size, OOD-dynamics, and optional stage-wise splits.
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

Strict v4.6 sparse rewards remain the environment default. Teacher training configs additionally enable a continuous-geometry progress shaping term:

```text
reward_progress * clip(potential(old_pos, current_t) - potential(new_pos, current_t))
```

This term estimates remaining time to the goal on a visibility roadmap built from continuous wall rectangles, gate approach points, and future gate safety checks. Window crossing cost includes estimated waiting time for a future safe opening. The same-time old/new comparison prevents pure time passing near a future-opening gate from becoming a large dense reward. The shaping path model is only a reward guide; it does not change observations, collision rules, or success criteria.

Teacher observations keep the original 39D state/goal/ray prefix and append global time phase plus fixed-length summaries for up to 10 gates. Gate summaries expose privileged timing and safety features so the policy can learn wait/cross/bypass behavior instead of inferring future windows from current rays alone.

## Training Curriculum

Full teacher training uses adaptive C1-C5 progression:

```text
promotion_success_rate = 0.70
promotion_eval_success_rate = 0.60
promotion_window_episodes = 100
min_steps_per_stage = 500_000
soft_max_steps_per_stage = 5_000_000
hard_max_steps_per_stage = 10_000_000
```

Stages promote only after the recent episode window reaches the rollout success threshold, deterministic train-validation reaches its threshold, and the minimum stage steps have elapsed. Soft max emits a diagnostic warning; hard max stops training and saves the current checkpoint/metrics.

## Code Boundary

All active code is directly under `gap_step/`. Legacy folders such as `trainers/`, `scripts/`, `gap_step/envs/`, `gap_step/models/`, and `gap_step/teachers/` are intentionally removed from the active architecture.
