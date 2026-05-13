# Architecture

## Flow

```text
gap_step/configs/train_teacher_*.yaml
    -> python -m gap_step.train
        -> gap_step.curriculum.sample_maze
        -> gap_step.env.ContinuousMazeEnv
        -> gap_step.graph.GraphObs / collate_graph_obs
        -> gap_step.model.GNNTeacherActorCritic
        -> checkpoints/teacher_final.pt + checkpoints/teacher_best.pt + results/train_metrics.csv

checkpoints/teacher_best.pt
    -> python -m gap_step.evaluate
    -> results/eval_metrics.csv

checkpoints/teacher_best.pt
    -> python -m gap_step.visualize
    -> results/*.gif
```

## Modules

- `gap_step/curriculum.py` defines C1, C1_5, C2A, C2B, C3, C4, C5 procedural maze generation, time-varying gate parameters, and ID/OOD test distributions.
- `gap_step/env.py` implements the Gymnasium-style continuous maze, collision checks, graph privileged observation, reward, and RGB rendering.
- `gap_step/graph.py` defines `GraphObs`, `GraphBatch`, feature dimensions, and graph collation.
- `gap_step/model.py` defines the pure PyTorch GNN actor-critic with tanh-squashed Gaussian actions.
- `gap_step/ppo.py` handles graph rollout collection, graph minibatch collation, GAE, and PPO updates with target-KL early stopping.
- `gap_step/train.py` trains the teacher across the adaptive curriculum and saves final/best checkpoints.
- `gap_step/evaluate.py` evaluates ID, OOD-size, OOD-dynamics, and optional stage-wise splits.
- `gap_step/visualize.py` saves typical GIF rollouts.

## Environment

Each episode samples a square maze with side length `S`. `curriculum.py` builds a randomized DFS grid topology, adds a few extra openings for loops, and converts grid edges into continuous horizontal/vertical walls with time-varying windows. A window is traversable only when both width and angle are safe.

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

Training configs additionally enable continuous-geometry progress shaping. The potential uses visibility over continuous wall rectangles and gate approach points; it is not a planner used by the policy and it does not change observations, collision rules, or success criteria.

## Graph Observation

The teacher sees full privileged simulator topology:

- `global_features`: robot state, goal relation, maze scale, time phase, stage progress, gate counts
- `cell nodes`: normalized cell center, start/goal/agent flags, relation to agent and goal
- `gate nodes`: gate center, orientation, width, clearance, safety, timing, and dynamics parameters
- `cell-cell edges`: adjacent topology with wall/open/gate type and gate timing when applicable
- `gate-cell edges`: each gate connected to the two cells it separates
- self-loops

The GNN consumes variable-size graphs directly through batching; there is no padding limit and no local ray observation in the teacher contract.

## Training Curriculum

Full teacher training uses adaptive progression:

```text
C1   static always-safe gate
C1_5 dynamic width gate with high open duty cycle
C2A  single dynamic gate that may require waiting
C2B  small maze with 1-2 dynamic gates
C3   adds rotating gates
C4   medium multi-gate maze
C5   final asynchronous multi-gate maze
```

Promotion requires recent stochastic rollout success, deterministic train-validation success, and minimum stage steps. `teacher_best.pt` is updated from deterministic validation, while `teacher_final.pt` stores the final training state.

## Code Boundary

All active code is directly under `gap_step/`. Legacy folders such as `trainers/`, `scripts/`, `gap_step/envs/`, `gap_step/models/`, and `gap_step/teachers/` are intentionally outside the active architecture.
