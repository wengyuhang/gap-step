# Task Log

## 2026-05-11

Refactored GAP-Step to PPO teacher only:

- moved active code into flat `gap_step/` modules
- removed old visual student, BC, heuristic teacher, `trainers/`, and `scripts/` paths
- added randomized grid-maze curriculum C1-C5 with continuous walls and time-varying windows
- added PPO teacher training, evaluation, and visualization entrypoints
- updated tests for curriculum, environment, and actor-critic model

Validation:

- `pytest -q`
- `python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml`

## 2026-05-12

Implemented continuous-geometry dynamic progress shaping:

- added optional `dynamic_geometry` progress shaping in `ContinuousMazeEnv`
- kept sparse rewards as the environment default
- enabled dynamic progress shaping in teacher smoke/full configs
- required reward potentials to use continuous geometry, not topology cells
- added diagnostics for progress reward, gate usage/wait time, collision type, action norm, and curriculum status

Validation:

- `pytest -q`
- full training/evaluation ran end to end, but final ID/OOD success stayed weak

Open issue:

- Shaping alone did not produce a strong teacher.

## 2026-05-13 Earlier Iteration

Tried a time-aware local privileged vector teacher:

- added gate timing summaries to the local teacher observation
- added log-std protection
- required deterministic validation before stage promotion

Result:

- The teacher learned some C1 behavior but failed to produce stable deterministic C2 behavior.
- Current checkpoint evaluation showed C1 around 70% on a small deterministic sample and C2-C5 at 0%.

Conclusion:

- The teacher was still not privileged enough; it lacked full topology and full gate dynamics context.

## 2026-05-13 GNN Privileged Teacher Refactor

Implemented the new mainline teacher:

- added `gap_step/graph.py` with `GraphObs`, `GraphBatch`, feature dimensions, and graph collation
- replaced teacher observation with graph privileged topology state
- added cell nodes, gate nodes, directed cell-cell edges, directed gate-cell edges, and self-loops
- replaced the MLP actor-critic with pure PyTorch GNN message passing
- updated PPO rollout/minibatch update to collate variable-size graph observations
- added target-KL early stopping diagnostics
- added `teacher_best.pt` checkpointing
- split curriculum into `C1`, `C1_5`, `C2A`, `C2B`, `C3`, `C4`, `C5`
- updated smoke/full configs for the GNN teacher
- updated tests for graph observations, GNN forward/action/log-prob, and graph PPO updates

Validation:

- `pytest -q`: 25 passed
- `python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml`: completed
- `python -m gap_step.evaluate --checkpoint checkpoints/teacher_best.pt --episodes 2 --stages C1,C1_5,C2A,C2B,C3,C4,C5 --output /tmp/gap_gnn_eval_smoke.csv`: completed

Note:

- Smoke training only validates plumbing and overwrites ignored generated outputs. It does not indicate policy quality.
- Next meaningful experiment should train through C1/C1_5/C2A/C2B and check deterministic C2B success.
