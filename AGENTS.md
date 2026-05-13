# AGENTS.md

Project-specific context for agents working on GAP-Step.

## Goal

GAP-Step trains a PPO privileged teacher in a continuous 2D rotating time-varying window maze. Current scope is teacher-only RL with a pure PyTorch GNN actor-critic that observes the full simulator topology graph and gate dynamics.

Out of scope for this stage: visual students, BC, heuristic demos, SITT, world models, future video prediction, active perception, 3D simulators, and quadrotor dynamics.

## Code Layout

Runnable code lives directly under `gap_step/`:

- `env.py`: continuous maze, collision, graph observation, reward, rendering
- `graph.py`: `GraphObs`, graph batching, feature-dimension constants
- `curriculum.py`: C1-C5 procedural maze generation plus C1_5/C2A/C2B bridge stages
- `model.py`: pure PyTorch GNN tanh-squashed Gaussian PPO teacher actor-critic
- `ppo.py`: graph rollout, graph minibatch collation, GAE, PPO update
- `train.py`: fixed/adaptive curriculum training entrypoint
- `evaluate.py`: ID/OOD and stage-wise evaluation
- `visualize.py`: GIF rollout visualization

Do not reintroduce `trainers/`, `scripts/`, `gap_step/envs/`, `gap_step/models/`, or `gap_step/teachers/` as active paths.

## Rules

- Read relevant `docs/` before non-trivial work.
- The teacher observation contract is graph-based: `GraphObs(global_features, node_features, node_type, edge_index, edge_features)`.
- The graph contains cell nodes, gate nodes, directed cell-cell topology edges, directed gate-cell edges, and self-loops.
- GNN observation is privileged simulator state. It may include full topology, wall/open/gate edge type, gate timing, gate dynamics, and current gate safety.
- Do not use A*, MPC, waypoint following, expert demonstrations, BC, or heuristic action labels for teacher policy execution.
- PPO actions use tanh-squashed Gaussian log probabilities matching executed actions; do not return to `Normal -> clamp(action)`.
- Progress reward must use continuous geometry, not cells; compare old/new positions at the same time and clip delta to avoid time-passing reward leakage.
- Do not change collision rules, success criteria, or maze generation semantics unless explicitly asked.
- Preserve generated-output ignores for `data/`, `checkpoints/`, `logs/`, `runs/`, and `results/`; avoid committing large artifacts unless asked.
- Use `wyh` conda env. In non-interactive shells: `source /home/jack/anaconda3/etc/profile.d/conda.sh && conda activate wyh`.
- Run `pytest -q` after behavior changes when feasible.

## Commands

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh && conda activate wyh
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
python -m gap_step.evaluate --checkpoint checkpoints/teacher_best.pt --episodes 20 --stages C1,C1_5,C2A,C2B,C3,C4,C5
python -m gap_step.evaluate --checkpoint checkpoints/teacher_best.pt
```
