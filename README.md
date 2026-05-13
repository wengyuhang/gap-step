# GAP-Step

GAP-Step is a compact continuous 2D maze project for training a PPO privileged teacher. The agent is a circular robot with double-integrator dynamics. Each episode samples a randomized grid-maze topology, converts it into continuous horizontal/vertical wall segments, and inserts time-varying windows whose usable state depends on both opening width and rotation angle.

The current teacher is a pure PyTorch GNN actor-critic. It observes full privileged topology and gate dynamics as a graph, while still outputting continuous acceleration actions directly. It does not run A*, MPC, waypoint following, demonstrations, BC, visual students, SITT, world models, active camera control, or 3D quadrotor dynamics.

## Code Layout

All runnable project code lives directly under `gap_step/`:

- `gap_step/env.py`: continuous maze environment, graph observation, collision, rendering
- `gap_step/graph.py`: `GraphObs`, `GraphBatch`, and graph collation
- `gap_step/curriculum.py`: online C1/C1_5/C2A/C2B/C3/C4/C5 procedural maze generator
- `gap_step/model.py`: GNN Gaussian actor-critic teacher
- `gap_step/ppo.py`: graph rollout collection and PPO update
- `gap_step/train.py`: teacher training entrypoint
- `gap_step/evaluate.py`: ID/OOD evaluation entrypoint
- `gap_step/visualize.py`: typical rollout GIF generation
- `gap_step/configs/`: teacher training YAML configs
- `gap_step/tests/`: curriculum, environment, graph, and model tests

## Environment

Use the existing conda environment:

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh
conda activate wyh
```

Or create a fresh environment from `environment.yml`.

## Commands

Smoke test:

```bash
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
python -m gap_step.evaluate --checkpoint checkpoints/teacher_best.pt --episodes 5
python -m gap_step.visualize --checkpoint checkpoints/teacher_best.pt
```

Full training:

```bash
python -m gap_step.train --config gap_step/configs/train_teacher_full.yaml
python -m gap_step.evaluate --checkpoint checkpoints/teacher_best.pt
python -m gap_step.visualize --checkpoint checkpoints/teacher_best.pt
```

Stage-wise evaluation:

```bash
python -m gap_step.evaluate --checkpoint checkpoints/teacher_best.pt --episodes 20 --stages C1,C1_5,C2A,C2B,C3,C4,C5
```

## Outputs

- `checkpoints/teacher_final.pt`
- `checkpoints/teacher_best.pt`
- `results/train_metrics.csv`
- `results/eval_metrics.csv`
- `results/typical_success.gif`
- `results/typical_wait.gif`
- `results/typical_collision.gif`

Generated artifacts under `data/`, `checkpoints/`, `logs/`, `runs/`, and `results/` are ignored by Git.

## Observation

The privileged teacher observation is variable-size graph state:

```text
GraphObs(
  global_features: [16],
  node_features: [num_nodes, 32],
  node_type: [num_nodes],
  edge_index: [2, num_edges],
  edge_features: [num_edges, 20],
)
```

The graph contains cell nodes, gate nodes, directed topology edges, directed gate-cell edges, and self-loops. The environment dynamics and collision checks remain continuous.
