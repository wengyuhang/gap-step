# Project Context

## Summary

GAP-Step is a continuous 2D rotating time-varying window maze project. The current implementation trains a PPO privileged teacher with a pure PyTorch GNN actor-critic. The environment remains continuous: the robot state, acceleration actions, walls, window slots, collision checks, and success checks are all continuous geometry.

The teacher observation is now privileged topology state rather than local ray state. Each step returns a graph containing cell nodes, gate nodes, topology edges, gate-cell edges, and self-loops. The graph exposes simulator-only structure such as wall/open/gate edge type and gate dynamics. The policy still outputs continuous acceleration directly and does not run planning at inference time.

## Scope

Included:

- continuous square mazes generated from randomized grid topology
- 2D double-integrator robot dynamics
- time-varying window width and rotation safety checks
- graph privileged observation with full topology and gate timing/dynamics
- pure PyTorch GNN PPO actor-critic teacher training
- adaptive curriculum training across C1, C1_5, C2A, C2B, C3, C4, C5
- ID, OOD-size, OOD-dynamics, and stage-wise evaluation
- rollout GIF visualization

Excluded:

- visual student policies
- behavior cloning and demonstration datasets
- heuristic teacher demonstrations
- A*/MPC/waypoint execution
- SITT proxy-student machinery
- world models or future video prediction
- active camera control
- full 3D quadrotor physics

## Observation Contract

The teacher observation is:

```text
GraphObs(
  global_features: [16],
  node_features: [num_nodes, 32],
  node_type: [num_nodes],
  edge_index: [2, num_edges],
  edge_features: [num_edges, 20],
)
```

Nodes:

- cell nodes for every generated maze cell
- gate nodes for every time-varying window

Edges:

- directed cell-cell edges for adjacent cells, labeled as wall/open/gate
- directed gate-cell edges connecting each gate to its two neighboring cells
- self-loops for every node

The graph is batched by `gap_step.graph.collate_graph_obs`, which concatenates nodes/edges and offsets edge indices. There is no fixed padding limit and no local ray prefix in the teacher contract.

## Current Training Status

The previous local teacher reached C1 but failed to learn stable deterministic C2 behavior. The project has therefore moved to a full privileged GNN teacher. The first validation target is C2A/C2B deterministic success, then C3-C5.

Generated outputs:

- `checkpoints/teacher_final.pt`
- `checkpoints/teacher_best.pt`
- `results/train_metrics.csv`
- `results/eval_metrics.csv`
- `results/typical_success.gif`
- `results/typical_wait.gif`
- `results/typical_collision.gif`

Generated outputs are ignored by Git.
