# Roadmap

## Current Mainline

- Continuous 2D time-varying window maze
- PPO privileged teacher only
- Pure PyTorch GNN actor-critic
- Graph observation with full topology and gate dynamics
- Adaptive curriculum: C1, C1_5, C2A, C2B, C3, C4, C5

## Near-Term Work

1. Run a focused GNN teacher training sweep through C2B.
2. Compare deterministic and stochastic success for C1, C1_5, C2A, and C2B.
3. Inspect wait/cross behavior near closed gates.
4. Tune entropy, target KL, GNN hidden size, and curriculum thresholds if C2A/C2B remain weak.
5. Only after C2B is stable, run C3-C5 full training.

## Later Work

- Improve graph pooling or add attention if C5 needs longer-range credit assignment.
- Add richer diagnostics for gate timing decisions.
- Consider a student policy only after the privileged teacher reliably solves C5.
