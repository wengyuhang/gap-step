# Decisions

## 2026-05-10: Keep GAP-Step as a 2D MVP

Decision:

Use a 2D double-integrator robot and time-varying wall windows instead of a full 3D quadrotor simulator.

Reasoning:

The project goal is to validate gate choice, crossing timing, visual BC, auxiliary prediction, and PPO fine-tuning. Full 3D dynamics would add simulator and control complexity before the core learning question is answered.

## 2026-05-10: Use heuristic privileged teacher

Decision:

Implement a deterministic heuristic teacher with access to current true state and current gate widths/safety flags.

Reasoning:

The teacher is meant to provide clean demonstrations, not solve future prediction. It must not see future gate states, so the student experiment remains focused on current visual inference and timing.

## 2026-05-10: Keep PPO continuous-action only

Decision:

PPO controls only the continuous 2D acceleration action. The gate head remains supervised and diagnostic.

Reasoning:

A full hybrid discrete-continuous PPO implementation would be heavier than needed for the MVP. The environment is controlled by acceleration, while gate choice can still be evaluated through the classification head and trajectory behavior.

## 2026-05-10: Ignore generated artifacts in Git

Decision:

Exclude `data/`, `checkpoints/`, `logs/`, and `runs/` from source control.

Reasoning:

These files can be large and are reproducible from scripts and configs. Keeping them out of Git makes the repository easier to clone and review.

## 2026-05-10: Prefer config-driven experiments

Decision:

Use YAML files under `configs/` for environment and training parameters.

Reasoning:

This makes E1/E2/E3 experiments easier to reproduce and reduces hidden hard-coded settings in scripts.

