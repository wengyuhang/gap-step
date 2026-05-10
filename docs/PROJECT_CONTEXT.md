# Project Context

## Summary

GAP-Step is a minimal research prototype for studying dynamic multi-window crossing with visual policies. The environment is a 2D workspace with a wall at the center. One or two windows open and close over time, and a circular robot must cross through a currently safe window to reach a target on the right side.

The core research question is whether a student that sees only rendered grayscale image stacks plus proprioception can learn the teacher's gate choice and crossing timing.

## Scope

Included:

- 2D double-integrator robot dynamics
- time-varying gate widths
- privileged heuristic teacher
- visual student policy with CNN image encoder
- behavior cloning with optional auxiliary heads
- compact continuous-action PPO fine-tuning
- evaluation metrics and rollout visualization

Excluded from this MVP:

- full 3D quadrotor physics
- full SITT proxy-student mechanism
- world model or future video prediction
- active camera control
- photorealistic rendering

## Main Hypothesis

Auxiliary prediction of current gate width and safety can make visual behavior cloning more stable and interpretable, and PPO initialized from BC+Aux can preserve or improve task performance.

## Expected Outputs

The scripts produce:

- demonstration datasets under `data/`
- checkpoints under `checkpoints/`
- TensorBoard logs under `runs/`
- evaluation tables under `logs/`
- rollout GIFs under `runs/`

These outputs are reproducible and intentionally ignored by Git.

## Baseline Tasks

- E1: single window, fixed open
- E2: single window, periodic width
- E3: two windows, asynchronous periodic widths

