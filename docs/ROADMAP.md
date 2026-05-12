# Roadmap

## Current MVP

- continuous 2D procedural maze from randomized grid topology
- rotating time-varying windows
- fixed 39D privileged ray observation
- PPO teacher actor-critic
- curriculum C1 -> C5
- ID, OOD-size, and OOD-dynamics evaluation
- GIF rollout visualization

## Near-Term Improvements

1. Debug the dynamic progress reward after the first full C1-C5 run: reward scale, potential clipping, gate wait cost, and whether collision/timeout terms dominate learning.
2. Add better stage-wise training diagnostics, including progress reward statistics and gate-use/wait-time summaries.
3. Smooth curriculum progression or add stage-specific evaluation so C4/C5 failures can be isolated earlier.
4. Add vectorized environments for faster teacher training once reward/curriculum behavior is more trustworthy.
5. Improve typical GIF selection by searching for actual success/wait/collision cases.
6. Save exact config snapshots next to the teacher checkpoint.
7. Add more collision geometry tests for edge contact and high-speed crossing.

## Out of Scope For This Stage

- visual student policies
- behavior cloning
- demonstration datasets
- SITT
- world models
- future video prediction
- active perception
- 3D quadrotor simulation
