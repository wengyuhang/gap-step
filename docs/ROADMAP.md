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

1. Improve PPO stability and diagnostics.
2. Add vectorized environments for faster teacher training.
3. Improve typical GIF selection by searching for actual success/wait/collision cases.
4. Save exact config snapshots next to the teacher checkpoint.
5. Add more collision geometry tests for edge contact and high-speed crossing.

## Out of Scope For This Stage

- visual student policies
- behavior cloning
- demonstration datasets
- SITT
- world models
- future video prediction
- active perception
- 3D quadrotor simulation
