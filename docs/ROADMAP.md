# Roadmap

## Current MVP

- continuous 2D procedural maze from randomized grid topology
- rotating time-varying windows
- 161D time-aware privileged teacher observation with a stable 39D ray prefix
- PPO teacher actor-critic with tanh-squashed Gaussian actions
- adaptive curriculum C1 -> C5 with rollout + deterministic validation promotion and hard stop
- ID, OOD-size, OOD-dynamics, and stage-wise evaluation
- GIF rollout visualization

## Near-Term Improvements

1. Run the new 161D time-aware adaptive full training and compare stage promotion, hard-stop behavior, and final ID/OOD success rates.
2. Inspect whether deterministic promotion and std protection prevent C3 collapse/early-stage forgetting.
3. Add vectorized environments for faster teacher training once reward/curriculum behavior is more trustworthy.
4. Improve typical GIF selection by searching for actual success/wait/collision cases.
5. Save exact config snapshots next to the teacher checkpoint.
6. Add more collision geometry tests for edge contact and high-speed crossing.

## Out of Scope For This Stage

- visual student policies
- behavior cloning
- demonstration datasets
- SITT
- world models
- future video prediction
- active perception
- 3D quadrotor simulation
