# Roadmap

## Current MVP

- continuous 2D procedural maze from randomized grid topology
- rotating time-varying windows
- fixed 39D privileged ray observation
- PPO teacher actor-critic with tanh-squashed Gaussian actions
- adaptive curriculum C1 -> C5 with success-rate promotion and hard stop
- ID, OOD-size, OOD-dynamics, and stage-wise evaluation
- GIF rollout visualization

## Near-Term Improvements

1. Run the new adaptive full training and compare stage promotion, hard-stop behavior, and final ID/OOD success rates.
2. Use the new diagnostics to decide whether to tune reward scale, gate wait cost, or curriculum thresholds.
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
