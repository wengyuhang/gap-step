# Roadmap

## Current MVP

- 2D dynamic gate crossing environment
- heuristic privileged teacher
- visual student policy
- BC-only and BC+Aux training
- PPO fine-tuning from BC+Aux
- CSV evaluation and GIF visualization
- basic tests for dynamics, environment, and teacher

## Near-Term Improvements

1. Improve PPO diagnostics
   - log policy entropy, value loss, KL proxy, and success curves more explicitly
   - add early stopping when deterministic evaluation degrades

2. Make evaluation more reproducible
   - save exact config snapshots next to each checkpoint
   - record seed lists used for evaluation
   - add a single command for full E1/E2/E3 experiment runs

3. Strengthen visual learning checks
   - add ablations for image-only, proprio-only, and image+proprio inputs
   - add confusion matrix for gate choice
   - save example auxiliary predictions over rollout time

4. Improve rendering artifacts
   - annotate selected gate and unsafe gate attempts in rollout GIFs
   - add side-by-side teacher vs student rollout rendering

## Medium-Term Research Extensions

- Add randomized wall and target layouts.
- Add observation noise and actuation noise.
- Add curriculum learning from E1 to E3.
- Add recurrent student policy for longer timing memory.
- Add domain randomization for gate appearance.

## Out of Scope Unless Requested

- full 3D quadrotor simulation
- SITT proxy-student implementation
- world model training
- future frame prediction
- active camera control

