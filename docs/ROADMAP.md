# Roadmap

## Done

- Replaced the dynamic-cell passage abstraction with generated aperture-window mazes.
- Added continuous collision-safe window geometry and preview assets.
- Built pure PPO training/evaluation/visualization flow.
- Reached the C5 ID target: `71.5%` over 200 unseen episodes.
- Produced ID/OOD evaluation and GIF artifacts.

## Current State

```text
id_test         71.5%
ood_window_test 54.0%
ood_maze_test   74.5%
```

## Next

- Improve `ood_window_test` generalization to unseen aperture timing.
- Consider curriculum slices focused on timing variation before increasing maze complexity.
- Keep wall/window collision semantics unchanged while tuning.
