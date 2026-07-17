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

## TOGT Extension Track

- Keep `togt_timevarying_window/` as an independent TOGT reproduction extension, not as a wrapper around the old maze environment.
- DynaTOGT is now implemented: dynamic/deformable windows `G_i(t)`, discrete warm start, L-BFGS-B continuous refinement, Hermite continuous trajectory, and Chinese presentation-style visualization.
- Ordered/static modes now accept arbitrary traversal task sequences, including repeated windows; `shuffled_dynamic` remains a one-pass permutation baseline.
- Current experiment outputs live under `togt_timevarying_window/results/`, not the old `outputs/` directory.
- Next useful extensions:
  - add a MINCO-style trajectory backend for closer comparison with the original TOGT paper;
  - add stronger quadrotor thrust/angular-rate feasibility metrics;
  - add collision/obstacle constraints beyond window traversal;
  - add publication-style result tables comparing `WaypointCenter`, `StaticTOGT`, `DiscreteDynamic`, and `DynaTOGT`.

## Non-Convex Time-Varying Window Track

- Keep `nonconvex_timevarying_window/` as the umbrella for the research problem, with the common problem definition at its root.
- Keep every solution in a sibling directory named after the algorithm; do not place method-specific source files at the umbrella root.
- Two independent methods are implemented: `atlas_dynatogt/` uses ear-clipped chart atlases, while `sc_dynatogt/` uses Chang-style boundary resampling, Clipper2 inset geometry, disk Schwarz--Christoffel parameterization, and degree-7 MINCO.
- The current task covers simple closed non-convex windows without holes or self-intersections, dynamic translation/rotation/scale, and one traversal of each window in the specified order.
- Baseline comparison and repeated traversal are not current requirements.
- SC-DynaTOGT's paper-inspired six-window closed loop and E0--E5 default experiments are complete. Its results are losslessly categorized with run manifests and a Chinese result homepage. The Matplotlib layer now provides a low-clutter route overview, fixed-scale crossing grid, and scale profile; the optional EGL/OpenGL renderer replays the same solved trajectory.
- A fixed-distance OpenGL next-gate video with a live scale readout remains optional future work. Static fixed-scale comparison and numerical scale evidence are already implemented.
- Future work can add other methods as new sibling directories and evaluate them independently under their own `results/` directories.
