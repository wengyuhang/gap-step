# Task Log

## 2026-05-15 Generated Window Maze Training

Implemented:

- procedural aperture-window maze family;
- continuous swept-circle collision;
- compact `GraphObs`;
- pure PPO curriculum training;
- ID/OOD evaluation;
- success/collision/timing/OOD GIFs.

Important adjustments during tuning:

- fixed long-episode rollout collection;
- removed the grid-like passage abstraction from the active line;
- added future aperture features to window nodes;
- changed window-near shaping to prioritize the live gap;
- calibrated C5 minimum gap from `0.65` to `0.72`.

Validation:

```text
pytest -q
45 passed
```

Final C5 summary:

```text
id_test         71.5%
ood_window_test 54.0%
ood_maze_test   74.5%
```

## 2026-06-01 TOGT Reproduction And Dynamic Gate Prototype

Implemented:

- added `复现/TOGT-Planner-reproduction/REPRODUCTION.md` for paper/source reproduction notes;
- replaced the initial maze-dependent prototype with an independent paper-style race track / ordered-gate environment;
- added dynamic gates/windows whose position, yaw, and shape scale vary with time;
- added a TOGT-style planner over `(gate order, arrival time, gate-interior candidate)`.

Validation:

```text
python -m togt_timevarying_window.demo
  track=paper_style_complex_dynamic_3d_gates dynamic=True lap_time=31.40 length=69.18 gates=12
python -m togt_timevarying_window.demo --static
  track=paper_style_complex_dynamic_3d_gates dynamic=False lap_time=28.60 length=66.04 gates=12
python -m py_compile togt_timevarying_window/*.py
  passed
pytest -q togt_timevarying_window/tests
  3 passed
python 复现/TOGT-Planner-reproduction/analyze_trajectory.py
  lap_time=8.21 path_length=83.189 max_speed=19.321
MPLBACKEND=Agg python plot_togt_traj.py
  generated source/scripts/plots/togt_traj.png
python -m togt_timevarying_window.export_demo
  generated outputs/dynamic_trajectory.csv, outputs/dynamic_trajectory.png, and outputs/dynamic_trajectory.gif
python -m togt_timevarying_window.export_demo --static
  generated outputs/static_trajectory.csv, outputs/static_trajectory.png, and outputs/static_trajectory.gif
```

Reproduction build note: `python 复现/TOGT-Planner-reproduction/check_reproduction.py` reports `source_tree=ok`, cmake/c++ present, and local `eigen_cmake` / `eigen_header` found; CMake/build/ctest now pass with local vendored Eigen.

Audit note: `docs/TOGT_REPRODUCTION_AUDIT.md` maps each requested TOGT reproduction/improvement deliverable to current evidence and records the resolved local Eigen dependency for native C++ execution.

Native TOGT build update: local Eigen 3.4.0 was installed under `复现/TOGT-Planner-reproduction/deps/eigen-install`; `cmake --build build -j2` completed and `ctest --test-dir build --output-on-failure` reported `100% tests passed, 0 tests failed out of 3`.

3D upgrade update: `togt_timevarying_window` now uses a nonlinearly arranged 12-gate 3D track with time-varying center, yaw/pitch/roll, and gate shape scale. The dynamic task is about 31.4s and `python -m togt_timevarying_window.export_demo` generates `dynamic_trajectory.gif` in addition to CSV/PNG.

## 2026-06-04 DynaTOGT Dynamic Time-Varying Window Rebuild

Implemented:

- rebuilt `togt_timevarying_window/` from the earlier discrete prototype into DynaTOGT;
- extended the TOGT paper constraint from static `p(t_i) in G_i` to dynamic/deformable `p(t_i) in G_i(t_i)`;
- added dynamic windows with translation, yaw/pitch/roll rotation, and anisotropic scale/shape changes;
- added arbitrary ordered traversal task sequences, including repeated windows such as `G1 -> G6 -> G1 -> G3 -> G2 -> G5 -> G4 -> G2`;
- added DynaTOGT warm start + L-BFGS-B continuous optimization over segment durations and window-local traversal variables;
- added Hermite continuous drone trajectory sampling with speed, acceleration, jerk, and yaw metrics;
- added baselines: `WaypointCenter`, `StaticTOGT`, `DiscreteDynamic`, and `DynaTOGT`;
- added `smoke` and `default` experiment suites under `togt_timevarying_window/results/`;
- replaced the old 3D debug visualization with Chinese presentation-style PNG/GIF showing `穿越成功`, `裕度`, current window pose, and past/future dashed poses;
- rewrote `togt_timevarying_window/README.md`, `ALGORITHM.md`, and `EXPERIMENTS.md` in Chinese with original TOGT comparison.

Current commands:

```bash
python -m togt_timevarying_window.demo --scenario canonical --mode ordered_dynamic
python -m togt_timevarying_window.export_demo --scenario canonical --mode ordered_dynamic
python -m togt_timevarying_window.export_demo --scenario canonical --mode ordered_dynamic --order G1,G6,G1,G3,G2,G5,G4,G2 --outdir togt_timevarying_window/results/repeated_demo
python -m togt_timevarying_window.experiments --suite smoke --outdir togt_timevarying_window/results
pytest -q togt_timevarying_window/tests
python -m py_compile togt_timevarying_window/*.py
```

Current validation:

```text
pytest -q togt_timevarying_window/tests
6 passed
python -m py_compile togt_timevarying_window/*.py
passed
```

Current demo artifacts:

```text
togt_timevarying_window/results/demo/trajectories/canonical_6_DynaTOGT.csv
togt_timevarying_window/results/demo/figures/canonical_6_DynaTOGT.png
togt_timevarying_window/results/demo/gifs/canonical_6_DynaTOGT.gif
togt_timevarying_window/results/repeated_demo/trajectories/canonical_6_DynaTOGT.csv
togt_timevarying_window/results/repeated_demo/figures/canonical_6_DynaTOGT.png
togt_timevarying_window/results/repeated_demo/gifs/canonical_6_DynaTOGT.gif
```

Traversal evidence is recorded in CSV crossing rows via `contains=True`, near-zero `plane_error`, and positive `gate_margin`.

## 2026-07-10 Non-Convex Time-Varying Window Research Organization

Implemented:

- defined the TOGT extension problem with non-convex, time-varying windows in `nonconvex_timevarying_window/PROBLEM_DEFINITION.md`;
- scoped the current task to simple closed regions without holes or self-intersections and one traversal of each window in the specified order;
- changed `nonconvex_timevarying_window/` into the umbrella directory for multiple future solution methods;
- moved the existing triangulation chart-atlas method into `nonconvex_timevarying_window/atlas_dynatogt/`;
- kept the method's source, algorithm document, figures, tests, and experiment results together under the algorithm directory;
- removed baseline comparison and demo entry points from the AtlasDynaTOGT experiment flow;
- updated package imports, CLI paths, documentation, and result paths for the nested method package.

Current commands:

```bash
python -m nonconvex_timevarying_window.atlas_dynatogt.experiments --suite smoke --outdir nonconvex_timevarying_window/atlas_dynatogt/results
python -m nonconvex_timevarying_window.atlas_dynatogt.experiments --suite default --outdir nonconvex_timevarying_window/atlas_dynatogt/results
python -m py_compile nonconvex_timevarying_window/atlas_dynatogt/*.py
pytest -q nonconvex_timevarying_window/atlas_dynatogt/tests
```

Current validation:

```text
default suite: 14 scenarios, 14 successes
pytest: 7 passed
```
