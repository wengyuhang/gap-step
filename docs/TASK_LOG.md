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

## 2026-07-17 SC-DynaTOGT Paper-Inspired Irregular Closed-Loop Demo

Implemented:

- based the new default `paper_irregular` layout on Gate1,2,3,4,6,7 from the original companion track `race_uzh_7g_multiprisma.yaml`, including its irregular positions, RPY values, and traversal pattern;
- scaled the layout to centre bounds `x=[-9.9,20.24] m`, `y=[-13.2,14.96] m`, and `z=[1.8,6.48] m`, with unequal route legs of `14.77--30.74 m`;
- set identical start and goal `[-16,4,3.2] m` while retaining the paper-style zig-zag order rather than a regular geometric loop;
- increased the default motion amplitude multiplier from `2.5` to `3.5`, giving a uniform scale range of `[0.58,1.42]`;
- added explicit `start`/`goal` overrides to the general boundary-scenario builder and closed-loop metadata to `summary.json`;
- updated static/GIF visualization to use one `start = goal` marker;
- retained `spacious` and `compact` as legacy layouts and separated all output directories so new runs do not overwrite prior experiments;
- preserved the regular closed-loop intermediate result under `results/diverse_closed_loop_regular_20260717/` and restored the prior open result under `results/diverse_demo/`.

Validation:

```text
pytest -q nonconvex_timevarying_window/sc_dynatogt/tests
93 passed in 30.83s

python -m nonconvex_timevarying_window.sc_dynatogt.diverse_demo \
  --mode full --quality smoke --layout paper_irregular --motion-scale 3.5 \
  --validation-samples 1000 \
  --outdir nonconvex_timevarying_window/sc_dynatogt/results/diverse_paper_irregular_closed
passed=true, closed_loop=true, 6/6 legal crossings
six SC maps: 1000/1000 legal each
total_time=14.9048121206 s, iterations=385, invalid_trial_count=0
sampled_dynamic_limits_satisfied=false
```

## 2026-07-17 SC-DynaTOGT Physical-Scene Visualization

Implemented:

- removed inset safe polygons from `trajectory.png` and `dynamic_windows.gif`; preprocessing diagnostics still show them;
- render every original time-varying boundary with one orange/graphite tubular racing-gate style, while preserving each actual non-convex outline;
- replaced point/diamond vehicle markers with an X-frame quadrotor, four rotor disks, a body, and a nose direction;
- derive quadrotor heading from MINCO velocity and tilt from acceleration plus gravity;
- draw quadrotors at all traversal instants in the static figure and animate one vehicle along the trajectory;
- added gate-order badges and a subtle ground reference while removing per-shape color differences and dense diagnostic labels;
- changed the default demo output to `results/diverse_paper_irregular_closed_physical_scene/` so the earlier irregular-loop artifacts remain untouched.

Preview artifacts:

```text
nonconvex_timevarying_window/sc_dynatogt/results/diverse_paper_irregular_closed_physical_scene/trajectory.png
nonconvex_timevarying_window/sc_dynatogt/results/diverse_paper_irregular_closed_physical_scene/dynamic_windows.gif
```

Validation:

```text
pytest -q nonconvex_timevarying_window/sc_dynatogt/tests
95 passed in 31.24s
python -m compileall -q nonconvex_timevarying_window/sc_dynatogt
passed
```

## 2026-07-17 SC-DynaTOGT AirSim-Style Offline Renderer

Implemented:

- added the optional `simulation_render.py` EGL/OpenGL path without changing the optimizer or Matplotlib diagnostics;
- construct physical tube meshes from the actual non-convex boundaries and update their translation, rotation, and uniform scale at every frame;
- added a volumetric quadrotor with frame, motors, propellers, canopy, nose, and navigation lights;
- map MINCO velocity/acceleration to vehicle heading and tilt, and use a smoothed third-person chase camera;
- added roads, grass, varied buildings with glass facades, trees, direct-light shadows, gradient sky, sun glow, atmospheric distance fog, and telemetry HUD;
- added `requirements-render.txt`, a saved-summary CLI, dense-boundary display reduction with sharp-corner retention, and an independent output directory;
- documented explicitly that this is trajectory-accurate offline rendering rather than AirSim physics/sensor simulation.

Artifacts and validation:

```text
airsim_overview.png: 960 x 540
airsim_chase.png:    960 x 540
airsim_chase.mp4:    H.264/yuv420p, 960 x 540, 144 frames, 12 fps, 12.0 s
MP4 decode check:    frames 0, 72, and 143 valid
pytest -q nonconvex_timevarying_window/sc_dynatogt/tests
104 passed in 31.18s
python -m compileall -q nonconvex_timevarying_window/sc_dynatogt
passed
```

## 2026-07-17 SC-DynaTOGT Window-Scale Visibility Diagnosis

Diagnosed without changing source code or experiment artifacts:

- confirmed that `simulation_render._window_pose` places `s(t)R(t)` in each gate node's three-dimensional transform and that every video frame updates this pose;
- confirmed the six-window demo uses scale amplitude `0.42`, so every window spans `[0.58,1.42]` over its motion cycle;
- measured traversal-time scales `L=1.419`, `U=0.874`, `star=0.716`, `limacon=1.378`, `wavy=1.153`, and `line_bezier=0.611`;
- identified chase-camera distance, perspective, simultaneous rotation, and different native gate shapes/sizes as the reasons the scale change is difficult to compare visually;
- retained fixed-distance `GATE CAM` and a live `SCALE ×` readout as an unimplemented follow-up, not as current output behavior.

## 2026-07-17 SC-DynaTOGT Lossless Result Organization

Implemented:

- added `results_manager.py` with dry-run-by-default migration, resumable journals, per-file byte counts/SHA-256, run manifests, `current_demo.json`, a machine-readable catalog, Markdown index, and Chinese HTML result homepage;
- moved 543 existing files (60,079,505 bytes) into `experiments/`, `demos/`, `diagnostics/`, and `work/` without copying, deleting, or overwriting any historical artifact;
- merged the selected paper-irregular closed loop into one run while retaining its original plots, physical-scene render, and old OpenGL output under `legacy/`;
- added a low-clutter route overview with one representative quadrotor, a 2-by-3 fixed-world-scale crossing grid, and a full six-gate scale profile;
- changed future experiment/demo defaults to timestamped categorized directories while preserving explicit `--outdir` support;
- changed the OpenGL default to follow `current_demo.json`, write into the selected run's `opengl/`, and refresh the run manifest/result homepage after rendering;
- renamed current OpenGL outputs to `opengl_overview.png`, `opengl_chase.png`, and `opengl_chase.mp4`, leaving the previous files intact below `legacy/original_opengl/`;
- muted environment materials in the OpenGL overview so the route and physical gates carry more visual weight.

Validation:

```text
migration dry-run: 543 files, 60,079,505 bytes, no destination conflicts
migration verify:  543/543 sizes and SHA-256 hashes matched
catalog:            9 runs, featured=20260717_paper_irregular_closed
OpenGL MP4:         H.264/yuv420p, 960 x 540, 144 frames, 12 fps, 12.0 s
MP4 decode check:   frames 0, 72, and 143 valid
pytest -q nonconvex_timevarying_window/sc_dynatogt/tests
111 passed in 31.49s
```
