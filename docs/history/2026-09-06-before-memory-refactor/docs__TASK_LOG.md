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

## 2026-07-21 MSR-DynaTOGT Multi-Start And Feasibility Repair

Implemented a new independent sibling method under `nonconvex_timevarying_window/msr_dynatogt/` without modifying SC-DynaTOGT, AtlasDynaTOGT, or their historical results:

- added reproducible SC-center, random spatial/temporal, turn-aware, and dispersed-region initializations;
- ran the stable SC-DynaTOGT `x=[K,D]` degree-7 MINCO optimizer for every start and retained a deduplicated candidate pool;
- enforced candidate ordering by designated-window legality, then high-density sampled dynamics, then total flight time, so shorter rotor-limit violations never outrank feasible candidates;
- sampled velocity, collective thrust, body rates, all four rotor thrusts, prescribed crossing order, true non-convex containment, and boundary margin;
- added uniform and local time dilation, iterative expansion, binary search, `T -> K` conversion, complete joint reoptimization, revalidation, and feasible-incumbent preservation;
- separated overall sampled-feasibility `success`, raw/reoptimization convergence, and final-result source;
- implemented A0 original SC, A1 SC + multistart, A2 SC + repair, and A3 full MSR, plus matched-start-count and measured matched-wall-time comparisons;
- added five prescribed scenes, timestamp-only result directories, complete CSV/JSON candidate records, Chinese `REPORT.md`, five trajectory comparisons, and aggregate time/runtime/feasibility/thrust figures;
- documented AtlasDynaTOGT only as an auxiliary structural comparison because its Hermite trajectory, anisotropic scale, dynamics, and metrics are not directly comparable.

Validation:

```text
python -m compileall -q nonconvex_timevarying_window/msr_dynatogt
passed
pytest -q nonconvex_timevarying_window/msr_dynatogt/tests
8 passed in 34.75s
pytest -q nonconvex_timevarying_window/sc_dynatogt/tests
111 passed in 31.89s
python -m nonconvex_timevarying_window.msr_dynatogt.experiments --suite smoke
completed: 5 scenes, 1 seed, 60 A0--A3/protocol rows, 378.643 s
result: nonconvex_timevarying_window/msr_dynatogt/results/20260721_132527_458246_smoke/
```

Native smoke summary:

```text
method  window legal  sampled dynamics feasible  mean total time  mean wall time
A0      5/5           0/5                        6.161245 s       7.161 s
A1      5/5           1/5                        5.970820 s      31.954 s
A2      5/5           5/5                        6.226261 s      18.864 s
A3      5/5           5/5                        6.023721 s      75.148 s
```

A3 improved sampled-dynamics feasibility by 100 percentage points over A0 and used 10.49x mean wall time. A0 was dynamically infeasible in every smoke scene, so its shorter/longer time differences are not feasible-solution superiority claims. All feasibility statements are high-density sampled only, not continuous-time certificates. The smoke optimizer cap is 24 iterations; final repaired incumbents were retained after the required reoptimization was run and rechecked, and no optimizer success was fabricated.

## 2026-07-22 MSR-DynaTOGT Full Formal Experiment

Completed the full prescribed suite without reducing seeds, starts, sampling density, or optimizer stopping criteria:

- ran 5 scenes x 155 seeds = 775 independent tasks with 7 starts and 129 high-density samples per segment;
- saved 775 candidate JSON files and 9,300 A0--A3 rows spanning native, matched-start, and matched-time protocols;
- used non-overwriting task checkpoints and resumed the same configuration after correcting worker CPU affinity; completed tasks were neither rerun nor overwritten;
- recorded 100% window legality for all methods and sampled-dynamics feasibility of 0.0%/0.1%/100%/100% for A0/A1/A2/A3;
- measured mean flight times of 5.377927/5.351334/5.389145/5.366490 s and mean per-run wall times of 237.436/1482.787/303.535/2385.702 s;
- reduced A3 mean peak rotor thrust from 5.017345 N before repair to 4.999816 N after repair with mean time scale 1.002526;
- generated five representative trajectory comparisons, four aggregate figures, a concise Chinese report, and a separate Chinese explanation for every figure;
- grouped failed seed ranges in the report while retaining every individual failure row in summary.json and runs.csv;
- kept AtlasDynaTOGT as a structural auxiliary note only because its trajectory and dynamics interfaces are not directly comparable.

Formal result:

```text
nonconvex_timevarying_window/msr_dynatogt/results/20260721_135308_842525_formal/
775/775 tasks, 9,300 rows, 93,509.732 s active final-session wall time
A0/A1/A2/A3 sampled feasible: 0/1/775/775
```

Interpretation: repair is responsible for the robust feasibility gain. A3 is 0.022655 s faster on average than feasible A2, with the clearest gains on the six-window loop and thrust-stressing scene, but costs 7.86x A2 wall time. Matched-start and matched-time comparisons select identical A2/A3 results, so the multistart gain is conditional on extra compute budget. All feasibility claims remain high-density sampled rather than continuous-time certificates.

## 2026-07-24 Closed-Loop Deformable Window FAPP-PPO

Implemented the first reinforcement-learning algorithm in the independent
`closed_loop_deformable_window/fapp_ppo/` method directory:

- generated every window's opening schedule, pose motion, and local deformation from independent per-window random streams fixed at reset;
- removed the old arrival-aligned opening construction, so route length, cruise speed, UAV state, and actions cannot trigger a window opening;
- modeled the local shape with a 64-point positive radial graph, five harmonic deformation coefficients, two independent axis scales, and a smooth opening envelope;
- interpolated centers, rotation vectors, and ordered boundary points with natural cubic splines and validated nonzero, simple, connected, hole-free physical polygons;
- allowed the true non-convex safe inward offset to become empty while the physical opening remains nonzero;
- used independent non-periodic renewal schedules with smoothstep opening/closing transitions;
- added Chinese training-reward, algorithm, window-model, experiment-protocol, and pilot-result figures;
- exported and frame-checked a Chinese H.264 MP4 demonstrating a completely closed target, UAV waiting, four legal crossings, and complete-state return;
- ran a 100-update/102,400-step validation training and paired 10-seed ID/tight pilot.

Independent-window audit:

```text
1,000 seeds, pairwise first-opening correlation: -0.034..0.014
first-opening range: 0.321..4.118 s
opportunities per window: 6..8
ID passable fraction: 45.86%..56.35%
ID non-passable fraction: 43.65%..54.14%
```

Training result and diagnosis:

```text
final FAPP-PPO ID:             0/10
Nominal-Reactive ID:           1/10
Nominal-Schedule ID:           1/10
update 25 development success: 3/10
updates 50/75/100:             0/10
maximum approximate KL:        0.0108 < target 0.02
```

The main non-convergence cause is a reward-credit defect: crossing a gate switches
the potential target to the next distant gate, introducing a `-6.5..-9.8` shaping
jump that nearly cancels the `+10` gate bonus. Persistent exploration standard
deviation near 0.30 and weak residual anchoring then cause late residual-policy
collapse. The early-checkpoint success MP4 is documented as a mechanism
demonstration only and is not substituted for the final-checkpoint result.

Validation:

```text
pytest -q closed_loop_deformable_window/fapp_ppo/tests
12 passed
pytest -q
46 passed
```

## 2026-08-02 CWB-SC-DynaTOGT Continuous Whole-Body Safety V1

Implemented a new independent sibling method under
`nonconvex_timevarying_window/cwb_sc_dynatogt/` without modifying any existing
algorithm directory:

- retained the original SC-DynaTOGT decision vector `x=[K,D]` and constant yaw;
- modeled the vehicle as an attitude-aware cuboid whose roll/pitch are recovered
  from the current MINCO trajectory by the existing differential-flatness code;
- computed all eight vertex `xi3` plane coordinates with the required extrema
  contact/section logic rather than a multi-input XOR;
- selected only the cuboid/plane intersection component containing each planned
  traversal time and represented section topology by source body-edge IDs;
- constructed the complete 3--6 vertex plane section and checked every section
  edge over adaptive time/lambda cells with the SC radial margin;
- separated numerical verification, explicit unsafe witnesses, uncertified cells,
  and numerical failures, and prevented V1 from emitting `CERTIFIED`;
- added segment-relative witnesses, stale-witness handling, finite active safety
  penalties, warm-started outer optimization, diagnostics, and a smoke CLI;
- documented two deliberate V1 limitations: sampled rather than interval-rigorous
  bounds, and a discrete auxiliary SC preimage pool/finite-difference gradient
  instead of the planned continuous `[K,D,U]` augmented-Lagrangian/autodiff V2.

Validation:

```text
pytest -q nonconvex_timevarying_window/cwb_sc_dynatogt/tests
12 passed in 0.78s
pytest -q nonconvex_timevarying_window/sc_dynatogt/tests
111 passed in 31.17s
python -m compileall -q nonconvex_timevarying_window/cwb_sc_dynatogt
passed
pytest -q
46 passed in 3.04s
python -m nonconvex_timevarying_window.cwb_sc_dynatogt.experiments --suite smoke --outdir /tmp/cwb_sc_smoke
completed; explicit UNSAFE after the configured three-round outer budget, certified=false
```
