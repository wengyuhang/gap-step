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
