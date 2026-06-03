# TOGT Reproduction Audit

## Scope

User request: reproduce `复现/论文/2309.06837v3.pdf` / TOGT-Planner, create reproduction work under `复现/`, create the improved algorithm as a new top-level project, record both in Markdown, and sync project documents. The improvement must use the paper-style environment rather than the older maze environment.

## Evidence

| Requirement | Evidence | Status |
| --- | --- | --- |
| Paper/source reproduction folder under `复现/` | `复现/TOGT-Planner-reproduction/source/`, `REPRODUCTION.md` | Done |
| Source-level reproduction notes | `复现/TOGT-Planner-reproduction/REPRODUCTION.md` | Done |
| Reproduction dependency check | `check_reproduction.py` reports source tree ok, cmake/c++ present, and local vendored Eigen found; `BUILD_DEPS.md` records build commands | Done |
| Result-level TOGT reproduction | `analyze_trajectory.py`: `lap_time=8.21`, `path_length=83.189`, `max_speed=19.321`; `plot_togt_traj.py` generated PNG | Done |
| Improved algorithm in top-level folder | `togt_timevarying_window/` | Done |
| Improvement uses paper-style environment | `environment.py` defines ordered `RaceTrack` and dynamic `DynamicGate`; no runtime imports from `gap_step` | Done |
| Window/gate 3D position, pose, and shape vary over time | `DynamicGate.center_at`, `yaw_at`, `pitch_at`, `roll_at`, `scale_at`, `polygon_at`; tests verify geometry changes | Done |
| Planner for 3D dynamic gates | `planner.py` searches over gate order, arrival time, and gate-interior 3D candidates | Done |
| Improvement records | `togt_timevarying_window/README.md` | Done |
| Visual/export artifacts | 12-gate complex 3D `togt_timevarying_window/outputs/*_trajectory.csv/png/gif` | Done |
| Tests | `pytest -q togt_timevarying_window/tests` -> `3 passed` | Done |
| Project docs synced | `README.md`, `docs/ARCHITECTURE.md`, `DECISIONS.md`, `PROJECT_CONTEXT.md`, `ROADMAP.md`, `TASK_LOG.md` | Done |

## Native Build Result

Full native C++ execution of TOGT-Planner is completed with a local vendored Eigen install. Verified output:

```text
source_tree=ok
cmake=/usr/bin/cmake
cxx=/usr/bin/c++
eigen_cmake=复现/TOGT-Planner-reproduction/deps/eigen-install/share/eigen3/cmake/Eigen3Config.cmake
eigen_header=复现/TOGT-Planner-reproduction/deps/eigen-install/include/eigen3/Eigen/Core
```

`ctest --test-dir build --output-on-failure` reports `100% tests passed, 0 tests failed out of 3`.
