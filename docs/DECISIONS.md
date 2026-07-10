# Decisions

## 2026-05-15: Use Generated Aperture-Window Mazes

Decision: the active task is a generated family of mazes, not a fixed benchmark image.

Reason: the target is a class of UAV window-crossing problems.

## 2026-05-15: Keep Continuous Geometry

Decision: use continuous 2D actions and swept-circle collision. Any black wall/window contact is terminal failure.

Reason: discrete cell transitions hid invalid wall/window crossings.

## 2026-05-15: Train Pure PPO Only

Decision: the teacher remains pure privileged PPO with curriculum learning.

Reason: the user explicitly rejected planner/BC/expert assistance for the current mainline.

## 2026-05-15: Calibrate C5 Geometry, Not Topology

Decision: keep C5 at full path length, six dynamic windows, and mixed geometry, but set the minimum nominal gap to `0.72`.

Reason: the previous `0.65` floor produced a stable `62.5%` ID plateau under pure PPO. Raising only the geometric clearance reached `71.5%` while preserving the high-difficulty structure.

## 2026-05-15: Report OOD Honestly

Decision: retain and report both OOD splits after ID acceptance.

Reason: `ood_window_test` remains materially weaker than ID, so it must be visible in the project record.

## 2026-06-01: Make TOGT Extension An Independent Paper-Style Project

Decision: TOGT reproduction lives under `复现/TOGT-Planner-reproduction/`, and the time-varying window adaptation lives under `togt_timevarying_window/`.

Reason: the requested reproduction/improvement should use the paper environment abstraction. The extension therefore models ordered gates `G_i(t)` with dynamic position and shape rather than reusing the earlier maze environment.

## 2026-07-10: Organize Non-Convex Research As A Multi-Method Project

Decision: use `nonconvex_timevarying_window/` as the umbrella directory for the non-convex time-varying window problem. Keep `PROBLEM_DEFINITION.md` at the umbrella root and place each solution in a separately named algorithm directory. The existing triangulation-chart method is stored under `atlas_dynatogt/`.

Reason: the non-convex problem will be studied with multiple methods. Separating the shared problem statement from method-specific code, tests, documentation, figures, and results prevents the first implementation from being mistaken for the whole research task.
