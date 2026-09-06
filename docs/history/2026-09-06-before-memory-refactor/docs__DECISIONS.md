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

## 2026-07-17: Keep Cinematic Rendering Separate From Planning

Decision: keep SC-DynaTOGT's EGL/OpenGL renderer optional and downstream of the saved scenario and degree-7 MINCO trajectory.  It may add physical meshes, lighting, environment assets, HUD, and cameras, but it must not change optimization geometry or be described as AirSim dynamics/sensor simulation.

Reason: the renderer is intended to communicate the already-solved experiment more realistically.  A separate layer preserves numerical reproducibility and prevents presentation effects from being mistaken for planning or feasibility logic.

## 2026-07-17: Report Scale State Separately From Its Visual Salience

Decision: document that the OpenGL window transform applies the true `s(t)R(t)` on every frame and that the current diverse demo uses `s(t) in [0.58,1.42]`.  Also report that chase-camera perspective makes this change hard to see.  A fixed-distance `GATE CAM` with a live `SCALE ×` readout is a future visualization improvement, not a completed feature.

Reason: geometric correctness and ease of visual comparison are different claims.  Keeping them separate avoids both falsely diagnosing a missing scale transform and overstating what the current video communicates.

## 2026-07-17: Organize Results Without Deleting History

Decision: categorize SC-DynaTOGT results into experiments, demos, diagnostics, and work areas. Every migration must be dry-run first, journal every file's old/new path, size, and SHA-256, and preserve old visual variants under `legacy/`. New runs use timestamped directories and manifests; the result homepage is generated from saved summaries rather than hand-maintained values.

Reason: formal experiments, intermediate chunks, historical demos, and presentation renders have different roles. Separating them makes the current result easy to find without sacrificing reproducibility or prior data.
