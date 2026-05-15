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
