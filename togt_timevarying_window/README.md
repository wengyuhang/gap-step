# DynaTOGT: Dynamic Time-Varying Window Traversal

This subproject is a standalone research prototype built from the TOGT paper idea: a drone should traverse the geometry of each gate, not merely pass through gate centers. It extends the static TOGT constraint

```text
p(t_i) in G_i
```

to dynamic, deformable windows:

```text
ordered_dynamic:  p(t_i) in G_i(t_i)
shuffled_dynamic: p(t_k) in G_sigma(k)(t_k)
```

`gap_step/` PPO code is not used or modified.

## Algorithm

The new algorithm is **DynaTOGT**. It is not a full quadrotor MINCO/L-BFGS reproduction; it is a general dynamic-window TOGT research prototype:

- window-local traversal variables are mapped directly into `G_i(t_i)`, so geometric feasibility is strict;
- a discrete search builds a warm start over traversal time and traversal point;
- SciPy L-BFGS-B optimizes segment durations and local traversal-point variables;
- a piecewise Hermite polynomial trajectory is sampled for position, velocity, acceleration, jerk, and yaw;
- objective = total time + path length + acceleration/jerk penalties + feasibility penalties.

The default demo is intentionally slowed down for readability: canonical windows use enlarged motion amplitudes, the drone speed limit is conservative, and exported GIFs use more frames with a slower playback duration.

Default specified order:

```text
G1 -> G6 -> G3 -> G2 -> G5 -> G4
```

For `static` and `ordered_dynamic`, `--order` is a traversal task sequence, not a permutation constraint. A window may appear multiple times and the sequence can omit other windows:

```bash
python -m togt_timevarying_window.export_demo \
  --scenario canonical \
  --mode ordered_dynamic \
  --order G1,G6,G1,G3,G2,G5,G4,G2 \
  --outdir togt_timevarying_window/results/repeated_demo
```

`shuffled_dynamic` remains an automatic one-pass permutation search baseline.

Default window shapes:

```text
G1 rectangle
G2 circle
G3 pentagon
G4 slanted quadrilateral
G5 hexagon
G6 triangle
```

## Commands

Single demo:

```bash
python -m togt_timevarying_window.demo --scenario canonical --mode ordered_dynamic
python -m togt_timevarying_window.demo --scenario canonical --mode shuffled_dynamic
```

Export one demo:

```bash
python -m togt_timevarying_window.export_demo --scenario canonical --mode ordered_dynamic
```

Run experiments:

```bash
python -m togt_timevarying_window.experiments --suite smoke --outdir togt_timevarying_window/results
python -m togt_timevarying_window.experiments --suite default --outdir togt_timevarying_window/results
```

## Outputs

```text
togt_timevarying_window/results/<suite>/summary.csv
togt_timevarying_window/results/<suite>/trajectories/*.csv
togt_timevarying_window/results/<suite>/figures/*.png
togt_timevarying_window/results/<suite>/gifs/*.gif
```

Trajectory CSV files contain crossing points and dense drone samples with position, velocity, acceleration, and yaw. PNG/GIF files show the drone traversing dynamic windows while the windows translate, rotate, and scale.

For traversal evidence, each `crossing` row also records:

```text
local_u, local_v, plane_error, gate_margin, contains
```

`contains=True`, near-zero `plane_error`, and positive `gate_margin` are the numerical proof that the drone is inside the active dynamic window at that crossing time. GIF exports also insert explicit crossing-time frames labeled `PASS Gx` with the measured margin.

## Experiments

The default suite evaluates:

- `canonical_6`
- motion ablations: translation-only, rotation-only, scale-only, combined
- speed sweep: slow and fast dynamic windows
- 10 fixed-seed random non-collinear 3D tracks

Baselines:

- `WaypointCenter`
- `StaticTOGT`
- `DiscreteDynamic`
- `DynaTOGT`

Metrics:

- success
- duration
- path length
- total cost
- minimum dynamic gate margin
- max speed
- max acceleration
- mean jerk
- optimization time

## Tests

```bash
pytest -q togt_timevarying_window/tests
python -m py_compile togt_timevarying_window/*.py
```
