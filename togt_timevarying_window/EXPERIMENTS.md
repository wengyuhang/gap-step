# DynaTOGT Experiments

## Suites

`smoke` is a fast integration suite for CI-style checks:

```text
canonical track x WaypointCenter / StaticTOGT / DynaTOGT
```

`default` is the main evaluation suite:

```text
canonical
translation_only
rotation_only
scale_only
slow_dynamic
fast_dynamic
random_0 ... random_9
```

Each track is evaluated with:

```text
WaypointCenter
StaticTOGT
DiscreteDynamic
DynaTOGT
```

## Commands

```bash
python -m togt_timevarying_window.experiments --suite smoke --outdir togt_timevarying_window/results
python -m togt_timevarying_window.experiments --suite default --outdir togt_timevarying_window/results
```

## Output Layout

```text
results/<suite>/summary.csv
results/<suite>/trajectories/*.csv
results/<suite>/figures/*.png
results/<suite>/gifs/*.gif
```

Only DynaTOGT figures/GIFs are generated for every default scenario to keep output size controlled. The smoke suite exports all baseline visuals.

## Metrics

`summary.csv` columns:

```text
scenario
baseline
mode
success
order
duration
path_length
total_cost
min_gate_margin
max_speed
max_acceleration
mean_jerk
optimization_time
```

## Expected Evidence

The implementation is considered valid when:

- DynaTOGT succeeds on `canonical`;
- DynaTOGT succeeds on dynamic ablation scenes;
- StaticTOGT fails or has worse dynamic margins in at least some dynamic scenes;
- DynaTOGT improves cost or path length over WaypointCenter in at least one scene;
- CSV/PNG/GIF artifacts are generated.
