# TOGT vs Conditional Dual-Constraint CEM benchmark

## Frozen protocol

```json
{
  "courses": 4,
  "difficulty_levels": [
    {
      "name": "easy",
      "aperture_scale": 1.25,
      "motion_amplitude_scale": 0.55,
      "motion_rate_scale": 0.7
    },
    {
      "name": "moderate",
      "aperture_scale": 1.16,
      "motion_amplitude_scale": 0.8,
      "motion_rate_scale": 0.9
    },
    {
      "name": "hard",
      "aperture_scale": 1.08,
      "motion_amplitude_scale": 1.1,
      "motion_rate_scale": 1.15
    },
    {
      "name": "extreme",
      "aperture_scale": 1.0,
      "motion_amplitude_scale": 1.4,
      "motion_rate_scale": 1.4
    }
  ],
  "course_generator_seed": 20260911,
  "population": 16,
  "maximum_rounds": 10,
  "repair_max_iterations": 120,
  "final_audit_maximum_step_seconds": 0.001,
  "methods": [
    "TOGT",
    "Conditional Dual-Constraint CEM"
  ],
  "wall_seconds": 480.2829308299697
}
```

## Aggregate results

| Difficulty | Method | Joint feasible | Collision rate | Median plan time (s) | Median min margin (m) | Median feasible flight time (s) |
|---|---|---:|---:|---:|---:|---:|
| easy | TOGT | 0/1 | 1.000 | 1.855 | -0.2166 | — |
| easy | Conditional Dual-Constraint CEM | 1/1 | 0.000 | 69.498 | 0.0372 | 24.314 |
| moderate | TOGT | 0/1 | 1.000 | 0.862 | -0.2339 | — |
| moderate | Conditional Dual-Constraint CEM | 1/1 | 0.000 | 73.916 | 0.0549 | 20.850 |
| hard | TOGT | 0/1 | 1.000 | 0.470 | -0.3321 | — |
| hard | Conditional Dual-Constraint CEM | 1/1 | 0.000 | 63.666 | 0.0096 | 21.542 |
| extreme | TOGT | 0/1 | 1.000 | 0.645 | -0.2996 | — |
| extreme | Conditional Dual-Constraint CEM | 1/1 | 0.000 | 72.741 | 0.0044 | 42.829 |

All methods use the same known-window model, `[K,D]` parameterization, native TOGT dynamics, and maximum 1 ms independent audits. Safety/dynamics are dense numerical evidence, not continuous certificates. Failures are retained in the raw CSV and rate plots.
