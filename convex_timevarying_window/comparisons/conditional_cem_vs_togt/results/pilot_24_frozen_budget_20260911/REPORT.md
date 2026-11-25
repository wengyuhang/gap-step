# TOGT vs Conditional Dual-Constraint CEM benchmark

## Frozen protocol

```json
{
  "courses": 24,
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
  "workers": 4,
  "final_audit_maximum_step_seconds": 0.001,
  "methods": [
    "TOGT",
    "Conditional Dual-Constraint CEM"
  ],
  "wall_seconds": 774.4046543379663
}
```

## Aggregate results

| Difficulty | Method | Joint feasible | Dynamics pass | Safety pass | Collision rate | Median plan time (s) | Median min margin (m) | Median feasible flight time (s) |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| easy | TOGT | 0/6 | 0/6 | 0/6 | 1.000 | 14.204 | -0.2244 | — |
| easy | Conditional Dual-Constraint CEM | 5/6 | 5/6 | 5/6 | 0.167 | 94.945 | 0.0417 | 21.613 |
| moderate | TOGT | 0/6 | 0/6 | 0/6 | 1.000 | 1.683 | -0.2950 | — |
| moderate | Conditional Dual-Constraint CEM | 4/6 | 5/6 | 4/6 | 0.333 | 71.521 | 0.0199 | 23.581 |
| hard | TOGT | 0/6 | 0/6 | 0/6 | 1.000 | 2.157 | -0.3244 | — |
| hard | Conditional Dual-Constraint CEM | 5/6 | 5/6 | 5/6 | 0.167 | 73.715 | 0.0443 | 21.691 |
| extreme | TOGT | 0/6 | 0/6 | 0/6 | 1.000 | 2.521 | -0.3093 | — |
| extreme | Conditional Dual-Constraint CEM | 5/6 | 5/6 | 5/6 | 0.167 | 70.109 | 0.0274 | 22.156 |

There are 0 courses jointly feasible for both methods; only those courses support a fair paired flight-time comparison. The flight-time plot therefore marks all returned trajectories, with circles denoting jointly feasible outputs and crosses denoting failures.
All methods use the same known-window model, `[K,D]` parameterization, native TOGT dynamics, and maximum 1 ms independent audits. Safety/dynamics are dense numerical evidence, not continuous certificates. Failures are retained in the raw CSV and rate plots.
