# Planar translation--rotation ablation

## Protocol

```json
{
  "courses": 7,
  "seed": 20260911,
  "repeats": 1,
  "population": 64,
  "maximum_rounds": 50,
  "repair_max_iterations": 600,
  "workers": 4,
  "audit_step_s": 0.001,
  "fixed_opening_physical_size_m": [
    3.3,
    2.6
  ],
  "fixed_start_goal_distance_m": 36.0,
  "motion_sweep_fixed_window_count": 5,
  "count_sweep_fixed_motion_scale": 1.0,
  "motion_levels": [
    [
      "static",
      0.0
    ],
    [
      "low",
      0.5
    ],
    [
      "medium",
      1.0
    ],
    [
      "high",
      1.5
    ]
  ],
  "window_count_levels": [
    3,
    5,
    7
  ],
  "wall_seconds": 100.51970910798991
}
```

All openings are identical 3.30 m × 2.60 m physical rectangles. Windows remain at z=3.2 m; they only translate horizontally in y and yaw about the vertical axis. `no_audited_solution_under_budget` means no candidate passed the independent 1 ms audits within this generous finite budget; it is not an infeasibility proof.

| Study | Factor | Method | Audited feasible | No audited output | Median plan (s) | Median feasible flight time (s) |
|---|---|---|---:|---:|---:|---:|
| motion | static | TOGT | 0/1 | 0/1 | 15.917 | — |
| motion | static | Conditional Dual-Constraint CEM | 1/1 | 0/1 | 39.041 | 5.182 |
| motion | low | TOGT | 0/1 | 0/1 | 17.679 | — |
| motion | low | Conditional Dual-Constraint CEM | 1/1 | 0/1 | 39.875 | 4.966 |
| motion | medium | TOGT | 0/1 | 0/1 | 22.111 | — |
| motion | medium | Conditional Dual-Constraint CEM | 1/1 | 0/1 | 44.025 | 4.832 |
| motion | high | TOGT | 0/1 | 0/1 | 14.746 | — |
| motion | high | Conditional Dual-Constraint CEM | 1/1 | 0/1 | 42.825 | 5.200 |
| count | 3 | TOGT | 0/1 | 0/1 | 0.820 | — |
| count | 3 | Conditional Dual-Constraint CEM | 1/1 | 0/1 | 18.971 | 4.007 |
| count | 5 | TOGT | 0/1 | 0/1 | 13.538 | — |
| count | 5 | Conditional Dual-Constraint CEM | 1/1 | 0/1 | 40.517 | 4.731 |
| count | 7 | TOGT | 0/1 | 0/1 | 8.248 | — |
| count | 7 | Conditional Dual-Constraint CEM | 1/1 | 0/1 | 45.300 | 6.436 |
