# Cold-start sensitivity

Same recorded startup heads, same objective/constraints and uncompiled normal
L-BFGS-B stops; not timing measurements or executed candidate comparisons.
Kinetic seed uses travel_speed=limit/2, crossing_speed=0.75*limit, zero acceleration.
It is compared to the original cold initializer (0.45*limit crossing speed).
All eight candidates passed residual/event acceptance; no runtime selection used.

| Recorded head from | Seed | First gate/s | Second gate/s | Terminal/s | Objective | Evaluations |
|---|---|---:|---:|---:|---:|---:|
| racing_seed | original | 3.237576 | 7.146187 | 11.327159 | 11.347431 | 300 |
| racing_seed | kinetic | 2.840939 | 8.153560 | 11.344616 | 12.165413 | 628 |
| racing_seed_o2 | original | 3.767683 | 7.593135 | 11.834858 | 11.838581 | 346 |
| racing_seed_o2 | kinetic | 2.829598 | 7.530336 | 10.787032 | 11.615763 | 323 |
| racing_seed_buffer | original | 3.276606 | 7.539593 | 11.753880 | 12.148424 | 126 |
| racing_seed_buffer | kinetic | 2.835346 | 7.447470 | 10.874336 | 11.455664 | 646 |
| racing_seed_epoch | original | 4.228338 | 8.590493 | 13.250834 | 13.200447 | 308 |
| racing_seed_epoch | kinetic | 2.835038 | 7.442679 | 10.874815 | 11.431295 | 734 |

This supports testing a stronger cold velocity guess, but not claiming it
uniformly minimizes even the local objective (first head worsens). The final
trial uses one fixed initialization rule, not best-of-two optimization. Execution
speed and acceleration limits, cost weights, convergence tolerances, safety and
online deadline thresholds remain unchanged. It is a heuristic initialization,
not a theoretical all-state performance guarantee.
