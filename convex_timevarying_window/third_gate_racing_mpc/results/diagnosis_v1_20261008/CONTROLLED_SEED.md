# Same helper, different time initialization

2026-10-08. Uncompiled fixed-state diagnostics; not executed flights. Both columns
use the same new generic seed helper (including evaluating the moving terminal at
the seed arrival time) and identical fresh crossing speed 5.4 m/s. Only the travel
speed used to initialize new durations differs. This isolates that parameter from
the original-vs-new initializer comparison in counterfactuals.json.

| Next gate (1-based) | Seed travel speed | Valid | First crossing | Second crossing | Objective |
|---|---:|---|---:|---:|---:|
| 2 | 6 | yes | 7.551660 | 10.368667 | 11.448317 |
| 2 | 12 | yes | 8.090427 | 11.519842 | 12.309386 |
| 3 | 6 | yes | 10.926777 | 16.199296 | 12.996273 |
| 3 | 12 | yes | 10.433978 | 13.268093 | 10.762485 |
| 4 | 6 | no | 16.198828 | 20.274433 | 8257082.098434 |
| 4 | 12 | yes | 16.091855 | 19.716747 | 12.948745 |
| 5 | 6 | yes | 21.530620 | 25.371683 | 12.164214 |
| 5 | 12 | yes | 20.664592 | 24.465035 | 11.397888 |

The timing initializer materially affects local solutions, including recovering
an earlier and lower-cost phase for the problematic third-to-fourth-gate leg.
It is not uniformly better at every head and does not prove global optimality.

First actual flight with the new initializer and original -O1 compilation:
23.448 s versus historical 24.428 s; seven exits, no contact, minimum sampled
clearance 0.455419 m. Eight solves all normal stops, seven admitted; one 107.023 ms
solve rejected at the unchanged 100 ms deadline. Strict online result is FAILURE,
not a valid faster lap. Full artifacts: ../racing_seed_v1_20261008/.
The next revision changes only compiler optimization (-O2, no fast-math); all
objective, solver convergence, safety, event and control settings stay unchanged.
