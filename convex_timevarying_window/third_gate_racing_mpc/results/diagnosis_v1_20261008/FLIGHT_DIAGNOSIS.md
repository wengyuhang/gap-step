# Why the third-center zero-VA flight was slower

Historical center 24.428 s; third-center zero-VA 27.432 s. Same physical scene,
MPC and PX4. All 7 new plans converged and were accepted, with no deadline misses
or retries: waiting for the solver is not the explanation for the 3.004 s deficit.

Telemetry-position path lengths (not a continuous ground-truth length integral):
historical total 195.706 m; new 184.726 m. The new flight is about 11 m shorter,
so the deficit is not explained by a globally longer route.

| Leg | Old time/s | New time/s | Old length/m | New length/m | Old mean speed | New mean speed |
|---|---:|---:|---:|---:|---:|---:|
| Start-W1 | 3.276 | 3.316 | 16.711 | 15.606 | 5.159 | 4.742 |
| W1-W2 | 3.868 | 4.372 | 34.364 | 28.564 | 8.934 | 6.590 |
| W2-W3 | 2.932 | 3.396 | 29.802 | 25.692 | 10.306 | 7.652 |
| W3-W4 | 3.312 | 5.280 | 33.938 | 34.215 | 10.333 | 6.547 |
| W4-W5 | 4.308 | 5.220 | 27.181 | 35.549 | 6.383 | 6.859 |
| W5-W6 | 4.488 | 3.576 | 38.900 | 30.933 | 8.753 | 8.660 |
| W6-W7 | 2.244 | 2.272 | 14.810 | 14.167 | 6.641 | 6.321 |

New tracking error was at most 0.167 m in these legs, smaller than the historical
maximum 0.243 m. The controller followed a slower reference; this is not evidence
of MPC slowing down an otherwise fast reference.

Replanning also postponed the already planned next crossing by 0.468 s after W2
and 0.854 s after W4. No claim that these sum to the whole deficit: their effects
on moving-window phases and later states are coupled.

Code evidence: newly added durations were seeded by distance/6 plus acceleration
lower bound, and fresh crossing velocities by 0.45 * speed limit. On a 1 Hz moving
course, a longer initial duration can seed a later temporal solution. Normal
relative-objective convergence is not a global optimality certificate.

The counterfactuals.json replay uses the SAME recorded PVA heads and old references,
with unchanged objective/constraints. Stricter ftol alone stays near late phases.
Changing the kinematic initializer can produce substantially earlier, lower-cost,
constraint-valid solutions (not guaranteed for every head). The offline tests use
uncompiled evaluation and are NOT claimed to satisfy the flight's timing budget.
Numerical replay may differ from the original compiled solve because line-search
decisions and local minima are sensitive to floating-point arithmetic.

The implementation experiment keeps the third-center zero-VA terminal and all
constraints. It changes only new-horizon initialization, retaining cold startup,
shared absolute crossing times/states and same-horizon warm starts. It does not
restore periodic replanning, add timing deadlines, or increase speed limits.
This tests an initializer cause; it cannot yet explain every historical-method
difference, including the historical periodic update and different local tail.
