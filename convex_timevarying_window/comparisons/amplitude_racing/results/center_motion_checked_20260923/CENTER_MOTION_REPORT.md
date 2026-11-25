# Current checked method: center-motion matrix

Frequency scales translation and rotation periods together; only translation amplitude changes. Two fixed phases (0 and 120°) per condition are not random repetitions.

| Frequency | Center amplitude | Largest axis travel | Peak center speed* | Strict passes | Strict-pass flight time | Worst online cycle | Negative results |
|---:|---:|---:|---:|---:|---:|---:|---|
| 1× | 1× | 0.40 m | 0.143 m/s | 2/2 | 25.528, 25.296 s | 70.54 ms | — |
| 1× | 2× | 0.80 m | 0.285 m/s | 2/2 | 25.808, 25.288 s | 70.13 ms | — |
| 1× | 4× | 1.60 m | 0.570 m/s | 2/2 | 25.380, 29.528 s | 71.10 ms | — |
| 3× | 1× | 0.40 m | 0.428 m/s | 2/2 | 26.508, 27.108 s | 69.73 ms | — |
| 3× | 2× | 0.80 m | 0.855 m/s | 2/2 | 25.716, 25.736 s | 70.20 ms | — |
| 3× | 4× | 1.60 m | 1.711 m/s | 2/2 | 28.636, 25.488 s | 69.99 ms | — |
| 6× | 1× | 0.40 m | 0.855 m/s | 1/2 | 25.512 s | 73.15 ms | p0: sampled clearance -0.006 m |
| 6× | 2× | 0.80 m | 1.711 m/s | 2/2 | 25.588, 26.472 s | 70.79 ms | — |
| 6× | 4× | 1.60 m | 3.421 m/s | 2/2 | 29.140, 26.540 s | 71.38 ms | — |
| 10× | 1× | 0.40 m | 1.425 m/s | 2/2 | 25.440, 26.124 s | 71.74 ms | — |
| 10× | 2× | 0.80 m | 2.851 m/s | 2/2 | 29.284, 28.816 s | 74.34 ms | — |
| 10× | 4× | 1.60 m | 5.702 m/s | 2/2 | 32.004, 52.668 s | 70.18 ms | — |

*Numerical sampling over one motion period, not a continuous maximum certificate.
Strict pass means ordered seven-gate completion and goal return, zero Gazebo frame contacts, 10 ms sampled true sphere clearance and room bounds passed, and every full planning cycle under 100 ms.
No continuous-time or closed-loop safety certificate is claimed.

See `summary.json` and each run directory for all failures, raw logs, and audits.

## Slowest completed run

Rate 10×, center amplitude 4×, phase p1: 52.668 s. First accepted local plan at 18.876 s and first gate at 21.688 s. The planner did not accept a plan before that time; this does not prove that earlier safe flight was physically impossible.
