# Two-window planning + moving third-window reach time

2026-10-08. Independent method; no global continuation, center/free envelope,
bootstrap flight, per-window deadlines, or full-race monotonic admission.
Two free gate points and states use the existing MINCO-s3 and densified frame
constraints. The free local tail ends just beyond the second plane at the body
radius + frame thickness + inherited tracking margin + 0.02 m; it is not a parking
or halfway point. Exact projection imposes this exit plane, not a stiff penalty.
The final true return goal remains unchanged when the last gate enters the horizon.

One scalar tau estimates reaching the third moving center from this exit P/V/time:
norm(c(t+tau)-p-v*tau) <= a*tau^2/2 and norm(c(t+tau)-p) <= vmax*tau.
Only that third window is queried. No third polynomial or later window value.
The objective is local duration + tau + inherited smoothness/safety penalties.
This relaxed estimate ignores jerk/attitude and terminal velocity: not a safety
certificate, exact speed-bounded travel time, or guarantee of beating center.

L-BFGS-B must terminate successfully; return final result.x, never best iterate.
Normal convergence uses ftol=1e-6, gtol=1e-6, maxiter=10000 (SciPy success,
including relative objective reduction; not a claim of global/KKT optimality).
Development compared ftol 1e-6/1e-8/1e-10 on the same cold start: 353/1576/3106
iterations and objectives 10.451229/10.353071/10.351447. The 1e-6 setting is an
explicit accuracy/runtime tradeoff, not wall-time truncation. All three had
sampled residual <.003 and single-passage events. Actual runtime is reported.
CasADi analytic cost/gradient is compiled with -O1 before flight. Compilation
is charged separately; no trajectory is solved during this construction.
Unconverged results fail admission. A converged solve exceeding 100 ms also fails
online admission; report its actual time rather than disguising truncation.
Existing actual-body-exit/tracking-error events, 150 ms future-reference PVA
handover, MPC, PX4, contacts and independent 4 ms truth audit are unchanged.
PVA handover is not a full actual-state recovery guarantee.

Protocol: original seven anchors, fixed planes, 1 Hz / 2 m / phase 0 only.
One trial per variant: new free + third estimate; matched center + same estimate;
free without estimate (same exit-tail rule). Preserve every failure. No phase
scan or repeats; pre-race infrastructure failure alone permits one retry.
Historical center 24.428 s and previous global continuation 22.760 s are separate
references, not matched ablations. Strict return and seven-gate race separate.
