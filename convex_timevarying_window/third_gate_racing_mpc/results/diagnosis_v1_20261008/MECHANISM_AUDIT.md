# Mechanism audit, 2026-10-08

## What the evidence supports

1. The previous 27.432 s lap had seven admitted solves, no retries and no
   planner deadline misses. Therefore its 3.004 s loss to historical center is
   NOT a solver-waiting explanation.
2. Its reference was slower rather than simply longer or badly tracked. The
   third-to-fourth-gate leg alone lost 1.968 s over nearly equal distance.
3. A fixed 6 m/s new-leg duration guess can place local optimization near a later
   periodic-window opportunity. Controlled same-head, same-helper tests recover
   substantially lower objective and earlier fourth-gate arrival using the
   configured racing speed for the guess. Not every head improves; no global
   optimum claim is supported.
4. Receding-horizon changes of endpoint/state still matter: previously planned
   common-window arrivals moved later by 0.468 and 0.854 s at two updates. The
   initializer change does not impose a new monotonicity contract or solve the
   exact whole-course value problem.
5. In later development flights, rejected-solution retries expose a separate
   implementation weakness: core.initial returned searched.x.copy() at a new
   now and head state. Its exp(logT) durations were unmodified, so the old absolute
   crossing times were silently shifted. This is not an epoch-consistent warm
   start for moving gates. It cannot explain the original no-retry 27.432 s lap;
   it matters for the later failure/retry sequences.

## Minimal final implementation

- Keep the two free crossings and true third moving-center terminal with world
  V=A=0. No farther-course trajectory, candidate envelope or deadline contract.
- Reconstruct initialization from the admitted reference and new head. Retain
  shared absolute crossing times/PVA; new durations use the configured speed and
  acceleration scales. Cold launch retains a conservative duration guess but
  raises only its crossing-velocity guess from 0.45 to 0.75 of the same speed limit.
  Failed vectors are not blindly
  reused at later epochs; failed geometry still feeds existing refinement.
- Keep jerk/safety costs, dynamics, safety thresholds and MPC. At the actual end
  of the course, exclude post-race return duration from time cost: lap timing
  stops at last-window exit, not at return home. Keep the return polynomial and
  all its constraints. Earlier moving-third-window duration remains in cost.
- Reuse CasADi evaluation memory, copy returned gradients, unchanged SciPy
  convergence tolerances and final iterate. No wall-time truncation or best iterate.
- Every call, including a rejected call, counts toward the 100 ms online audit.

## What is not established

Historical center had different planning/update/termination choices, so this is
not a pure free-point-versus-center ablation. A superset of center crossing points
does not imply faster executed laps with different receding-horizon states,
different terminal boundaries, non-time costs and a local optimizer.

One fixed-scenario flight per development revision cannot prove statistical
reliability, global superiority, worst-case runtime, or continuous safety.
Runtime result and physical result must remain separate even when the lap is faster.

## Later negative evidence and objective mismatch

Epoch-correct initialization passed online timing (max94.767ms) but flew27.428s;
the subsequent cold-seed revision passed timing (max75.407ms) but flew31.064s.
Thus initializer changes alone have not demonstrated robust superiority.
The latter exited W6 at22.692s, then replanned W7 to31.064s without retries.
The original cost included the post-race return duration. Removing it is a
metric-alignment change, not proof that it alone caused the entire delay.
Same recorded W7 head, uncompiled replay: with return-time cost crossing27.242s,
end31.095s; without crossing25.797s, end40.797s. Both residual/event-valid. The
slower return is disclosed, and this replay does not predict exact PX4 lap time.
