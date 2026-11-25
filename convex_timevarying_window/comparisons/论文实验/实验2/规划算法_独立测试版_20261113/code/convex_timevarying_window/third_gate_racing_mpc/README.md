# Third-center zero-VA racing initializer

2026-10-08. Changes to third_gate_stop_mpc: preserve the racing-horizon objective,
constraints, free first two crossings, third moving-center zero-world-VA terminal,
normal optimizer convergence, event-triggered replanning and MPC.
Only when advancing to a new horizon, initialize newly appended leg durations
with max(0.8, distance / configured speed limit, sqrt(3 distance / acceleration))
instead of distance / the old hard-coded 6 m/s. Shared crossings retain their
absolute time and PVA. Same-horizon admitted-curve seeds retain the original rule.
Cold launch uses the conservative duration scale speed/2 but a crossing-velocity
guess of 0.75*speed instead of 0.45*speed; this is not a higher execution limit.
A rejected candidate's decision vector is never blindly copied
to a new epoch: retry seeds also rebuild from the admitted curve and current head.
Its geometry is retained for the existing refinement procedure, not dropped.
This is an initial guess, NOT executable time scaling or an increased speed limit.
No gate IDs, frequency-specific constants, multi-candidate selection, new runtime
mode or timing contract is added. Third terminal is evaluated at its actual seed
arrival time when constructing the curve, with exact zero V/A.

Diagnosis: original flight 27.432 s versus historical center 24.428 s, with 7/7
accepted solves and no deadline misses. W3-W4 path lengths were 34.215/33.938 m,
but mean speed 6.547/10.333 m/s. New total estimated path length was shorter, so
slower flight was not simply caused by extra detours. At identical replay heads,
stricter ftol alone did not recover fast phases. Changing only time initialization
at first=2 changed a valid candidate's W4 time 16.186 -> 13.268 s and objective
13.215 -> 10.762. These are non-executed counterfactuals, not lap-time guarantees.

Develop/test on the same hardest original 1 Hz / 2 m / initial phase only, one
flight per explicitly frozen algorithm revision. Compare to the ORIGINAL 24.428 s
historical center, not a newly slowed center. Retain all development failures.
No safety/100 ms relaxation; normal successful optimizer final result required.

Final-horizon objective correction: when the tail is the actual return-to-start
leg (no third window remaining), subtract only its duration from the objective.
Race timing ends at last body exit, not at return home. Keep the return polynomial,
its exact jerk cost, dynamics, geometry and endpoint constraints. This does not
subtract the moving-third-window tail in earlier horizons. This is still a local
surrogate, not exact optimization of full-body exit time or a global bound.

Evaluation implementation: CasADi FunctionBuffer reuses input/output storage;
the parameter vector is set once per solve and gradients are copied for SciPy.
The L-BFGS-B settings, final-result-only rule, objective and gradient are unchanged.
Compiled at -O1 without fast-math. A trial of -O2 alone did not remove deadline
misses and is retained as a failed development revision, not the final solution.

Development evidence (each version one flight, not repeat statistics):
- racing_seed_v1_20261008: 23.448 s, no collision/seven exits, but one 107.023 ms
  call: strict online FAIL.
- racing_seed_o2_v1_20261008: 23.964 s, no collision/seven exits, but three misses,
  maximum 129.538 ms: strict online FAIL. Build 439.438 s, separately reported.
- racing_seed_buffer_v1_20261008: 22.968 s, no collision/seven exits, but one
  139.905 ms failed retry: strict online FAIL. A preceding arm failure did not
  reach racing and was retained/retried once under the existing protocol.
- racing_seed_epoch_v1_20261008: 27.428 s, safety/online passed, maximum 94.767 ms,
  but slower than center. Its cold solution reached gate one at 4.3 s.
- next revision changes only cold crossing-velocity initialization as above.
  Same-head replay at four recorded startup states gives first crossings near
  2.83 s; third-terminal objective improves at three heads, worsens at one.
  This is initialization evidence, not a global or complete-lap guarantee.
- racing_seed_cold_v1_20261008: 31.064 s, safety/online passed, maximum 75.407 ms;
  six gates by 22.692 s, but the final normal replan delayed gate seven to 31.064 s.
  No failures or retries. The next revision excludes post-race return duration
  from the race objective as described above; all prior negative results retained.
