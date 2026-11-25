# Timing diagnosis (2026-10-08)

This is a separate cold-start profiling probe, run while the last flight was
still in preflight compilation. It is not a timed flight or a replacement result.

The new free solver at state [track.start, zero V/A/J], absolute time 0.15 s,
with frozen default settings converged in 353 iterations / 534 evaluations.
The objective was 10.451228636656728. cProfile adds measurement overhead:
the profiled whole solve was approximately 104 ms, optimization 101.326 ms.

- CasADi Function_call: 535 calls, 58 ms self / 60 ms cumulative.
- Native scipy L-BFGS-B setulb: 888 calls, 5 ms self.
- Remaining overhead includes Python callback dispatch, numpy/DM conversion,
  optimizer bookkeeping and sampled event evaluation.

The first actual free flight had 10 normally converged returned plans, 7 accepted.
Three first attempts after advancing the window horizon exceeded 100 ms:
124.565, 154.922, 105.998 ms. Their optimization portions were 118.730,
152.011, 103.226 ms. Next retries were 29.182, 44.283, 31.221 ms end-to-end.
This directly locates those deadline failures mainly inside iterative optimization,
not MPC, IPC or the final event check. It does not prove a unique reason for slow
lap time, nor that the third-window estimate alone creates the runtime cost.

All seven executed plans were converged final results and below 100 ms; rejected
over-budget attempts still count as online deadline misses. Do not conceal them
by reporting accepted-plan latency only.
