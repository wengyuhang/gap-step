"""Centre-point starts, multiple duration starts, shortest admitted result."""
from copy import deepcopy
from dataclasses import replace
import time
import numpy as np
from convex_timevarying_window.comparisons.single_gate_swing.planner import (
    Planner as Parent, Config)


class MultipleTimeStarts:
    """All branches use the original objective/constraints and original four seeds."""
    time_seed_count = 4

    def order_seeds(self, seeds, state, now):
        index = self._time_seed_index
        if len(seeds) != self.time_seed_count:
            raise RuntimeError('Unexpected temporal seed family')
        seed = np.asarray(seeds[index], float).copy()
        if not np.array_equal(seed[2:4], np.zeros(2)):
            raise RuntimeError('Every temporal initialization must start at the gate centre')
        self._active_time_seed = seed.copy()
        return [seed]

    def solve(self, state, now, first):
        if self.nvar != 7:
            raise ValueError('Expected two times, two point coordinates and three crossing velocities')
        cfg, admitted = self.cfg, self.admitted
        head = np.asarray(state, float).copy()
        began = time.perf_counter()
        candidates = []
        best = None
        try:
            # A previous solution must not silently introduce another spatial/time start.
            self.admitted = None
            for index in range(self.time_seed_count):
                remaining = (None if cfg.max_solve_ms is None else
                    cfg.max_solve_ms-1000*(time.perf_counter()-began))
                if remaining is not None and remaining <= 0:
                    break
                self.cfg = replace(cfg, max_solve_ms=remaining)
                self._time_seed_index = index
                self._active_time_seed = None
                started = time.perf_counter()
                plan = None
                error = None
                try:
                    plan = super().solve(head, now, first)
                except RuntimeError as exc:
                    if not str(exc).startswith(('no_converged_safe_passage:', 'no_safe_passage_under_budget:')):
                        raise
                    error = str(exc)
                elapsed = 1000*(time.perf_counter()-started)
                stats = deepcopy(self.solvers[0][0].stats())
                row = dict(seed_index=index, seed_variables=self._active_time_seed.tolist(),
                    elapsed_ms=elapsed, admitted=plan is not None, error=error, optimizer_stats=stats,
                    duration_s=None if plan is None else float(plan.durations.sum()))
                candidates.append(row)
                if plan is not None and (best is None or plan.durations.sum() < best[0].durations.sum()):
                    best = plan, index
        finally:
            self.cfg = cfg
            self.admitted = admitted
            self._time_seed_index = None
            self._active_time_seed = None
        elapsed = 1000*(time.perf_counter()-began)
        stats = {} if best is None else deepcopy(best[0].optimizer_stats)
        stats.update(candidate_policy='shortest_admitted_of_four_center_point_time_starts',
            optimization_ms=elapsed, wall_time_limit_ms=cfg.max_solve_ms,
            iter_count=sum(r['optimizer_stats'].get('iter_count', 0) for r in candidates),
            temporal_initialization_scales=[1., .8, 1.2, 1.5],
            temporal_candidates=candidates, all_time_seeds_attempted=len(candidates)==self.time_seed_count,
            same_request_all_starts=True, all_spatial_initializations_at_center=True,
            preliminary_center_solve=False, previous_solution_used_as_initialization=False,
            selected_seed_index=None if best is None else best[1],
            converged=best is not None, formal_certificate=False,
            return_status='safe_multistart_found' if best is not None else 'no_admitted_multistart_candidate',
            nominal_global_optimality=False)
        self.solvers[0][0].value = stats
        if best is None:
            raise RuntimeError('no_admitted_multistart_candidate: '+str(elapsed))
        plan = best[0]
        stats['final_objective'] = float(plan.durations.sum())
        plan.solve_ms = elapsed
        plan.optimizer_stats = stats
        return plan


class Planner(MultipleTimeStarts, Parent):
    """Both free and fixed-centre variants use this identical multistart policy."""
