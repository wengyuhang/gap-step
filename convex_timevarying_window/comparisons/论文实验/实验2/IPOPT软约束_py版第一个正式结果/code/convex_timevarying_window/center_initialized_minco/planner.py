"""Reuse one MINCO graph: solve at the centre, release point, retain incumbent."""
from copy import deepcopy
from dataclasses import replace
import time
import numpy as np
from convex_timevarying_window.comparisons.single_gate_swing.planner import (
    Planner as TranslationPlanner, Config)


class CenterInitialized:
    """Mixin for seven-variable P/V-MINCO; only indices 2:4 encode the gate point.

    Both stages solve the SAME request (head, absolute start, constraints, goal).
    A finite max_solve_ms, when supplied, is shared rather than doubled.
    The parent accepts only converged, finite-constraint/local-safety checked plans.
    """

    def order_seeds(self, seeds, state, now):
        seed = getattr(self, '_center_refinement_seed', None)
        if seed is not None:
            return [seed.copy()]
        return super().order_seeds(seeds, state, now)

    def solve(self, state, now, first):
        if not self.cfg.free_points:
            return super().solve(state, now, first)
        if self.nvar != 7:
            raise ValueError('Centre initialization expects two times, two point coordinates and velocity')
        began = time.perf_counter()
        cfg, lower, upper, admitted = self.cfg, self.lower.copy(), self.upper.copy(), self.admitted
        head = np.asarray(state, float).copy()
        center = candidate = None
        center_stats = free_stats = None
        free_error = None
        free_ms = 0.
        try:
            self.cfg = replace(cfg, free_points=False)
            self.lower[2:4] = self.upper[2:4] = 0.
            self.admitted = None
            center_start = time.perf_counter()
            try:
                center = super().solve(head, now, first)
            except Exception:
                stats = deepcopy(self.solvers[0][0].stats())
                stats.update(candidate_policy='center_then_free_incumbent', failed_stage='center',
                    optimization_ms=1000*(time.perf_counter()-began), both_stages_in_timing=True)
                self.solvers[0][0].value = stats
                raise
            center_ms = 1000*(time.perf_counter()-center_start)
            center_stats = deepcopy(center.optimizer_stats)
            if not np.array_equal(center.x[2:4], np.zeros(2)):
                raise RuntimeError('Centre-stage point was not fixed at the centre')
            # Preserve the complete admitted trajectory, not just its objective.
            self.last_center_plan = deepcopy(center)
            self.lower[:] = lower
            self.upper[:] = upper
            remaining_ms = (None if cfg.max_solve_ms is None else
                max(0., cfg.max_solve_ms-1000*(time.perf_counter()-began)))
            if remaining_ms is None or remaining_ms > 0:
                self.cfg = replace(cfg, max_solve_ms=remaining_ms)
                self.admitted = None
                self._center_refinement_seed = center.x.copy()
                free_start = time.perf_counter()
                try:
                    candidate = super().solve(head, now, first)
                except RuntimeError as exc:
                    if not str(exc).startswith(('no_converged_safe_passage:', 'no_safe_passage_under_budget:')):
                        raise
                    free_error = str(exc)
                finally:
                    free_ms = 1000*(time.perf_counter()-free_start)
                    free_stats = deepcopy(self.solvers[0][0].stats())
            else:
                free_error = 'shared_wall_budget_exhausted_after_center'
        finally:
            self.cfg = cfg
            self.lower[:] = lower
            self.upper[:] = upper
            self.admitted = admitted
            self._center_refinement_seed = None

        center_time = float(center.durations.sum())
        free_time = None if candidate is None else float(candidate.durations.sum())
        improved = free_time is not None and free_time < center_time-1e-8
        selected = candidate if improved else deepcopy(center)
        total_ms = 1000*(time.perf_counter()-began)
        stats = deepcopy(selected.optimizer_stats)
        stats.update(
            candidate_policy='center_then_free_incumbent',
            optimization_ms=total_ms, wall_time_limit_ms=cfg.max_solve_ms,
            iter_count=center_stats.get('iter_count', 0)+(free_stats or {}).get('iter_count', 0),
            center_stage_ms=center_ms, free_stage_ms=free_ms, both_stages_in_timing=True,
            center_stage=center_stats, free_stage=free_stats, free_stage_error=free_error,
            center_reference=dict(made_at=float(center.made_at), head_pvaj=head.tolist(),
                duration_s=center_time, durations=center.durations.tolist(),
                variables=center.x.tolist(), coefficients=center.coefficients.tolist()),
            refinement_initial_variables=center.x.tolist(),
            free_candidate_duration_s=free_time,
            selected_stage='free' if improved else 'center',
            nominal_time_saved_s=center_time-float(selected.durations.sum()),
            nominal_nonworsening=True, same_head_and_absolute_time=True,
            final_objective=float(selected.durations.sum()),
            guarantee_scope='returned nominal duration versus same-request admitted centre; not closed-loop flight time')
        selected.optimizer_stats = stats
        selected.solve_ms = total_ms
        self.solvers[0][0].value = stats
        return selected


class Planner(CenterInitialized, TranslationPlanner):
    """Current fixed-plane translation method with centre-initialized refinement."""
