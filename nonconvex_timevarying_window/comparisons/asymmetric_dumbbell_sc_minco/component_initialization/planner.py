"""Change spatial initialization only; reuse the original local solver verbatim."""
from dataclasses import replace
import time

import numpy as np
from shapely.geometry import LineString, Polygon

from ..planner import Planner as BasePlanner, inverse_along_corridor
from ..solver import solve as solve_local
from .geometry import component_seeds


def inverse_seed(mapping, target):
    """Invert the original SC map; never remap the eroded/disconnected domain."""
    try:
        disk = mapping.inverse(target)
        d = disk / np.sqrt(1. - float(disk @ disk))
        if np.isfinite(d).all() and np.linalg.norm(mapping.map_unconstrained(d)-target) < 1e-7:
            return d
    except (RuntimeError, ValueError):
        pass
    # Reuse the previous inverse continuation only when its straight path is
    # inside this physical polygon. This is numerical inversion, not a flight.
    if not Polygon(mapping.vertices).covers(LineString([mapping.q0, target])):
        raise RuntimeError('SC inverse requires an interior continuation path')
    d = inverse_along_corridor(mapping, target)
    if np.linalg.norm(mapping.map_unconstrained(d)-target) > 1e-7:
        raise RuntimeError('SC inverse did not reproduce the component seed')
    return d


class Planner(BasePlanner):
    def __init__(self, track, cfg, mapping):
        began = time.perf_counter()
        self.components = component_seeds(track.windows[0].aperture.physical_vertices, cfg.radius)
        self.erosion_ms = 1000*(time.perf_counter()-began)
        self.seed_coordinates = []
        began = time.perf_counter()
        for component in self.components:
            try:
                self.seed_coordinates.append(dict(d=inverse_seed(mapping, component.point), error=None))
            except (RuntimeError, ValueError) as exc:
                self.seed_coordinates.append(dict(d=None, error=str(exc)))
        self.inverse_ms = 1000*(time.perf_counter()-began)
        super().__init__(track, cfg, mapping)

    def solve(self, state, now=0., first=0):
        if first != 0:
            raise ValueError('This experiment has one gate')
        original_cfg = self.cfg
        began = time.perf_counter()
        deadline = began + original_cfg.max_solve_ms/1000.
        branches, accepted = [], []
        try:
            for component, coordinate in zip(self.components, self.seed_coordinates):
                row = dict(component=component.record(), accepted=False,
                           flight_executed=False, stats={}, evaluations=[])
                branches.append(row)
                if coordinate['error'] is not None:
                    row['failure'] = 'initialization_failed: '+coordinate['error']
                    continue
                remaining_ms = 1000*(deadline-time.perf_counter())
                if remaining_ms <= 3.:
                    row['failure'] = 'not_attempted_shared_budget_exhausted'
                    continue
                self.cfg = replace(original_cfg, max_solve_ms=remaining_ms)
                self.initial_d = coordinate['d'].copy()
                self.evaluations, self.map_ms = [], 0.
                self._map_cached_d = self._map_cached_value = None
                self.solvers[0][0].value = {}
                row['initial_d'] = self.initial_d.tolist()
                try:
                    plan = solve_local(self, state, now, first)
                    row.update(accepted=True, nominal_total_time_s=float(plan.durations.sum()),
                               final_q=self.mapping.map_unconstrained(plan.x[2:4]).tolist(),
                               crossing_time_s=float(plan.crossing_times[0]),
                               plan=dict(coefficients=plan.coefficients.tolist(),
                                         durations=plan.durations.tolist(), x=plan.x.tolist()))
                    accepted.append((component.index, plan))
                except RuntimeError as exc:
                    if str(exc) != 'no_safe_passage_under_budget':
                        raise  # Do not silently turn implementation failures into infeasibility.
                    row['failure'] = str(exc)
                row['stats'] = self.solvers[0][0].stats()
                row['evaluations'] = self.evaluations
        finally:
            self.cfg = original_cfg
        elapsed = 1000*(time.perf_counter()-began)
        winner = min(accepted, key=lambda item: float(item[1].durations.sum())) if accepted else None
        stats = dict(return_status='safe_passage_found' if winner else 'no_accepted_component',
                     optimization_ms=elapsed, accepted=winner is not None,
                     selected_component=winner[0] if winner else None,
                     component_count=len(self.components), branches=branches,
                     erosion_radius_m=original_cfg.radius,
                     offline_erosion_ms=self.erosion_ms, offline_inverse_ms=self.inverse_ms,
                     candidate_policy='original_local_solve_per_component_then_shortest_accepted',
                     time_seed_policy='original_four_phase_seeds_per_spatial_seed',
                     point_parameterization='original_SC_physical_polygon_not_eroded',
                     formal_certificate=False, flight_executed=False,
                     all_components_attempted=all(bool(row['stats']) for row in branches),
                     local_check_ms=sum(row['stats'].get('local_check_ms', 0.) for row in branches))
        self.solvers[0][0].value = stats
        if winner is None:
            raise RuntimeError('no_accepted_component')
        plan = winner[1]
        stats['selected_local_stats'] = plan.optimizer_stats
        plan.optimizer_stats, plan.solve_ms = stats, elapsed
        return plan
