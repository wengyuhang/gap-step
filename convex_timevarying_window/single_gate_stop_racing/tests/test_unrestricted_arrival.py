"""Arrival seeds do not become extra time constraints in SLSQP."""
import numpy as np
import pytest

from convex_timevarying_window.single_gate_stop_racing import solver
from convex_timevarying_window.single_gate_stop_racing.planner import (
    Config, Single, parking_points, route_signs,
)
from convex_timevarying_window.single_gate_stop_racing.scenario import Case, build_case, workspace_bounds
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack


@pytest.mark.parametrize('frequency,now', [(1., 0.), (4., 6.174)])
def test_optimizer_retains_full_duration_bounds(monkeypatch, frequency, now):
    track, _ = build_case(Case(frequency, 4., 0.))
    lower, upper = workspace_bounds()
    cfg = Config(compile_objective=False, feasibility_restore=False, max_solve_ms=10000.,
                 workspace_lower=lower, workspace_upper=upper)
    local = SCWindowTrack('arrival_test', track.start, parking_points(track, cfg)[0],
                         (track.windows[0],), (0,))
    planner = Single(local, cfg, route_signs(track)[0])
    state = np.r_[track.start, np.zeros(9)]

    class BoundsObserved(Exception):
        pass

    def inspect_minimize(fun, initial, **kwargs):
        # SLSQP sees y with x = origin + scale*y. The unmodified tail
        # duration has unit scale, arrival has the existing phase scaling.
        import inspect
        evaluate = inspect.getclosurevars(fun).nonlocals['evaluate']
        context = inspect.getclosurevars(evaluate).nonlocals
        origin, scale = context['origin'], context['scale']
        bounds = np.asarray(kwargs['bounds'])
        actual_lower = origin + scale*bounds[:, 0]
        actual_upper = origin + scale*bounds[:, 1]
        np.testing.assert_allclose(actual_lower[:2], np.log([.3, .3]))
        np.testing.assert_allclose(actual_upper[:2], np.log([15., 15.]))
        assert np.exp(actual_upper[0])-np.exp(actual_lower[0]) > 10.
        raise BoundsObserved

    monkeypatch.setattr(solver, 'minimize', inspect_minimize)
    with pytest.raises(BoundsObserved):
        planner.solve(state, now, 0)
