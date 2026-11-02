"""Regression: same sole left initialization, only escape switch differs."""
import numpy as np
import pytest

from convex_timevarying_window.single_gate_stop_racing.planner import Config
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import B, SCDiskMap
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.scenario import (
    VERTICES, LEFT_INITIAL_POINT, build_case, room)
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.single_seed_poll.solver import (
    Planner, disk_trials)


def test_symmetric_disk_poll_keeps_endpoints_in_original_domain():
    trials = list(disk_trials(np.zeros(2)))
    points = np.array([B(d) for d, _ in trials])
    assert len(points) == 24  # unit-radius endpoints are excluded
    assert np.all(np.linalg.norm(points, axis=1) < 1.)
    for point in points:
        assert np.min(np.linalg.norm(points+point, axis=1)) < 1e-12


def test_one_left_seed_escapes_without_changing_constraints():
    mapping = SCDiskMap.fit(VERTICES)
    cfg = Config(compile_objective=False, max_solve_ms=15000.)
    track = build_case()
    state = np.r_[track.start, np.zeros(9)]
    control = Planner(track, cfg, mapping)
    with pytest.raises(RuntimeError, match='local_infeasible_stagnation'):
        control.solve(state, escape=False)
    candidate = Planner(track, cfg, mapping)
    plan = candidate.solve(state, escape=True)
    old, new = control.solvers[0][0].stats(), candidate.solvers[0][0].stats()
    np.testing.assert_array_equal(old['initial_variables'], new['initial_variables'])
    np.testing.assert_allclose(mapping.map_unconstrained(np.asarray(new['initial_variables'])[2:4]),
                               LEFT_INITIAL_POINT, atol=1e-7)
    assert old['initial_count'] == new['initial_count'] == 1
    assert new['hop_count'] == 1
    assert new['polls'][0]['selected']['violation_loss'] > new['polls'][0]['origin_loss']
    assert new['rounds'][-1]['minimum_residual'] >= -1e-5
    assert new['rounds'][-1]['passage']['passed']
    assert room(mapping.map_unconstrained(plan.x[2:4])) == 'right'
    assert all(track.windows[0].aperture.contains(v['q'], tolerance=1e-7)
               for v in candidate.evaluations)
    assert not new['formal_certificate']
