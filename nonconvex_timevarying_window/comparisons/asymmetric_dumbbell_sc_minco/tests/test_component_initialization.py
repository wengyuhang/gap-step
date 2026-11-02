from types import SimpleNamespace

import numpy as np
import pytest
from shapely.geometry import Point, Polygon

from convex_timevarying_window.dynamic_passage_minco.planner import Stats
from convex_timevarying_window.single_gate_stop_racing.planner import Config
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.scenario import VERTICES
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.component_initialization.geometry import component_seeds
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.component_initialization import planner as module


def test_two_components_and_representative_clearance():
    radius = Config().radius
    seeds = component_seeds(VERTICES, radius)
    assert len(seeds) == 2
    assert seeds[0].point[0] < -.4 and seeds[1].point[0] > 0
    for seed in seeds:
        assert seed.region.covers(Point(seed.point))
        assert seed.clearance >= radius
        assert Polygon(VERTICES).covers(Point(seed.point))
    refined = component_seeds(VERTICES, radius, quad_segs=128)
    assert len(refined) == len(seeds)
    # A maximum-clearance centre need not be unique. Compare the regions and
    # achieved clearance, not identical coordinates along a flat optimum.
    for coarse, fine in zip(seeds, refined):
        assert coarse.region.hausdorff_distance(fine.region) < 3e-5
        assert abs(coarse.clearance-fine.clearance) < 2e-6


def test_geometry_only_not_hardcoded_left_right():
    shift = np.array([10., 4.])
    original = component_seeds(VERTICES, .38594472)
    moved = component_seeds(VERTICES+shift, .38594472)
    assert len(moved) == len(original)
    for a, b in zip(original, moved):
        assert a.region.buffer(1e-9).covers(Point(b.point-shift))
        assert b.clearance == pytest.approx(a.clearance, abs=2e-6)
    square = np.array([[0., 0.], [2., 0.], [2., 2.], [0., 2.]])
    assert len(component_seeds(square, .2)) == 1
    assert component_seeds(square, 2.) == []
    with pytest.raises(ValueError):
        component_seeds(square, -.1)


def fake_planner():
    planner = object.__new__(module.Planner)
    planner.cfg = Config(max_solve_ms=5000.)
    planner.components = component_seeds(VERTICES, planner.cfg.radius)
    planner.seed_coordinates = [dict(d=c.point.copy(), error=None) for c in planner.components]
    planner.erosion_ms = planner.inverse_ms = 0.
    planner.solvers = [(Stats(),)]
    planner.mapping = SimpleNamespace(map_unconstrained=lambda d: np.asarray(d))
    return planner


@pytest.mark.parametrize('fail_first', [False, True])
def test_all_components_are_attempted_and_shortest_accepted_is_selected(monkeypatch, fail_first):
    planner = fake_planner()
    config = planner.cfg
    calls = []

    def local(p, state, now, first):
        calls.append(p.initial_d.copy())
        stats = dict(optimization_ms=1., local_check_ms=.1)
        p.solvers[0][0].value = stats
        if fail_first and len(calls) == 1:
            raise RuntimeError('no_safe_passage_under_budget')
        return SimpleNamespace(durations=np.array([1., 3.-len(calls)]),
                               x=np.r_[0., 0., p.initial_d, np.zeros(3)],
                               coefficients=np.zeros((2, 8, 3)), crossing_times=np.array([1.]),
                               optimizer_stats=stats)

    monkeypatch.setattr(module, 'solve_local', local)
    plan = planner.solve(np.zeros(12))
    assert len(calls) == 2
    assert plan.optimizer_stats['selected_component'] == 1
    assert planner.cfg is config
    assert plan.optimizer_stats['all_components_attempted']


def test_implementation_exception_is_not_mislabeled_as_infeasibility(monkeypatch):
    planner = fake_planner()
    config = planner.cfg

    def broken(*args):
        raise RuntimeError('unexpected implementation problem')

    monkeypatch.setattr(module, 'solve_local', broken)
    with pytest.raises(RuntimeError, match='unexpected implementation'):
        planner.solve(np.zeros(12))
    assert planner.cfg is config
