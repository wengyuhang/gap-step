from types import SimpleNamespace
import numpy as np
import pytest
from convex_timevarying_window.simple_two_gate_soft_only_mpc.planner import Planner, Config
from convex_timevarying_window.simple_two_gate_soft_only_mpc.execution import candidate_reason, planning_block_reason, Progress
from convex_timevarying_window.simple_two_gate_soft_only_mpc.reference import Reference
from convex_timevarying_window.simple_two_gate_mpc.planner import Planner as Original
from convex_timevarying_window.comparisons.original_fixed_plane_deadline.adapter import install
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case, build_case


@pytest.fixture(scope='module')
def problem():
    install(); track, _ = build_case(Case(1., 2., 0.))
    cfg = Config(compile_objective=False)
    return track, Planner(track, cfg, build_indices=[0]), Original(track, cfg, build_indices=[0])


def test_identical_objective_and_gradient(problem):
    track, new, old = problem
    state = np.r_[track.start, np.zeros(9)]; x = new.initial(state, .15, 0)
    np.testing.assert_array_equal(x, old.initial(state, .15, 0))
    par = np.r_[.15, state, np.tile([.15,.35,.65,.85], 6)]
    for a, b in zip(new.solvers[0][0].fg(x, par), old.solvers[0][0].fg(x, par)):
        np.testing.assert_array_equal(a, b)


def test_no_event_validation_or_residual_veto(problem, monkeypatch):
    track, new, _ = problem
    def forbidden(*args): raise AssertionError('Post-optimization scan ran')
    import convex_timevarying_window.event_free_terminal_mpc.core as core
    monkeypatch.setattr(core, 'plan_event_check', forbidden)
    monkeypatch.setattr(new, 'validate', forbidden)
    solver, evaluate, *bounds = new.solvers[0]
    # Deliberately return an unoptimized finite seed with a non-converged flag.
    class SeedSolver:
        def __call__(self, **kwargs): return {'x': kwargs['x0']}
        def stats(self): return {'return_status':'test nonconvergence', 'converged':False}
    solvers = list(new.solvers)
    solvers[0] = (SeedSolver(), evaluate, *bounds)
    monkeypatch.setattr(new, 'solvers', solvers)
    plan = new.solve(np.r_[track.start, np.zeros(9)], .15, 0)
    assert plan.violation > .003
    assert plan.event_valid is None and not plan.optimizer_stats['converged']
    assert candidate_reason(plan, 0, 'approach') is None
    assert new.prefix_clearance(plan, .15) is None
    ref = Reference(track.start, 7)
    assert ref.install(plan)
    ref.advance(plan.made_at, 0, 'approach')
    state, _ = ref.state_at(plan.end_time+2.)
    np.testing.assert_allclose(state[:3], plan.sample(plan.end_time))
    np.testing.assert_array_equal(state[3:], np.zeros(9))


def test_only_numeric_and_measured_identity():
    c = SimpleNamespace(first_gate=2, durations=np.ones(6), coefficients=np.zeros((6,6,3)),
                        x=np.zeros(28), made_at=-5., violation=999., event_valid=False)
    assert candidate_reason(c, 2, 'approach') is None
    assert candidate_reason(c, 3, 'approach') == 'stale_gate'
    assert candidate_reason(c, 2, 'exit') == 'body_exiting'
    c.x[0] = np.nan
    assert candidate_reason(c, 2, 'approach') == 'invalid_numeric_result'
    assert planning_block_reason(SimpleNamespace(crossed=False), None, 0., 100.) is None


def test_actual_progress_near_frame_not_nominal_abort():
    aperture = SimpleNamespace(contains=lambda q: True, boundary_distance=lambda q: .01)
    w = SimpleNamespace(aperture=aperture, world_to_local=lambda p,t:(np.asarray(p)[:2], p[2]))
    p = Progress([w], [1.], .4, .1)
    assert p.update([0,0,1],[0,0,-1],0.,1.)
    assert p.gate == 1 and p.violation is None


def test_driver_contains_no_legacy_veto():
    from pathlib import Path
    text = (Path(__file__).parents[1]/'px4.py').read_text()
    for forbidden in ('candidate.violation >= .003', 'elapsed >= 100.', 'candidate.event_valid:',
                      "reason = 'late_result'", 'raise RuntimeError(progress.violation)'):
        assert forbidden not in text
