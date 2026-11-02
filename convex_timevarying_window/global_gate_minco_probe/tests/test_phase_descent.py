import numpy as np
from convex_timevarying_window.global_gate_minco_probe.phase_descent import earlier_period_seed
from convex_timevarying_window.global_gate_minco_probe.flight import Reference, Scheduler
from convex_timevarying_window.single_gate_stop_racing.scenario import Case, build_case


def test_earlier_opportunity_preserves_phase_and_pv():
    track, _ = build_case(Case(4., 4., 0.))
    x = np.r_[np.log(np.full(8, 5.)), np.arange(46.)]
    seed = earlier_period_seed(x, track)
    np.testing.assert_allclose(np.exp(seed[:8]), [4.75]*7+[5.])
    np.testing.assert_array_equal(seed[8:], x[8:])
    for w, t, s in zip(track.windows, np.cumsum(np.exp(x[:8])), np.cumsum(np.exp(seed[:8]))):
        np.testing.assert_allclose(w.state_at(t)[0], w.state_at(s)[0], atol=1e-10)


def test_global_reference_has_no_prerace_preview_or_gate_discard():
    plan = object(); ref = Reference(np.zeros(3), plan)
    ref.advance(-.01, 0, 'approach')
    np.testing.assert_array_equal(ref.sample([-.01, .5, 1.]), np.zeros((3, 9)))
    assert ref.advance(0., 0, 'approach') is plan
    assert ref.advance(20., 4, 'crossing') is plan
    scheduler = Scheduler()
    scheduler.observe(0., 0, False, 100.)
    assert not scheduler.due(0.)
