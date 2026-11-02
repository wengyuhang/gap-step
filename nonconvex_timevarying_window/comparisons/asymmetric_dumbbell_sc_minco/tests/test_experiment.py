from dataclasses import replace
import numpy as np
import pytest
from shapely.geometry import Polygon

from convex_timevarying_window.single_gate_stop_racing.planner import Config
from convex_timevarying_window.single_gate_stop_racing.geometry import symbolic_constraints
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.scenario import (
    VERTICES, LEFT_INITIAL_POINT, Case, build_case, room)
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.planner import Planner, recenter_at_left


@pytest.fixture(scope='module')
def mapping():
    return SCDiskMap.fit(VERTICES)


def test_physical_polygon_and_disconnected_body_regions():
    cfg = Config()
    p = Polygon(VERTICES)
    assert p.is_valid and p.exterior.is_ccw and not p.interiors
    safe = p.buffer(-(cfg.radius + cfg.frame_thickness + cfg.passage_margin))
    assert len(safe.geoms) == 2
    ap = build_case().windows[0].aperture
    assert ap.contains(LEFT_INITIAL_POINT)
    assert ap.contains([1.6, 0.])
    assert ap.contains([-.2, 0.])
    assert not ap.contains([-.2, .5])  # In convex hull, OUTSIDE actual polygon.
    assert ap.boundary_distance(LEFT_INITIAL_POINT) == pytest.approx(.6)
    assert ap.boundary_distance([-.2, 0.]) == pytest.approx(.2)


def test_finite_segment_constraints_match_true_nonconvex_boundary():
    import casadi as ca
    ap = build_case().windows[0].aperture
    for p in ([0., 0., 0.], [-1., 0., .2], [1.6, 0., .4], [-.2, .5, .1]):
        q = np.array(p)
        g = symbolic_constraints(ap, ca.DM(q), .57)
        expected = ap.boundary_distance(q[:2]) ** 2 + q[2] ** 2 - .57 ** 2
        assert min(float(v) for v in g) == pytest.approx(expected)


def test_plane_and_motion_are_as_requested():
    w = build_case(Case()).windows[0]
    for t in np.linspace(0, 1, 11):
        c, R, *_ = w.state_at(t)
        np.testing.assert_allclose(R, w.state_at(0.)[1], atol=0.)
        assert c[1] == 0 and c[2] == 3.2
        assert abs(R[:, 2] @ w.motion.translation_amplitude) < 1e-12


def test_local_bound_handles_actual_concave_frame(mapping):
    from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
    from convex_timevarying_window.single_gate_stop_racing.geometry import passage, passage_python
    track = build_case(Case(3., 0.))
    window = track.windows[0]
    # Straight, forward passage through the small room; no optimization.
    coeff = np.zeros((2, 8, 3))
    coeff[0, 0] = [-1., -4., 3.2]
    coeff[0, 1] = [0., 4., 0.]
    coeff[1, 0] = [-1., 0., 3.2]
    coeff[1, 1] = [0., 4., 0.]
    plan = Plan(0., 0, np.ones(2), coeff, np.zeros(7), 0., 0., 'test', 1)
    for piece in (0, 1):
        assert passage(plan, window, .57, piece)['passed']
        assert passage_python(plan, window, .57, piece)['passed']
    # A centre in the corridor is legal as a POINT, not as a body.
    plan.coefficients[:, 0, 0] = -.2
    for piece in (0, 1):
        assert not passage(plan, window, .57, piece)['passed']
        assert not passage_python(plan, window, .57, piece)['passed']


def test_equivalent_sc_normalization_at_left(mapping):
    centered = recenter_at_left(mapping)
    np.testing.assert_allclose(centered.map_unconstrained([0., 0.]), LEFT_INITIAL_POINT, atol=1e-7)
    assert np.linalg.cond(centered.jacobian_unconstrained([0., 0.])) == pytest.approx(1.)
    # Both maps represent the exact same physical point under a disk automorphism.
    from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import B
    for d in ([.01, .02], [1., -.3], [-2., 1.]):
        disk = B(d)
        raw = centered._automorphism(complex(*disk))
        a = mapping.normalization
        original_disk = (raw-a)/(1-np.conj(a)*raw)
        q = mapping.evaluate(original_disk)
        np.testing.assert_allclose(q, centered.map_unconstrained(d), atol=2e-7)


def test_left_inverse_and_full_nlp_analytic_jacobian(mapping):
    cfg = replace(Config(), compile_objective=False, nodes_per_piece=2)
    track = build_case(Case(3., 2.))
    p = Planner(track, cfg, mapping)
    np.testing.assert_allclose(mapping.map_unconstrained(p.initial_d), LEFT_INITIAL_POINT, atol=1e-7)
    assert room(mapping.map_unconstrained(p.initial_d)) == 'left'
    offsets = np.tile(np.r_[cfg.passage_seed_offsets,
                           np.zeros(cfg.witness_slots-len(cfg.passage_seed_offsets))], 2)
    par = np.r_[0., track.start, np.zeros(9), offsets]
    x = np.r_[np.log([1.411, 2.1]), p.initial_d, [0., 5., 0.]]
    raw = p.function(x, par)
    analytic = np.vstack([np.asarray(raw[1])[None, :], raw[3]])
    finite = np.zeros_like(analytic)
    for k in range(7):
        h = 2e-7 * max(1., abs(x[k]))
        xp, xm = x.copy(), x.copy()
        xp[k] += h
        xm[k] -= h
        plus, minus = p.function(xp, par), p.function(xm, par)
        finite[:, k] = np.r_[float(plus[0])-float(minus[0]),
                              np.asarray(plus[2]-minus[2]).ravel()] / (2*h)
    np.testing.assert_allclose(analytic, finite, rtol=3e-4, atol=3e-5)
