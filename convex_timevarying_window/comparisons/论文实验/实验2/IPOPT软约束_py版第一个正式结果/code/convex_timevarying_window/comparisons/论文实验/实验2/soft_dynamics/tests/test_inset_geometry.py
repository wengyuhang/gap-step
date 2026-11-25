import casadi as ca
import numpy as np

from convex_timevarying_window.global_gate_minco_probe import planner as core
from convex_timevarying_window.single_gate_stop_racing import point_mapping
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario
from convex_timevarying_window.comparisons.论文实验.实验2.safety import frame_aperture
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import kernels
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.inset_geometry import Planner
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.interior_point import geometric_constraints


def test_inset_mapping_and_stronger_clearance_preserve_physical_windows():
    track, _ = scenario.build_case(scenario.Case(level='a75_f6'))
    originals = [(window.aperture.radius, None if window.aperture.kind == 'circle'
        else window.aperture.physical_vertices.copy()) for window in track.windows]
    factory, symbolic, inverse = core.Config, point_mapping.symbolic, point_mapping.inverse
    planner = Planner(track, True, solver='ipopt', dynamic_limit_fraction=.95,
        pose_function=scenario.pose)
    assert core.Config is factory and point_mapping.symbolic is symbolic
    assert planner.nvar == 96 and planner.velocity_start-planner.n == 25
    np.testing.assert_allclose(planner.cfg.passage_margin, .15)
    radius = planner.cfg.radius+planner.cfg.frame_thickness+planner.cfg.passage_margin+planner.cfg.witness_slack
    np.testing.assert_allclose(radius, .62594472)
    rng = np.random.default_rng(0)
    for window, aperture, original in zip(track.windows, planner.point_apertures, originals):
        assert window.aperture.radius == original[0]
        if original[1] is not None:
            np.testing.assert_array_equal(window.aperture.physical_vertices, original[1])
        frame = frame_aperture(window.aperture)
        for scale in (.01, 1., 100.):
            for raw in rng.normal(size=(20, point_mapping.dimension(aperture)))*scale:
                point = point_mapping.numeric(aperture, raw)
                if frame.kind == 'circle':
                    assert frame.radius-np.linalg.norm(point) >= radius-1e-10
                else:
                    vertices = frame.physical_vertices
                    edges = np.roll(vertices, -1, axis=0)-vertices
                    normals = np.c_[edges[:, 1], -edges[:, 0]]/np.linalg.norm(edges, axis=1)[:, None]
                    assert np.min(np.sum(normals*vertices, axis=1)-normals@point) >= radius-1e-10
    assert len(planner.seeds()) == 4 and point_mapping.inverse is inverse


def test_inset_curve_matches_both_finite_geometry_paths(monkeypatch):
    monkeypatch.setattr(kernels, 'local_options', lambda name: {'cse': True, 'never_inline': True})
    track, _ = scenario.build_case(scenario.Case(level='a75_f6'))
    for free in (True, False):
        planner = Planner(track, free, solver='ipopt', dynamic_limit_fraction=.95,
            pose_function=scenario.pose)
        assert planner.nvar == (96 if free else 71)
        variable = ca.MX.sym('inset_trajectory', planner.nvar)
        witnesses = planner.witnesses()
        witnesses[(0, 1)] = dict(kind='after', values=[0., .03])
        original, original_upper = geometric_constraints(planner, variable, witnesses)
        factorized, factorized_upper = kernels.geometric_constraints(planner, variable, witnesses)
        np.testing.assert_array_equal(factorized_upper, original_upper)
        difference = factorized-original
        check = ca.Function('inset_geometry_consistency', [variable],
            [difference, ca.jacobian(difference, variable)])
        for seed in planner.seeds():
            values, derivatives = check(seed)
            np.testing.assert_allclose(values, 0., atol=1e-8)
            np.testing.assert_allclose(derivatives, 0., atol=1e-6)
            if not free:
                expected = np.array([window.state_at(t)[0] for window, t in
                    zip(track.windows, np.cumsum(np.exp(seed[:planner.n]))[:-1])])
                np.testing.assert_allclose(np.asarray(planner.point_function(seed)).T, expected, atol=1e-12)
