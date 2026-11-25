import copy

import numpy as np
import pytest

from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.best_center_release import make_planner
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import run
from convex_timevarying_window.comparisons.论文实验.实验2.matched_full_matrix.togt import matched_objective, initial_vector
from convex_timevarying_window.single_gate_stop_racing import point_mapping


@pytest.mark.parametrize('center', [False, True])
def test_equal_initial_curves_objectives_and_dynamics(center):
    track, _ = run.scenario.build_case(run.scenario.Case(level='a25_f6'))
    initializer = make_planner(track, True)
    ours = make_planner(track, not center)
    objective = matched_objective(track, center, initializer)
    assert objective.dimension == (8 if center else 33)
    limits = objective.config.dynamic_limits
    np.testing.assert_allclose(limits.max_rotor_thrust, 8.54858*.95)
    np.testing.assert_allclose(limits.max_body_rate_xy, np.deg2rad(209.))
    np.testing.assert_allclose(limits.max_body_rate_z, np.deg2rad(190.))
    for index, ours_seed in enumerate(ours.seeds()):
        initial = initial_vector(objective, initializer, index)
        forward = objective.forward(initial)
        coefficients = forward.trajectory.coefficients*forward.durations[:,None,None]**np.arange(8)[None,:,None]
        expected = np.asarray(ours.curve_values(ours_seed)).ravel().reshape(8,3,8).transpose(0,2,1)
        np.testing.assert_allclose(coefficients, expected, rtol=0., atol=1e-8)
        np.testing.assert_allclose(objective.value_and_gradient(initial)[0], ours.objective(ours_seed)[0], rtol=1e-8, atol=1e-4)
        if center:
            for window, at, point in zip(track.windows, forward.traversal_times, forward.waypoints):
                np.testing.assert_allclose(point,window.state_at(at)[0],atol=1e-10)


def test_identical_safe_maps_and_physical_track_unchanged():
    track, _ = run.scenario.build_case(run.scenario.Case(level='a75_f6'))
    original = copy.deepcopy(track)
    initializer = make_planner(track, True)
    objective = matched_objective(track, False, initializer)
    rng = np.random.default_rng(76)
    for window, old, aligned, inset in zip(track.windows, original.windows, objective.track.windows, initializer.point_apertures):
        assert window.aperture.margin == old.aperture.margin
        if inset.kind == 'circle':
            assert window.aperture.radius == old.aperture.radius == aligned.aperture.radius
        else:
            np.testing.assert_array_equal(window.aperture.physical_vertices, aligned.aperture.physical_vertices)
            np.testing.assert_array_equal(window.aperture.safe_vertices,old.aperture.safe_vertices)
        for raw in rng.normal(size=(30,point_mapping.dimension(inset))):
            actual, _ = aligned.aperture.point_and_jacobian(raw)
            np.testing.assert_allclose(actual,point_mapping.numeric(inset,raw),atol=1e-12)


def test_matched_dynamic_togt_gradient():
    track, _ = run.scenario.build_case(run.scenario.Case(level='a25_f6'))
    initializer = make_planner(track,True)
    for center in (False,True):
        objective = matched_objective(track,center,initializer)
        initial = initial_vector(objective,initializer,0)
        rng = np.random.default_rng(4)
        direction = rng.normal(size=len(initial))
        step = 1e-7
        _, gradient = objective.value_and_gradient(initial)
        difference = (objective.value_and_gradient(initial+step*direction)[0]-objective.value_and_gradient(initial-step*direction)[0])/(2*step)
        np.testing.assert_allclose(difference,gradient@direction,rtol=3e-4,atol=.1)
