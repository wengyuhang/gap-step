import numpy as np

from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.phase_initialization import Planner as PhasePlanner
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.spatial_initialization import Planner
from convex_timevarying_window.comparisons.论文实验.实验2.safety import frame_aperture
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap


def test_spatial_initialization_preserves_times_and_samples_inside_inset_regions():
    track, _ = scenario.build_case(scenario.Case(level='a75_f6'))
    sampled = Planner(track, True, solver='ipopt', dynamic_limit_fraction=.95,
        pose_function=scenario.pose)
    centered = PhasePlanner(track, True, solver='ipopt', dynamic_limit_fraction=.95,
        pose_function=scenario.pose)
    np.testing.assert_array_equal(sampled.initial_times(), centered.initial_times())
    local_points = sampled.initial_points()
    np.testing.assert_array_equal(local_points, sampled.initial_points())
    assert local_points.shape == (8, 7, 2)
    assert np.all(np.linalg.norm(local_points, axis=2) > 1e-6)
    for gate, window in enumerate(track.windows):
        aperture = sampled.point_apertures[gate]
        assert all(aperture.contains(point) for point in local_points[:, gate])
        assert np.min(frame_aperture(window.aperture).boundary_distance(local_points[:, gate])) >= .62594472-1e-10


def test_each_spatial_seed_recomputes_consistent_points_and_minco_pvaj():
    track, _ = scenario.build_case(scenario.Case(level='a75_f6'))
    planner = Planner(track, True, solver='ipopt', dynamic_limit_fraction=.95,
        pose_function=scenario.pose)
    for seed, local_points in zip(planner.seeds(), planner.initial_points()):
        durations = np.exp(seed[:planner.n])
        arrivals = np.cumsum(durations)[:-1]
        expected_points = np.array([window.state_at(at)[0]+window.state_at(at)[1][:, :2]@point
            for window, at, point in zip(track.windows, arrivals, local_points)])
        np.testing.assert_allclose(np.asarray(planner.point_function(seed)).T, expected_points, atol=1e-12)
        curve = MincoSnap(BoundaryState(track.start), BoundaryState(track.goal), expected_points, durations)
        derivatives = np.concatenate([curve.evaluate(arrivals, order).ravel() for order in (1, 2, 3)])
        np.testing.assert_allclose(seed[planner.velocity_start:], derivatives, rtol=1e-8, atol=1e-6)
