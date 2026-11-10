import numpy as np

from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.best_center_release import make_planner, release_initial


def test_center_release_preserves_a25_curve_and_opens_all_points():
    track, _ = scenario.build_case(scenario.Case(level='a25_f6'))
    free = make_planner(track, True)
    center = make_planner(track, False)
    assert center.velocity_start-center.n==0
    assert free.velocity_start-free.n==25
    for free_seed, center_seed in zip(free.seeds(), center.seeds()):
        np.testing.assert_array_equal(free_seed[:free.n], center_seed[:center.n])
        arrivals = np.cumsum(np.exp(center_seed[:center.n]))[:-1]
        points = np.array([window.state_at(at)[0] for window,at in zip(track.windows,arrivals)])
        np.testing.assert_allclose(np.asarray(free.point_function(free_seed)).T,points,atol=1e-12)
        np.testing.assert_allclose(np.asarray(center.point_function(center_seed)).T,points,atol=1e-12)
        coefficients = np.asarray(center.curve_values(center_seed)).ravel().reshape(center.n,3,8).transpose(0,2,1)
        record = dict(variables=center_seed.tolist(), coefficients=coefficients.tolist(),
            objective=center.objective(center_seed)[0], nominal_time_s=float(np.exp(center_seed[:center.n]).sum()))
        released, audit = release_initial(free, record)
        np.testing.assert_allclose(released,free_seed,rtol=0.,atol=1e-12)
        assert audit['passed'] and audit['all_point_variables_open']
