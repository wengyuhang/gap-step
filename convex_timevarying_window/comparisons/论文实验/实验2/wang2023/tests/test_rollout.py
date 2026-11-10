import numpy as np
import pytest
from convex_timevarying_window.comparisons.论文实验.实验2.wang2023.rollout import crop, Reference, Settings, outward_normals
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario
from 复现.wang2023_verified.minco import basis


def test_crop_preserves_all_four_physical_states():
    rng = np.random.default_rng(4)
    c = rng.normal(size=(8, 3))
    a, b, duration = .2, .73, 2.4
    transformed = crop(c, a, b)
    for u in np.linspace(0., 1., 11):
        for d in range(4):
            np.testing.assert_allclose(basis(u, d, 4)@transformed/((b-a)*duration)**d,
                basis(a+(b-a)*u, d, 4)@c/duration**d, atol=1e-9)


def test_reference_has_forward_nonstopping_tails_and_same_task_normals():
    from convex_timevarying_window.single_gate_stop_racing.planner import route_signs
    track, _ = scenario.build_case(scenario.Case(level='static'))
    reference = Reference(track, Settings(), scenario.ASSETS/'meshes')
    for i, (window, normal, sign) in enumerate(zip(track.windows, outward_normals(track), route_signs(track))):
        np.testing.assert_array_equal(normal, -sign*window.R[:, 2])
        if i < 6:
            assert (reference.tails[i][:, 0]-window.center0)@normal > reference.body_depths[i]
            assert np.linalg.norm(reference.tails[i][:, 1]) > .1
            np.testing.assert_allclose(np.linalg.norm(reference.tails[i][:, 1]), Settings().seed_speed)
            np.testing.assert_array_equal(reference.tails[i][:, 2:], np.zeros((3, 2)))
    np.testing.assert_allclose(reference.tails[-1][:, 0], track.goal)
    np.testing.assert_array_equal(reference.tails[-1][:, 1:], np.zeros((3, 3)))


@pytest.mark.parametrize('index', [0, 1])
def test_high_dynamic_absolute_clock_center_is_exact(index):
    from dataclasses import replace
    from convex_timevarying_window.comparisons.论文实验.实验2.wang2023.rollout import model_config
    from 复现.wang2023_verified import Planner
    from 复现.wang2023_verified import x500
    track, _ = scenario.build_case(scenario.Case(level='a75_f6'))
    settings = replace(Settings(), compile_functions=False)
    reference = Reference(track, settings, scenario.ASSETS/'meshes')
    window = track.windows[index]
    planner = Planner([lambda t: scenario.pose(window, t)[0]], model_config(settings),
        flatness_function=x500.flatness, yaw=lambda t: 0*t+np.pi/2)
    head = np.zeros((3, 4)); head[:, 0] = track.start
    now = .71
    seed = reference.seed(planner, head, now, index, 1.2)
    trajectory, _, _ = planner.unpack(seed, planner.parameters(head, reference.tails[index], now))
    at = now+trajectory.segment_durations[0]
    np.testing.assert_allclose(trajectory.sample(at), window.state_at(at)[0], atol=1e-8)
