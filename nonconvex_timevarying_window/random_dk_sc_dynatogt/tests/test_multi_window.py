from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from nonconvex_timevarying_window.rot_sync_sc_togt.geometry import RotatingWindow, basis_from_normal
from nonconvex_timevarying_window.rot_sync_sc_togt.scenarios import RotSyncScenario
from nonconvex_timevarying_window.sc_dynatogt.collision import CuboidBody
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState
from nonconvex_timevarying_window.sc_dynatogt.optimizer import OptimizationConfig
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import k_from_durations

from nonconvex_timevarying_window.random_dk_sc_dynatogt.multi_window import MultiWindowObjective, audit_multi
from nonconvex_timevarying_window.random_dk_sc_dynatogt.safety import screen_candidate
from nonconvex_timevarying_window.random_dk_sc_dynatogt.search import SearchConfig, generate_candidates


def fixture(omega=0.):
    physical = np.array([[-1., -1], [1, -1], [1, 1], [-1, 1]])
    sc = SimpleNamespace(evaluate=lambda z: np.array([z.real, z.imag]), jacobian=lambda _: np.eye(2))
    gate = SimpleNamespace(sc_map=sc, safe_region=SimpleNamespace(distance=.2),
                           safe_polygon=.8*physical, dense_boundary=SimpleNamespace(vertices=physical))
    basis, normal = basis_from_normal([1., 0, 0])
    # Match a monotone rest-to-rest seventh-order polynomial at t=2,4,6.
    # Equally spaced waypoints at equally spaced times can cause backtracking.
    windows = tuple(RotatingWindow(f"w{i}", gate, np.array([x, 0., 0]), basis, normal,
                                   .2*i, omega, 0., .2) for i, x in enumerate((-3.435546875, 0., 3.435546875)))
    scenario = RotSyncScenario("three-test", BoundaryState([-4., 0, 0]), BoundaryState([4., 0, 0]),
                               windows, "test", CuboidBody((.08, .08, .04)))
    objective = MultiWindowObjective(scenario, OptimizationConfig())
    x = np.r_[k_from_durations(np.full(4, 2.)), np.tile([.1, .05], 3)]
    return scenario, objective, x


def test_first_k_changes_all_later_absolute_phases_and_waypoints():
    scenario, objective, x = fixture(omega=1.3)
    original = objective.forward(x)
    changed = x.copy()
    changed[0] += .2
    new = objective.forward(changed)
    assert objective.dimension == 10
    shifts = new.crossing_times-original.crossing_times
    assert shifts[0] > 0
    np.testing.assert_allclose(shifts, np.full(3, shifts[0]), atol=1e-12)
    np.testing.assert_allclose(original.local_points, new.local_points)
    for i, w in enumerate(scenario.windows):
        p = new.trajectory.evaluate(new.crossing_times[i])
        np.testing.assert_allclose(p, w.world_point(new.local_points[i], new.crossing_times[i]), atol=1e-11)
        assert np.linalg.norm(p-original.trajectory.evaluate(original.crossing_times[i])) > 1e-3


def test_multigate_candidates_have_four_k_and_three_independent_d_blocks():
    _, _, x = fixture()
    rows = list(generate_candidates(x, 4, SearchConfig(per_scale=4)))
    assert all(row["x"].shape == (10,) for row in rows)
    both = next(row for row in rows if row["mode"] == "DK")
    assert np.all(np.linalg.norm(both["delta"][4:].reshape(3, 2), axis=1) > 0)
    assert np.all(both["delta"][:4] != 0)


def test_full_screen_checks_all_three_windows_and_reports_later_failure():
    scenario, objective, x = fixture()
    f = objective.forward(x)
    result = screen_candidate(f, scenario, objective.config)
    assert result["passed"] and len(result["spheres"]) == 3
    bad_windows = list(scenario.windows)
    tiny_gate = SimpleNamespace(safe_region=SimpleNamespace(distance=.2),
                                safe_polygon=bad_windows[-1].safe_polygon*.01,
                                dense_boundary=bad_windows[-1].gate.dense_boundary)
    bad_windows[-1] = replace(bad_windows[-1], gate=tiny_gate)
    failure = screen_candidate(f, replace(scenario, windows=tuple(bad_windows)), objective.config)
    assert not failure["passed"]
    assert failure["window_index"] == 2 and len(failure["spheres"]) == 2


def test_final_audit_checks_entire_same_trajectory_for_each_window_and_ands_results():
    scenario, objective, x = fixture()
    f = objective.forward(x)
    calls = []
    def audit_one(subscene, subforward, config):
        i = len(calls)
        assert subforward.trajectory is f.trajectory
        assert len(subscene.windows) == 1
        np.testing.assert_allclose(subforward.local_points, f.local_points[i:i+1])
        np.testing.assert_allclose(subforward.crossing_times, f.crossing_times[i:i+1])
        calls.append(subscene.windows[0].name)
        return dict(trajectory_validation_pass=i != 1), None
    report = audit_multi(scenario, f, objective.config, audit_one=audit_one)
    assert calls == ["w0", "w1", "w2"]
    assert not report["trajectory_validation_pass"]
    assert report["prescribed_order_increasing"]
    passed = audit_multi(scenario, f, objective.config,
                         audit_one=lambda *_: (dict(trajectory_validation_pass=True), None))
    assert passed["trajectory_validation_pass"]
    f.crossing_times = f.crossing_times[::-1]
    wrong_order = audit_multi(scenario, f, objective.config,
                              audit_one=lambda *_: (dict(trajectory_validation_pass=True), None))
    assert not wrong_order["trajectory_validation_pass"]


def test_repeated_middle_plane_crossing_is_rejected_even_when_sphere_safe():
    scenario, _, x = fixture()
    windows = tuple(replace(w, center=np.array([px, 0., 0])) for w, px in zip(scenario.windows, (-2., 0., 2.)))
    scenario = replace(scenario, windows=windows)
    objective = MultiWindowObjective(scenario, OptimizationConfig())
    result = screen_candidate(objective.forward(x), scenario, objective.config)
    assert not result["passed"] and result["reason"] == "crossing_order_or_count"
    assert result["window_index"] == 1
    assert result["spheres"][1]["passed"]
    assert len(result["spheres"][1]["crossings"]) == 3
