import numpy as np
from convex_timevarying_window.comparisons.single_gate_translation.scenario import Case, build_case
from convex_timevarying_window.dynamic_passage_minco.planner import Config, Planner
from convex_timevarying_window.dynamic_passage_minco.safety import passage, rectangle_distance_squared
from convex_timevarying_window.togt_point_only_mpc.trajectory import Plan


def test_separate_pieces_find_both_sides_of_sweeping_collision():
    track, _ = build_case(Case(3.5, 2., -3.5*np.pi))
    w = track.windows[0]; normal = w.state_at(0.)[1][:, 2]
    coefficients = np.zeros((2, 8, 3))
    coefficients[0, 0] = w.center0+5*normal
    coefficients[1, 0] = w.center0
    coefficients[:, 1] = -5*normal
    plan = Plan(0., 0, np.array([.5, .5]), coefficients, np.zeros(4), 0., 0., 'test', 1)
    parts = [passage(plan, w, .57, piece_indices=(piece,)) for piece in range(2)]
    assert all(not part['passed'] for part in parts)
    assert [part['witness_piece'] for part in parts] == [0, 1]


def test_three_hz_four_metre_fixed_input_regression():
    track, _ = build_case(Case(3., 4., 0.))
    # Uncompiled interpreter regression: numerical correctness, NOT an online
    # timing measurement. Compiled timing is saved in the v6 probe and flight.
    cfg = Config(compile_objective=False, max_solve_ms=2000.)
    planner = Planner(track, cfg)
    plan = planner.solve(np.r_[track.start, np.zeros(9)], .162, 0)
    assert plan.optimizer_stats['converged']
    assert plan.optimizer_stats['passage']['passed']
    assert len(plan.optimizer_stats['passage']['parts']) == 2
    w = track.windows[0]; rotation = w.state_at(0.)[1]
    times = np.arange(plan.made_at, plan.end_time, .0005)
    centers = np.array([w.state_at(float(t))[0] for t in times])
    relative = (plan.sample_many(times)-centers)@rotation
    distance_sq = rectangle_distance_squared(relative,
                    np.max(np.abs(w.aperture.physical_vertices), axis=0)+cfg.frame_offset)
    radius = cfg.radius+cfg.frame_thickness+cfg.passage_margin
    assert distance_sq.min() >= radius**2-1e-7
