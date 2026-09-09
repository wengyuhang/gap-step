from types import SimpleNamespace

import numpy as np

from nonconvex_timevarying_window.feasibility_guided_cem_sc_dynatogt.search import (
    CEMConfig,
    PhaseFrontEndConfig,
    feasible_rank,
    polar_decode,
    polar_encode,
    proposal_key,
)


def test_polar_coordinates_round_trip_native_k_and_d():
    x = np.array([.2, -.4, 1.1, .7, 7., -2., .3, .8, -4., -9.])
    np.testing.assert_allclose(polar_decode(polar_encode(x, 4), 4), x, atol=1e-12)


def test_final_rank_discards_every_failed_candidate():
    rows = [dict(id=1, flight_time=1., screen=dict(passed=False)),
            dict(id=2, flight_time=3., screen=dict(passed=True)),
            dict(id=3, flight_time=2., screen=dict(passed=True))]
    assert [row["id"] for row in feasible_rank(rows)] == [3, 2]
    assert feasible_rank(rows[:1]) == []


def test_proposal_evidence_cannot_outrank_a_hard_pass():
    failed = dict(passed=False, reason="dynamics_velocity", spheres=[dict(passed=True)] * 3,
                  dynamics=dict(max_velocity=7.000001))
    passed = dict(passed=True, reason="pass")
    assert proposal_key(passed, 100.) > proposal_key(failed, 1.)


def test_seven_window_partial_geometry_cannot_outrank_dynamics_only_failure():
    partial = dict(
        passed=False,
        reason="sphere_violated",
        spheres=[dict(passed=True)] * 6 + [dict(passed=False, minimum_margin=-1e-6)],
    )
    dynamics = dict(
        passed=False,
        reason="dynamics_velocity",
        spheres=[dict(passed=True)] * 7,
        dynamics=dict(max_velocity=7.1),
    )
    assert proposal_key(dynamics, 10.0) > proposal_key(partial, 9.0)


def test_configs_reject_degenerate_population_and_accept_frozen_protocol():
    CEMConfig()
    PhaseFrontEndConfig()
    try:
        CEMConfig(population=16, elite=16)
    except ValueError:
        pass
    else:
        raise AssertionError("degenerate elite population accepted")
