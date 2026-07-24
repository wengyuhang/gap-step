from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from nonconvex_timevarying_window.msr_dynatogt.candidate_pool import (
    Candidate,
    CandidatePool,
)
from nonconvex_timevarying_window.msr_dynatogt.config import CandidatePoolConfig
from nonconvex_timevarying_window.msr_dynatogt.initializations import InitialGuess


def _candidate(
    label: int,
    *,
    total_time: float,
    legal: bool,
    dynamic: bool,
    waypoint_shift: float = 0.0,
) -> Candidate:
    result = SimpleNamespace(
        total_time=total_time,
        objective=total_time,
        success=True,
        waypoints=np.array([[waypoint_shift, 0.0, 0.0]]),
        durations=np.array([0.5 * total_time, 0.5 * total_time]),
        iterations=1,
        evaluations=2,
    )
    feasibility = SimpleNamespace(
        window_order_legal=legal,
        window_internal_legal=legal,
        sampled_dynamic_limits_satisfied=dynamic,
    )
    guess = InitialGuess("test", label, label, np.zeros(4))
    return Candidate(
        initialization=guess,
        raw_result=result,
        raw_feasibility=feasibility,
        result=result,
        feasibility=feasibility,
        optimization_seconds=0.1,
    )


def test_pool_prioritizes_legal_then_dynamic_then_time():
    pool = CandidatePool()
    pool.add(_candidate(0, total_time=0.5, legal=False, dynamic=True))
    pool.add(_candidate(1, total_time=0.7, legal=True, dynamic=False))
    pool.add(_candidate(2, total_time=1.2, legal=True, dynamic=True))
    pool.add(_candidate(3, total_time=1.0, legal=True, dynamic=True))
    assert pool.best.initialization.index == 3
    assert [candidate.initialization.index for candidate in pool.candidates] == [3, 2, 1, 0]


def test_feasible_candidate_beats_shorter_rotor_limit_violation():
    pool = CandidatePool()
    pool.add(_candidate(0, total_time=0.4, legal=True, dynamic=False))
    pool.add(_candidate(1, total_time=1.8, legal=True, dynamic=True))
    assert pool.best.initialization.index == 1


def test_pool_deduplicates_nearly_identical_results():
    pool = CandidatePool(
        CandidatePoolConfig(
            time_relative_tolerance=1.0e-3,
            waypoint_absolute_tolerance=1.0e-2,
            duration_absolute_tolerance=1.0e-2,
        )
    )
    pool.add(_candidate(0, total_time=2.0, legal=True, dynamic=True))
    pool.add(
        _candidate(
            1,
            total_time=2.0005,
            legal=True,
            dynamic=True,
            waypoint_shift=1.0e-3,
        )
    )
    assert len(pool) == 1
    assert pool.duplicates_removed == 1
