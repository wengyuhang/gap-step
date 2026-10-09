import pytest
from convex_timevarying_window.comparisons.wang_free_state_mpc.run import (
    conditions, condition_rows, apply_passage, source_hashes)


def test_matrix_is_exact_previous_union():
    matrix = conditions()
    center = {(r,t,1,p) for r in (1,3,6,10) for t in (1,2,4) for p in (0,1)}
    amplitude = {(r,a,a,p) for r in (1,10) for a in (1,2,4) for p in (0,1)}
    amplitude |= {(10,t,a,p) for t,a in ((4,1),(1,4)) for p in (0,1)}
    rate = {(r,1,1,p) for r in (1,3,6,10) for p in (0,1,2)}
    assert len(matrix) == len(set(matrix)) == 38
    assert set(matrix) == center | amplitude | rate


def test_event_failure_cannot_be_accepted_as_fast_lap():
    row = dict(physical_success=True, realtime_success=True, strict_dynamics_success=True)
    result = apply_passage(row, dict(passed=False, violations=['reverse']))
    assert result['legacy_realtime_success']
    assert not result['realtime_success'] and not result['physical_success']
    assert not result['strict_dynamics_success']
    assert row['realtime_success']  # Preserve input evidence.


def test_no_algorithm_retry_selection():
    first = dict(condition_id=[1,1,1,0], attempt=1, infrastructure_failure=False)
    second = dict(first, attempt=2)
    with pytest.raises(ValueError):
        condition_rows([first,second])
    assert condition_rows([dict(first,infrastructure_failure=True),second]) == [second]


def test_flight_sources_are_exact_event_v4():
    assert len(source_hashes()) == 65
