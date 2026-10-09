from convex_timevarying_window.comparisons.checked_soft_mpc_optimized.run import conditions,source_hashes
from convex_timevarying_window.comparisons.checked_soft_mpc_optimized.report import groups,reason


def test_full_matrix_same_union():
    assert len(conditions())==len(set(conditions()))==38
    assert set(conditions())==set.union(*groups().values())


def test_condensed_and_worker_sources_are_frozen():
    hashes=source_hashes()
    assert 'convex_timevarying_window/checked_soft_mpc_isolated/worker.py' in hashes
    assert 'convex_timevarying_window/checked_soft_mpc_isolated/fast/mpc.py' in hashes
    assert 'convex_timevarying_window/checked_soft_mpc_racing/mpc.py' in hashes


def test_nominal_violation_is_not_a_primary_failure_reason():
    assert reason(dict(collision=True,strict_dynamics_success=False))=='门框接触'
    assert reason(dict(failure='timeout',strict_dynamics_success=False))=='60s未完成'
