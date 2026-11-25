import pytest

from convex_timevarying_window.comparisons.checked_soft_racing.run import conditions
from convex_timevarying_window.comparisons.checked_soft_racing.report import key, paired, selected, stats


def row(phase=0., passed=True, time=25., **extra):
    return dict(rate=1, translation_scale=1, rotation_scale=1, phase=phase,
                realtime_success=passed, physical_success=passed,
                flight_time_s=time, run='test', min_clearance_m=.4, **extra)


def test_union_is_38_unique_conditions_covering_each_original_matrix():
    union = set(conditions())
    assert len(union) == 38
    assert all((r, t, 1, p) in union for r in (1,3,6,10) for t in (1,2,4) for p in (0,1))
    assert all((r, t, t, p) in union for r in (1,10) for t in (1,2,4) for p in (0,1))
    assert all((r, 1, 1, p) in union for r in (1,3,6,10) for p in (0,1,2))


def test_infrastructure_retry_is_not_a_chosen_best_lap():
    failed = row(passed=False, time=None, infrastructure_failure=True)
    final = row(passed=False, time=None, infrastructure_failure=False)
    assert selected([failed, final]) == [final]
    with pytest.raises(ValueError):
        selected([row(time=25.), row(time=24.)])


def test_failed_elapsed_is_not_ranked_as_lap_time():
    new = [row(time=24.), row(phase=1., passed=False, time=3.)]
    old = [row(time=25.), row(phase=1., time=26.)]
    result = paired(new, old)
    assert result['common_successes'] == 1
    assert result['new_faster'] == 1
    assert result['median_new_minus_old_s'] == -1.
    assert result['old_only_success'] == 1
    assert stats(new)['median_success_time_s'] == 24.


def test_phase_and_amplitude_are_part_of_condition():
    r = row()
    assert key(r) != key(dict(r, translation_scale=4.))
    assert key(r) != key(dict(r, rotation_scale=4.))
    assert key(r) != key(dict(r, phase=2.094395102393))
    assert key(r) != key(dict(r, layout=1.))
