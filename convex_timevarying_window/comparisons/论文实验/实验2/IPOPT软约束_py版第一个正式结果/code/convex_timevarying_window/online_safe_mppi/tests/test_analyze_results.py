import pytest

from convex_timevarying_window.online_safe_mppi.analyze_results import (
    common_successful_pairs,
    validate_paired_records,
)


def test_paired_speed_analysis_excludes_failed_runs_from_lap_time():
    """Catches silently treating an aborted duration as racing performance."""
    proposed=[{"seed":0,"success":False},{"seed":1,"success":True}]
    center=[{"seed":0,"success":True},{"seed":1,"success":True}]
    validate_paired_records(proposed,center)
    pairs=common_successful_pairs(proposed,center)
    assert [(p["seed"],c["seed"]) for p,c in pairs]==[(1,1)]


def test_paired_speed_analysis_requires_matching_seeds():
    proposed=[{"seed":0,"success":True}]
    center=[{"seed":1,"success":True}]
    with pytest.raises(ValueError,match="seed"):
        validate_paired_records(proposed,center)
