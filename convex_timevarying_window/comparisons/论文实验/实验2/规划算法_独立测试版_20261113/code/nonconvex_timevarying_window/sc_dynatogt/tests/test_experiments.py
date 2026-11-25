import json

import pytest

from nonconvex_timevarying_window.sc_dynatogt.experiments import (
    ExperimentSettings,
    main,
    run_e0,
    run_e1,
)


def test_e1_smoke_writes_all_six_boundary_families(tmp_path):
    result = run_e1(tmp_path / "E1", ExperimentSettings(suite="smoke"))
    assert result["all_boundaries_have_an_accepted_count"]
    assert len(result["rows"]) == 6
    assert (tmp_path / "E1" / "boundary_sampling.csv").is_file()


def test_e0_regression_meets_one_percent_threshold(tmp_path):
    result = run_e0(tmp_path / "E0", ExperimentSettings(suite="smoke"))
    assert result["passed"]
    assert result["relative_total_time_error"] <= 0.01
    assert (tmp_path / "E0" / "summary.json").is_file()


def test_cli_can_select_one_experiment(tmp_path):
    code = main(["--suite", "smoke", "--experiment", "E1", "--outdir", str(tmp_path)])
    assert code == 0
    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert tuple(summary["experiments"]) == ("E1",)


@pytest.mark.parametrize(
    "kwargs",
    ({"replicates": 0}, {"replicates": -1}, {"mapping_samples": 0}, {"mapping_samples": -1}),
)
def test_experiment_settings_reject_nonpositive_counts(kwargs):
    with pytest.raises(ValueError):
        ExperimentSettings(**kwargs)
