from __future__ import annotations

import csv
import json

from nonconvex_timevarying_window.msr_dynatogt import experiments


def test_smoke_experiment_completes_and_writes_required_artifacts(
    tmp_path, monkeypatch, static_scenario
):
    monkeypatch.setattr(
        experiments,
        "build_scenes",
        lambda suite: {"static_single": static_scenario},
    )
    output = experiments.run_suite(
        "smoke",
        seed=0,
        seed_count=1,
        selected_scenes=("static_single",),
        results_root=tmp_path,
    )
    required = (
        "config.json",
        "runs.csv",
        "summary.json",
        "REPORT.md",
        "FIGURE_EXPLANATIONS.md",
        "status.json",
        "figures/trajectory_comparison_static_single.png",
        "figures/total_time_comparison.png",
        "figures/computation_time_comparison.png",
        "figures/sampled_dynamic_feasibility_rate.png",
        "figures/repair_thrust_before_after.png",
    )
    assert all((output / relative).is_file() for relative in required)
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    assert summary["experiment_completed"] is True
    assert summary["completed_tasks"] == 1
    assert "连续时间严格证明" in summary["feasibility_scope"]
    explanations = (output / "FIGURE_EXPLANATIONS.md").read_text(encoding="utf-8")
    assert "trajectory_comparison_static_single.png" in explanations
    assert "total_time_comparison.png" in explanations
    assert "computation_time_comparison.png" in explanations
    assert "sampled_dynamic_feasibility_rate.png" in explanations
    assert "repair_thrust_before_after.png" in explanations
    with (output / "runs.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 12
    assert {row["comparison_protocol"] for row in rows} == {
        "native",
        "matched_starts",
        "matched_time",
    }
