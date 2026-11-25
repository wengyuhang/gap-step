from __future__ import annotations

import json
from pathlib import Path
import re

import pytest

from nonconvex_timevarying_window.sc_dynatogt.results_manager import (
    apply_migration,
    build_catalog,
    migration_plan,
    timestamped_run_directory,
    verify_migration,
    write_run_manifest,
)


def _json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def _legacy_results(root: Path) -> None:
    _json(
        root / "default/summary.json",
        {"experiments": {name: {"passed": True} for name in ("E0", "E1", "E2", "E3", "E4", "E5")}},
    )
    (root / "default/E0").mkdir(parents=True)
    (root / "default/E0/data.csv").write_text("x,y\n1,2\n", encoding="utf-8")
    _json(
        root / "diverse_paper_irregular_closed/summary.json",
        {
            "demo": "diverse_six_window",
            "passed": True,
            "optimization_success": True,
            "designated_order_legal": True,
            "sampled_dynamic_limits_satisfied": False,
            "mapping_validation": [{"passed": True}] * 6,
            "result": {"total_time": 14.9, "iterations": 385},
        },
    )
    (root / "diverse_paper_irregular_closed/trajectory.csv").write_text(
        "time,px\n0,0\n", encoding="utf-8"
    )
    (root / "diverse_paper_irregular_closed/trajectory.png").write_bytes(b"old-png")
    (root / "diverse_paper_irregular_closed_physical_scene").mkdir(parents=True)
    (root / "diverse_paper_irregular_closed_physical_scene/trajectory.png").write_bytes(
        b"physical-png"
    )
    (root / "diverse_paper_irregular_closed_airsim_style").mkdir(parents=True)
    (root / "diverse_paper_irregular_closed_airsim_style/airsim_chase.mp4").write_bytes(
        b"video"
    )


def test_migration_dry_run_is_read_only_and_apply_is_lossless(tmp_path: Path) -> None:
    root = tmp_path / "results"
    _legacy_results(root)
    before = {
        path.relative_to(root).as_posix(): path.read_bytes()
        for path in root.rglob("*")
        if path.is_file()
    }

    plan = migration_plan(root)
    assert plan["status"] == "planned"
    assert plan["file_count"] == len(before)
    assert not (root / "migration_manifest.json").exists()
    assert {
        path.relative_to(root).as_posix(): path.read_bytes()
        for path in root.rglob("*")
        if path.is_file()
    } == before

    result = apply_migration(root, refresh=False)
    assert result["status"] == "complete"
    assert verify_migration(root)["file_count"] == len(before)
    manifest = json.loads((root / "migration_manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "complete"
    assert len(manifest["moves"]) == len(before)
    assert (root / "experiments/formal/20260714_default/summary.json").is_file()
    featured = root / "demos/runs/20260717_paper_irregular_closed"
    assert (featured / "legacy/original_visuals/trajectory.png").read_bytes() == b"old-png"
    assert (featured / "figures/trajectory_physical.png").read_bytes() == b"physical-png"
    assert (featured / "legacy/original_opengl/opengl_chase.mp4").read_bytes() == b"video"
    assert not (root / "default").exists()

    repeated = apply_migration(root, refresh=False)
    assert repeated["status"] == "verified"


def test_migration_refuses_an_existing_destination(tmp_path: Path) -> None:
    root = tmp_path / "results"
    (root / "default").mkdir(parents=True)
    (root / "default/value.txt").write_text("source", encoding="utf-8")
    destination = root / "experiments/formal/20260714_default/value.txt"
    destination.parent.mkdir(parents=True)
    destination.write_text("collision", encoding="utf-8")
    with pytest.raises(FileExistsError):
        migration_plan(root)


def test_in_progress_migration_resumes_from_verified_journal(tmp_path: Path) -> None:
    root = tmp_path / "results"
    _legacy_results(root)
    plan = migration_plan(root)
    journal = {**plan, "status": "in_progress", "started_at": "2026-07-17T00:00:00+08:00"}
    _json(root / "migration_manifest.json", journal)
    first = plan["moves"][0]
    source = root / first["source"]
    destination = root / first["destination"]
    destination.parent.mkdir(parents=True)
    source.rename(destination)

    resumed = migration_plan(root)
    assert resumed["status"] == "resuming"
    assert apply_migration(root, refresh=False)["status"] == "complete"
    assert verify_migration(root)["status"] == "verified"


def test_catalog_reports_featured_warning_and_legacy_summaries(tmp_path: Path) -> None:
    root = tmp_path / "results"
    featured = root / "demos/runs/featured"
    _json(
        featured / "summary.json",
        {
            "demo": "diverse_six_window",
            "passed": True,
            "optimization_success": True,
            "designated_order_legal": True,
            "sampled_dynamic_limits_satisfied": False,
            "mapping_validation": [{"passed": True}] * 6,
            "result": {"total_time": 12.3, "iterations": 42},
        },
    )
    (featured / "figures").mkdir(parents=True)
    (featured / "figures/route_overview.png").write_bytes(b"png")
    write_run_manifest(
        featured,
        run_id="featured",
        kind="demo",
        role="featured",
        featured=True,
    )
    _json(root / "legacy/summary.json", {"experiments": {"E0": {"passed": True}}})

    catalog = build_catalog(root)
    assert len(catalog["runs"]) == 2
    assert catalog["featured_run"] == "demos/runs/featured"
    page = (root / "index.html").read_text(encoding="utf-8")
    assert "动力学硬上限" in page
    assert "False" in page
    assert "figures/route_overview.png" in page
    markdown = (root / "INDEX.md").read_text(encoding="utf-8")
    for target in re.findall(r'(?:href|src)="([^"]+)"', page):
        assert (root / target).exists(), target
    for target in re.findall(r"\[[^]]+\]\(([^)]+)\)", markdown):
        assert (root / target).exists(), target


def test_current_demo_pointer_overrides_an_older_featured_manifest(tmp_path: Path) -> None:
    root = tmp_path / "results"
    for name, featured in (("old", True), ("new", False)):
        run = root / "demos/runs" / name
        _json(
            run / "summary.json",
            {"demo": "diverse_six_window", "passed": True, "result": {}},
        )
        write_run_manifest(
            run,
            run_id=name,
            kind="demo",
            role="run",
            featured=featured,
        )
    _json(
        root / "current_demo.json",
        {"run": "demos/runs/new", "summary": "demos/runs/new/summary.json"},
    )
    catalog = build_catalog(root)
    assert catalog["featured_run"] == "demos/runs/new"
    assert [run["run_id"] for run in catalog["runs"] if run["featured"]] == ["new"]


def test_timestamped_run_directory_is_sortable_and_avoids_collisions(tmp_path: Path) -> None:
    from datetime import datetime, timezone

    when = datetime(2026, 7, 17, 8, 9, 10, tzinfo=timezone.utc)
    first = timestamped_run_directory(tmp_path, "demos/runs", "paper irregular", when=when)
    assert first.name == "20260717_080910_paper_irregular"
    first.mkdir(parents=True)
    second = timestamped_run_directory(tmp_path, "demos/runs", "paper irregular", when=when)
    assert second.name == "20260717_080910_paper_irregular_02"
