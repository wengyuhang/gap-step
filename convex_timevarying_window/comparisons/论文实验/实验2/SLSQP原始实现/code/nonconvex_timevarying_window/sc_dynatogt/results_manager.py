"""Organize, catalogue, and present SC-DynaTOGT experiment artifacts.

The result tree is intentionally ignored by Git, so this module treats every
migration as a small data-management operation: it inventories the legacy
files, records SHA-256 hashes, moves files without copying, and verifies the
destination before declaring the migration complete.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import html
import json
import os
from pathlib import Path
import re
from typing import Any, Iterable


RESULTS_ROOT = Path("nonconvex_timevarying_window/sc_dynatogt/results")
FEATURED_RUN = Path("demos/runs/20260717_paper_irregular_closed")
SCHEMA_VERSION = 1

_DIRECT_MIGRATIONS = {
    "default": "experiments/formal/20260714_default",
    "smoke": "experiments/smoke/20260714_smoke",
    "e1_default": "experiments/diagnostics/e1_sampling_20260714",
    "multiwindow_demo": "experiments/diagnostics/e4_multiwindow_20260714",
    "examples": "diagnostics/examples",
    "default_chunks": "work/chunks/default_20260714",
    "diverse_demo": "demos/archive/spacious_open_20260717",
    "diverse_closed_loop_regular_20260717": "demos/archive/regular_closed_20260717",
}
_FEATURED_SOURCES = (
    "diverse_paper_irregular_closed",
    "diverse_paper_irregular_closed_physical_scene",
    "diverse_paper_irregular_closed_airsim_style",
)


def _now() -> datetime:
    return datetime.now().astimezone()


def _write_json(path: Path, payload: object) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)
    return path


def timestamped_run_directory(
    results_root: str | Path,
    category: str | Path,
    label: str,
    *,
    when: datetime | None = None,
) -> Path:
    """Return a new sortable run directory without creating it."""

    clean_label = re.sub(r"[^a-zA-Z0-9_-]+", "_", label).strip("_")
    if not clean_label:
        raise ValueError("run label must contain at least one path-safe character")
    timestamp = (when or _now()).strftime("%Y%m%d_%H%M%S")
    base = Path(results_root).expanduser() / Path(category) / f"{timestamp}_{clean_label}"
    candidate = base
    suffix = 2
    while candidate.exists():
        candidate = base.with_name(f"{base.name}_{suffix:02d}")
        suffix += 1
    return candidate


def _relative(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _iter_files(root: Path) -> Iterable[Path]:
    if not root.is_dir():
        return ()
    return (path for path in sorted(root.rglob("*")) if path.is_file())


def _featured_destination(relative: Path, source_name: str) -> Path:
    if source_name == "diverse_paper_irregular_closed":
        if relative == Path("summary.json"):
            return FEATURED_RUN / "summary.json"
        if relative == Path("trajectory.csv"):
            return FEATURED_RUN / "data/trajectory.csv"
        if relative.parts and relative.parts[0] == "preprocessed_gates":
            return FEATURED_RUN / "preprocessing" / Path(*relative.parts[1:])
        if relative.name in {"trajectory.png", "dynamic_windows.gif"}:
            return FEATURED_RUN / "legacy/original_visuals" / relative.name
        return FEATURED_RUN / "legacy/original_run" / relative
    if source_name == "diverse_paper_irregular_closed_physical_scene":
        if relative == Path("trajectory.png"):
            return FEATURED_RUN / "figures/trajectory_physical.png"
        if relative == Path("dynamic_windows.gif"):
            return FEATURED_RUN / "media/dynamic_windows.gif"
        return FEATURED_RUN / "legacy/physical_scene" / relative
    if source_name == "diverse_paper_irregular_closed_airsim_style":
        renamed = {
            "airsim_overview.png": "opengl_overview.png",
            "airsim_chase.png": "opengl_chase.png",
            "airsim_chase.mp4": "opengl_chase.mp4",
        }
        return FEATURED_RUN / "legacy/original_opengl" / renamed.get(
            relative.name, relative.name
        )
    raise ValueError(f"unknown featured source: {source_name}")


def migration_plan(results_root: str | Path = RESULTS_ROOT) -> dict[str, Any]:
    """Build and validate the legacy-to-organized file mapping."""

    root = Path(results_root).expanduser()
    manifest_path = root / "migration_manifest.json"
    legacy_roots = [root / name for name in (*_DIRECT_MIGRATIONS, *_FEATURED_SOURCES)]
    sources_present = any(path.exists() for path in legacy_roots)
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest.get("status") == "complete" and not sources_present:
            return {
                "status": "already_complete",
                "root": str(root),
                "file_count": manifest.get("file_count", 0),
                "total_bytes": manifest.get("total_bytes", 0),
                "moves": manifest.get("moves", []),
            }
        if manifest.get("status") == "complete" and sources_present:
            raise RuntimeError(
                "a completed migration manifest exists but legacy source directories reappeared"
            )
        if manifest.get("status") == "in_progress":
            for move in manifest.get("moves", []):
                source = root / str(move["source"])
                destination = root / str(move["destination"])
                if source.exists() and destination.exists():
                    raise RuntimeError(
                        f"both source and destination exist during migration resume: {source}"
                    )
                if not source.exists() and not destination.exists():
                    raise RuntimeError(
                        f"both source and destination are missing during migration resume: {source}"
                    )
                if destination.is_file():
                    if destination.stat().st_size != int(move["size"]):
                        raise RuntimeError(f"resumed destination size mismatch: {destination}")
                    if _sha256(destination) != str(move["sha256"]):
                        raise RuntimeError(f"resumed destination hash mismatch: {destination}")
            return {**manifest, "status": "resuming"}

    moves: list[dict[str, Any]] = []
    for source_name, destination_name in _DIRECT_MIGRATIONS.items():
        source_root = root / source_name
        for source in _iter_files(source_root):
            destination = root / destination_name / source.relative_to(source_root)
            moves.append(
                {
                    "source": _relative(source, root),
                    "destination": _relative(destination, root),
                    "size": source.stat().st_size,
                    "sha256": _sha256(source),
                }
            )
    for source_name in _FEATURED_SOURCES:
        source_root = root / source_name
        for source in _iter_files(source_root):
            relative = source.relative_to(source_root)
            destination = root / _featured_destination(relative, source_name)
            moves.append(
                {
                    "source": _relative(source, root),
                    "destination": _relative(destination, root),
                    "size": source.stat().st_size,
                    "sha256": _sha256(source),
                }
            )

    if not moves:
        raise FileNotFoundError(f"no legacy SC-DynaTOGT results found below {root}")
    destinations = [move["destination"] for move in moves]
    if len(destinations) != len(set(destinations)):
        raise RuntimeError("migration plan contains duplicate destinations")
    for move in moves:
        destination = root / move["destination"]
        if destination.exists():
            raise FileExistsError(f"migration destination already exists: {destination}")
    return {
        "status": "planned",
        "schema_version": SCHEMA_VERSION,
        "root": str(root),
        "file_count": len(moves),
        "total_bytes": sum(int(move["size"]) for move in moves),
        "moves": moves,
    }


def _verify_moves(root: Path, moves: Iterable[dict[str, Any]]) -> None:
    for move in moves:
        source = root / str(move["source"])
        destination = root / str(move["destination"])
        if source.exists():
            raise RuntimeError(f"source still exists after migration: {source}")
        if not destination.is_file():
            raise RuntimeError(f"missing migrated destination: {destination}")
        if destination.stat().st_size != int(move["size"]):
            raise RuntimeError(f"size mismatch after migration: {destination}")
        if _sha256(destination) != str(move["sha256"]):
            raise RuntimeError(f"SHA-256 mismatch after migration: {destination}")


def verify_migration(results_root: str | Path = RESULTS_ROOT) -> dict[str, Any]:
    root = Path(results_root).expanduser()
    manifest_path = root / "migration_manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"migration manifest not found: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("status") != "complete":
        raise RuntimeError("migration manifest is not complete")
    _verify_moves(root, manifest["moves"])
    return {
        "status": "verified",
        "file_count": manifest["file_count"],
        "total_bytes": manifest["total_bytes"],
    }


def _remove_empty_legacy_directories(root: Path) -> None:
    names = (*_DIRECT_MIGRATIONS, *_FEATURED_SOURCES)
    for name in names:
        source_root = root / name
        if not source_root.exists():
            continue
        directories = [path for path in source_root.rglob("*") if path.is_dir()]
        for directory in sorted(directories, key=lambda value: len(value.parts), reverse=True):
            try:
                directory.rmdir()
            except OSError:
                pass
        try:
            source_root.rmdir()
        except OSError:
            pass


def _artifact_paths(run_root: Path) -> list[str]:
    suffixes = {".json", ".csv", ".png", ".gif", ".mp4", ".npz"}
    excluded = {"run_manifest.json"}
    return [
        path.relative_to(run_root).as_posix()
        for path in sorted(run_root.rglob("*"))
        if path.is_file() and path.suffix.lower() in suffixes and path.name not in excluded
    ]


def write_run_manifest(
    run_root: str | Path,
    *,
    run_id: str,
    kind: str,
    role: str,
    summary: str | None = "summary.json",
    featured: bool = False,
    migrated_from: Iterable[str] = (),
) -> Path:
    root = Path(run_root).expanduser()
    payload = {
        "schema_version": SCHEMA_VERSION,
        "run_id": run_id,
        "kind": kind,
        "role": role,
        "featured": bool(featured),
        "created_at": _now().isoformat(timespec="seconds"),
        "summary": summary,
        "migrated_from": list(migrated_from),
        "artifacts": _artifact_paths(root),
    }
    return _write_json(root / "run_manifest.json", payload)


def refresh_run_manifest(run_root: str | Path) -> Path:
    """Refresh an existing manifest's artifact list without changing its identity."""

    root = Path(run_root).expanduser()
    path = root / "run_manifest.json"
    if not path.is_file():
        raise FileNotFoundError(f"run manifest not found: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["updated_at"] = _now().isoformat(timespec="seconds")
    payload["artifacts"] = _artifact_paths(root)
    return _write_json(path, payload)


def _write_migrated_run_manifests(root: Path) -> list[str]:
    definitions = (
        ("experiments/formal/20260714_default", "20260714_default", "experiment", "formal", False, ("default",)),
        ("experiments/smoke/20260714_smoke", "20260714_smoke", "experiment", "smoke", False, ("smoke",)),
        ("experiments/diagnostics/e1_sampling_20260714", "20260714_e1_sampling", "experiment", "diagnostic", False, ("e1_default",)),
        ("experiments/diagnostics/e4_multiwindow_20260714", "20260714_e4_multiwindow", "experiment", "diagnostic", False, ("multiwindow_demo",)),
        ("demos/archive/spacious_open_20260717", "20260717_spacious_open", "demo", "archive", False, ("diverse_demo",)),
        ("demos/archive/regular_closed_20260717", "20260717_regular_closed", "demo", "archive", False, ("diverse_closed_loop_regular_20260717",)),
        (str(FEATURED_RUN), "20260717_paper_irregular_closed", "demo", "featured", True, _FEATURED_SOURCES),
        ("diagnostics/examples", "examples", "collection", "diagnostic", False, ("examples",)),
        ("work/chunks/default_20260714", "20260714_default_chunks", "collection", "work", False, ("default_chunks",)),
    )
    outputs: list[str] = []
    for relative, run_id, kind, role, featured, sources in definitions:
        run_root = root / relative
        if not run_root.is_dir():
            continue
        summary = "summary.json" if (run_root / "summary.json").is_file() else None
        path = write_run_manifest(
            run_root,
            run_id=run_id,
            kind=kind,
            role=role,
            summary=summary,
            featured=featured,
            migrated_from=sources,
        )
        outputs.append(_relative(path, root))
    return outputs


def update_current_demo(results_root: str | Path, run_root: str | Path) -> Path:
    root = Path(results_root).expanduser().resolve()
    run = Path(run_root).expanduser().resolve()
    if root != run and root not in run.parents:
        raise ValueError("current demo must be located below the results root")
    summary = run / "summary.json"
    if not summary.is_file():
        raise FileNotFoundError(f"current demo summary not found: {summary}")
    payload = {
        "schema_version": SCHEMA_VERSION,
        "updated_at": _now().isoformat(timespec="seconds"),
        "run": run.relative_to(root).as_posix(),
        "summary": summary.relative_to(root).as_posix(),
    }
    return _write_json(root / "current_demo.json", payload)


def resolve_current_demo_summary(results_root: str | Path = RESULTS_ROOT) -> Path:
    root = Path(results_root).expanduser()
    pointer = root / "current_demo.json"
    if not pointer.is_file():
        raise FileNotFoundError(
            f"current demo pointer not found: {pointer}; pass --summary explicitly"
        )
    payload = json.loads(pointer.read_text(encoding="utf-8"))
    summary = root / str(payload["summary"])
    if not summary.is_file():
        raise FileNotFoundError(f"current demo summary does not exist: {summary}")
    return summary


def _summary_status(payload: dict[str, Any]) -> dict[str, Any]:
    if payload.get("demo"):
        result = payload.get("result", {})
        validations = payload.get("mapping_validation", [])
        return {
            "passed": bool(payload.get("passed", False)),
            "optimization_success": bool(payload.get("optimization_success", False)),
            "designated_order_legal": bool(payload.get("designated_order_legal", False)),
            "sampled_dynamic_limits_satisfied": bool(
                payload.get("sampled_dynamic_limits_satisfied", False)
            ),
            "mapping_passed": sum(bool(item.get("passed", False)) for item in validations),
            "mapping_total": len(validations),
            "total_time": result.get("total_time"),
            "iterations": result.get("iterations"),
        }
    experiments = payload.get("experiments")
    if isinstance(experiments, dict):
        statuses = {
            name: bool(value.get("passed", False))
            for name, value in experiments.items()
            if isinstance(value, dict)
        }
        return {"passed": bool(statuses) and all(statuses.values()), "experiments": statuses}
    return {"passed": bool(payload.get("passed", False))}


def _load_catalog_entry(results_root: Path, manifest_path: Path) -> dict[str, Any]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    run_root = manifest_path.parent
    summary_name = manifest.get("summary")
    summary_payload: dict[str, Any] = {}
    if summary_name and (run_root / str(summary_name)).is_file():
        summary_payload = json.loads((run_root / str(summary_name)).read_text(encoding="utf-8"))
    return {
        "run_id": manifest.get("run_id", run_root.name),
        "path": _relative(run_root, results_root),
        "kind": manifest.get("kind", "unknown"),
        "role": manifest.get("role", "unknown"),
        "featured": bool(manifest.get("featured", False)),
        "status": _summary_status(summary_payload),
        "artifacts": _artifact_paths(run_root),
    }


def discover_runs(results_root: str | Path = RESULTS_ROOT) -> list[dict[str, Any]]:
    """Discover manifest-backed runs plus unmigrated legacy summaries."""

    root = Path(results_root).expanduser()
    entries = [_load_catalog_entry(root, path) for path in sorted(root.rglob("run_manifest.json"))]
    manifest_roots = [root / entry["path"] for entry in entries]
    known_paths = {entry["path"] for entry in entries}
    for summary in sorted(root.rglob("summary.json")):
        if any(run == summary.parent or run in summary.parents for run in manifest_roots):
            continue
        payload = json.loads(summary.read_text(encoding="utf-8"))
        if not payload.get("demo") and not isinstance(payload.get("experiments"), dict):
            continue
        relative = _relative(summary.parent, root)
        if relative in known_paths:
            continue
        entries.append(
            {
                "run_id": summary.parent.name,
                "path": relative,
                "kind": "demo" if payload.get("demo") else "experiment",
                "role": "legacy",
                "featured": False,
                "status": _summary_status(payload),
                "artifacts": _artifact_paths(summary.parent),
            }
        )
    return sorted(entries, key=lambda item: (not item["featured"], item["role"], item["run_id"]))


def _artifact_link(run: dict[str, Any], ending: str) -> str | None:
    for artifact in run.get("artifacts", []):
        if str(artifact).endswith(ending):
            return f"{run['path']}/{artifact}"
    return None


def _markdown_report(catalog: dict[str, Any]) -> str:
    lines = ["# SC-DynaTOGT 实验结果", "", f"更新时间：{catalog['generated_at']}", ""]
    featured = next((run for run in catalog["runs"] if run["featured"]), None)
    if featured:
        status = featured["status"]
        lines.extend(
            [
                "## 精选不规则闭环", "",
                f"- 运行：`{featured['run_id']}`",
                f"- 总时间：`{status.get('total_time', 'n/a')}` s；迭代：`{status.get('iterations', 'n/a')}`",
                f"- 优化收敛：`{status.get('optimization_success')}`；指定顺序合法：`{status.get('designated_order_legal')}`",
                f"- 动力学硬上限：`{status.get('sampled_dynamic_limits_satisfied')}`（为 `false` 时必须明确报告）",
                "",
            ]
        )
        for label, ending in (
            ("干净总览", "figures/route_overview.png"),
            ("六窗口局部图", "figures/crossings_grid.png"),
            ("缩放曲线", "figures/scale_profile.png"),
            ("OpenGL 视频", "opengl/opengl_chase.mp4"),
        ):
            link = _artifact_link(featured, ending)
            if link:
                lines.append(f"- [{label}]({link})")
        lines.append("")
    lines.extend(["## 运行目录", "", "| 类型 | 运行 | 状态 | 路径 |", "|---|---|---|---|"])
    for run in catalog["runs"]:
        lines.append(
            f"| {run['role']} | {run['run_id']} | {run['status'].get('passed', False)} | "
            f"[{run['path']}]({run['path']}/) |"
        )
    return "\n".join(lines) + "\n"


def _html_report(catalog: dict[str, Any]) -> str:
    featured = next((run for run in catalog["runs"] if run["featured"]), None)
    featured_html = "<p>尚未指定精选运行。</p>"
    if featured:
        status = featured["status"]
        cards = (
            ("优化收敛", status.get("optimization_success")),
            ("穿越顺序合法", status.get("designated_order_legal")),
            ("SC 映射", f"{status.get('mapping_passed', 0)}/{status.get('mapping_total', 0)}"),
            ("动力学硬上限", status.get("sampled_dynamic_limits_satisfied")),
        )
        card_html = "".join(
            f'<div class="card {"warn" if label == "动力学硬上限" and value is False else ""}">'
            f"<small>{html.escape(label)}</small><strong>{html.escape(str(value))}</strong></div>"
            for label, value in cards
        )
        gallery = []
        for label, ending in (
            ("干净航线总览", "figures/route_overview.png"),
            ("六窗口固定视距局部图", "figures/crossings_grid.png"),
            ("窗口缩放曲线", "figures/scale_profile.png"),
        ):
            link = _artifact_link(featured, ending)
            if link:
                gallery.append(
                    f'<figure><a href="{html.escape(link)}"><img src="{html.escape(link)}" '
                    f'alt="{html.escape(label)}"></a><figcaption>{html.escape(label)}</figcaption></figure>'
                )
        video = _artifact_link(featured, "opengl/opengl_chase.mp4")
        video_html = f'<a class="button" href="{html.escape(video)}">查看 OpenGL 追踪视频</a>' if video else ""
        featured_html = (
            f"<h2>精选不规则闭环</h2><p><code>{html.escape(featured['run_id'])}</code> · "
            f"总时间 {status.get('total_time', 'n/a')} s · {status.get('iterations', 'n/a')} 次迭代</p>"
            f'<div class="cards">{card_html}</div>{video_html}<div class="gallery">{"".join(gallery)}</div>'
        )

    formal = next((run for run in catalog["runs"] if run["role"] == "formal"), None)
    experiment_rows = ""
    if formal:
        for name, passed in formal["status"].get("experiments", {}).items():
            experiment_rows += f"<tr><td>{html.escape(name)}</td><td>{'通过' if passed else '失败'}</td></tr>"
    grouped_rows = ""
    for run in catalog["runs"]:
        grouped_rows += (
            f"<tr><td>{html.escape(str(run['role']))}</td><td>{html.escape(str(run['run_id']))}</td>"
            f"<td>{'通过' if run['status'].get('passed') else '未通过/不适用'}</td>"
            f'<td><a href="{html.escape(run["path"])}">打开目录</a></td></tr>'
        )
    return f"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>SC-DynaTOGT 实验结果</title><style>
body{{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;margin:0;background:#f4f7fa;color:#17202a}}
main{{max-width:1180px;margin:auto;padding:32px}} h1,h2{{margin-bottom:.35em}} code{{background:#e8eef3;padding:.18em .4em;border-radius:4px}}
.cards{{display:grid;grid-template-columns:repeat(auto-fit,minmax(165px,1fr));gap:12px;margin:20px 0}}
.card{{background:white;border-left:5px solid #21a179;padding:14px;border-radius:8px;box-shadow:0 2px 12px #16324f16}}
.card.warn{{border-color:#e67e22;background:#fff7eb}} .card small,.card strong{{display:block}} .card strong{{font-size:1.25rem;margin-top:5px}}
.gallery{{display:grid;grid-template-columns:repeat(auto-fit,minmax(310px,1fr));gap:18px;margin:24px 0}}
figure{{margin:0;background:white;padding:10px;border-radius:9px;box-shadow:0 2px 12px #16324f16}} img{{width:100%;height:260px;object-fit:contain}}
figcaption{{padding:8px 4px}} table{{border-collapse:collapse;width:100%;background:white;margin-bottom:24px}} th,td{{padding:10px;border:1px solid #d7e0e7;text-align:left}}
.button{{display:inline-block;background:#1769aa;color:white;text-decoration:none;padding:10px 14px;border-radius:6px}} details{{margin-top:20px}}
</style></head><body><main><h1>SC-DynaTOGT 实验结果</h1><p>更新时间：{html.escape(catalog['generated_at'])}</p>
{featured_html}
<h2>E0–E5 正式实验</h2><table><thead><tr><th>实验</th><th>状态</th></tr></thead><tbody>{experiment_rows}</tbody></table>
<details><summary>全部运行、历史版本与中间结果</summary><table><thead><tr><th>类别</th><th>运行</th><th>状态</th><th>目录</th></tr></thead><tbody>{grouped_rows}</tbody></table></details>
</main></body></html>"""


def build_catalog(results_root: str | Path = RESULTS_ROOT) -> dict[str, Any]:
    root = Path(results_root).expanduser()
    runs = discover_runs(root)
    pointer = root / "current_demo.json"
    if pointer.is_file():
        current_path = str(json.loads(pointer.read_text(encoding="utf-8")).get("run", ""))
        if any(run["path"] == current_path for run in runs):
            for run in runs:
                run["featured"] = run["path"] == current_path
    catalog = {
        "schema_version": SCHEMA_VERSION,
        "generated_at": _now().isoformat(timespec="seconds"),
        "featured_run": next((run["path"] for run in runs if run["featured"]), None),
        "runs": runs,
    }
    _write_json(root / "catalog.json", catalog)
    (root / "INDEX.md").write_text(_markdown_report(catalog), encoding="utf-8")
    (root / "index.html").write_text(_html_report(catalog), encoding="utf-8")
    return catalog


def refresh_featured_figures(results_root: str | Path = RESULTS_ROOT) -> list[str]:
    """Reconstruct the saved featured demo and create presentation figures."""

    from .diverse_demo import load_diverse_demo
    from .visualization import plot_crossing_grid, plot_route_overview, plot_scale_profile

    root = Path(results_root).expanduser()
    run_root = root / FEATURED_RUN
    track, trajectory = load_diverse_demo(run_root / "summary.json")
    outputs = [
        plot_route_overview(track, trajectory, run_root / "figures/route_overview.png"),
        plot_crossing_grid(track, trajectory, run_root / "figures/crossings_grid.png"),
        plot_scale_profile(track, trajectory, run_root / "figures/scale_profile.png"),
    ]
    return [_relative(path, root) for path in outputs]


def apply_migration(
    results_root: str | Path = RESULTS_ROOT,
    *,
    refresh: bool = True,
) -> dict[str, Any]:
    root = Path(results_root).expanduser()
    plan = migration_plan(root)
    if plan["status"] == "already_complete":
        verification = verify_migration(root)
        if refresh:
            build_catalog(root)
        return {
            **verification,
            "root": str(root),
            "already_complete": True,
        }

    manifest = {
        **plan,
        "status": "in_progress",
        "started_at": _now().isoformat(timespec="seconds"),
    }
    _write_json(root / "migration_manifest.json", manifest)
    for move in plan["moves"]:
        source = root / move["source"]
        destination = root / move["destination"]
        if destination.is_file() and not source.exists():
            if destination.stat().st_size == move["size"] and _sha256(destination) == move["sha256"]:
                continue
            raise RuntimeError(f"existing destination does not match migration journal: {destination}")
        if not source.is_file():
            raise FileNotFoundError(f"migration source disappeared: {source}")
        if destination.exists():
            raise FileExistsError(f"migration destination already exists: {destination}")
        destination.parent.mkdir(parents=True, exist_ok=True)
        source.rename(destination)

    _remove_empty_legacy_directories(root)
    _verify_moves(root, plan["moves"])
    generated = _write_migrated_run_manifests(root)
    update_current_demo(root, root / FEATURED_RUN)
    generated.append("current_demo.json")
    if refresh:
        generated.extend(refresh_featured_figures(root))
        _write_migrated_run_manifests(root)
        build_catalog(root)
        generated.extend(["catalog.json", "INDEX.md", "index.html"])
    completed = {
        **manifest,
        "status": "complete",
        "completed_at": _now().isoformat(timespec="seconds"),
        "generated_files": sorted(set(generated)),
    }
    _write_json(root / "migration_manifest.json", completed)
    verify_migration(root)
    return {
        "status": "complete",
        "file_count": plan["file_count"],
        "total_bytes": plan["total_bytes"],
        "generated_files": completed["generated_files"],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Organize and catalogue SC-DynaTOGT results")
    subparsers = parser.add_subparsers(dest="command", required=True)
    migrate = subparsers.add_parser("migrate", help="plan or apply the lossless legacy migration")
    migrate.add_argument("--root", type=Path, default=RESULTS_ROOT)
    migrate.add_argument("--apply", action="store_true", help="apply the migration; default is dry-run")
    migrate.add_argument("--no-refresh", action="store_true", help="skip figures and result index")
    migrate.add_argument("--verbose", action="store_true", help="include every planned file move")
    index = subparsers.add_parser("index", help="regenerate catalog.json, INDEX.md, and index.html")
    index.add_argument("--root", type=Path, default=RESULTS_ROOT)
    verify = subparsers.add_parser("verify", help="verify every migrated file against its SHA-256")
    verify.add_argument("--root", type=Path, default=RESULTS_ROOT)
    args = parser.parse_args(argv)

    if args.command == "migrate":
        result = (
            apply_migration(args.root, refresh=not args.no_refresh)
            if args.apply
            else migration_plan(args.root)
        )
        if not args.verbose:
            result = {key: value for key, value in result.items() if key != "moves"}
    elif args.command == "index":
        catalog = build_catalog(args.root)
        result = {
            "status": "indexed",
            "root": str(args.root),
            "run_count": len(catalog["runs"]),
            "featured_run": catalog["featured_run"],
            "outputs": ["catalog.json", "INDEX.md", "index.html"],
        }
    else:
        result = verify_migration(args.root)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "FEATURED_RUN",
    "RESULTS_ROOT",
    "apply_migration",
    "build_catalog",
    "discover_runs",
    "main",
    "migration_plan",
    "refresh_run_manifest",
    "refresh_featured_figures",
    "resolve_current_demo_summary",
    "timestamped_run_directory",
    "update_current_demo",
    "verify_migration",
    "write_run_manifest",
]
