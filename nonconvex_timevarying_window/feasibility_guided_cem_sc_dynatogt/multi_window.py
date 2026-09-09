"""Run phase-seeded feasibility-guided CEM on the frozen three-U track."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from nonconvex_timevarying_window.random_dk_sc_dynatogt.experiment import jsonable, write_json
from nonconvex_timevarying_window.random_dk_sc_dynatogt.multi_window import (
    MultiWindowObjective,
    audit_multi,
    build_three_u_track,
)

from .search import CEMConfig, PhaseFrontEndConfig, feasible_rank, geometry_complete, local_cem_search, phase_front_end


HERE = Path(__file__).resolve().parent
RANDOM_RESULTS = HERE.parent / "random_dk_sc_dynatogt" / "results"
DEFAULT_BASELINE = RANDOM_RESULTS / "three_u_native_seed20260909_4000" / "baseline.json"
DEFAULT_TEMPLATES = (
    RANDOM_RESULTS / "u_w18_native_expanded_seed20260909_4000" / "result.json",
    RANDOM_RESULTS / "u_w18_fixed_warm_expanded_seed20260909_4000" / "result.json",
)


def scene_record(scenario):
    return dict(name=scenario.name, start=scenario.start_state.matrix, goal=scenario.goal_state.matrix,
                body_half_extents=scenario.body.half_extents, order=[w.name for w in scenario.windows],
                windows=[dict(name=w.name, center=w.center, normal=w.normal, plane_basis=w.plane_basis,
                              omega=w.omega, phase=w.theta0, thickness=w.thickness, radius=w.rho,
                              physical_polygon=w.physical_polygon, safe_polygon=w.safe_polygon)
                         for w in scenario.windows])


def load_template(path):
    result = json.loads(Path(path).read_text())
    selected = result.get("selected")
    if not selected or not selected.get("screen", {}).get("passed"):
        raise ValueError(f"template has no hard-screen-passing selected trajectory: {path}")
    source_scene = json.loads((Path(path).parent / "scene.json").read_text())
    window = source_scene["windows"][0] if "windows" in source_scene else source_scene
    crossing = float(selected["screen"]["spheres"][0]["crossings"][0])
    phase = (float(window["phase"]) + float(window["omega"]) * crossing) % (2 * np.pi)
    return dict(source=str(Path(path).resolve()), phase=phase, d=np.asarray(selected["x"][-2:], dtype=float),
                source_flight_time=float(selected["flight_time"]))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--baseline-json", type=Path, default=DEFAULT_BASELINE)
    parser.add_argument("--template-result", type=Path, nargs=2, default=DEFAULT_TEMPLATES)
    parser.add_argument("--population", type=int, default=256)
    parser.add_argument("--maximum-rounds", type=int, default=20)
    parser.add_argument("--post-feasible-rounds", type=int, default=1)
    args = parser.parse_args(argv)
    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()

    scenario, dynamic_config, _ = build_three_u_track()
    objective = MultiWindowObjective(scenario, dynamic_config)
    scene = scene_record(scenario)
    write_json(output / "scene.json", scene)
    digest = hashlib.sha256(json.dumps(jsonable(scene), sort_keys=True).encode()).hexdigest()
    baseline = json.loads(args.baseline_json.read_text())
    if baseline["scene_sha256"] != digest or baseline["config"] != jsonable(asdict(dynamic_config)):
        raise ValueError("baseline scene/config mismatch")
    center = np.asarray(baseline["x"], dtype=float)
    templates = [load_template(path) for path in args.template_result]
    front_config = PhaseFrontEndConfig()
    cem_config = CEMConfig(population=args.population, maximum_rounds=args.maximum_rounds,
                           post_feasible_rounds=args.post_feasible_rounds)
    write_json(output / "protocol.json", dict(
        algorithm="phase-seeded feasibility-guided full-covariance CEM",
        baseline=str(args.baseline_json.resolve()), templates=templates,
        phase_front_end=asdict(front_config), cem=asdict(cem_config),
        intermediate="all-window sphere contact intervals, order/count, native sampled dynamics",
        selection="flight time among hard-screen passes only",
        final="oriented cuboid audit only for sorted hard-screen passes"))

    rows = []
    with (output / "candidates.jsonl").open("x") as log:
        def record(row):
            rows.append(row)
            log.write(json.dumps(jsonable(row), ensure_ascii=False, allow_nan=False) + "\n")
            log.flush()
            if (row["id"] + 1) % 250 == 0:
                print(f"evaluated {row['id'] + 1}: {row['stage']} {row['screen']['reason']}", flush=True)

        front_rows, next_id = phase_front_end(objective, center, scenario, dynamic_config, templates,
                                               front_config, on_record=record)
        ranked = feasible_rank(front_rows)
        candidates = [row for row in front_rows if geometry_complete(row, len(scenario.windows))]
        if not ranked and not candidates:
            raise RuntimeError("phase front end found no complete three-window geometry seed")
        seed = ranked[0] if ranked else min(candidates,
            key=lambda row: row["screen"].get("dynamics", {}).get("max_velocity", np.inf))
        print(f"phase front end: {len(front_rows)} evaluations, seed vmax="
              f"{seed['screen'].get('dynamics', {}).get('max_velocity')}", flush=True)
        cem_rows, cem_ranked, rounds, _ = local_cem_search(
            objective, seed["x"], scenario, dynamic_config, cem_config,
            start_id=next_id, on_record=record)
        ranked = feasible_rank(front_rows + cem_rows)

    selected = None
    final_records = []
    final_started = time.perf_counter()
    for row in ranked:
        forward = objective.forward(row["x"])
        audit = audit_multi(scenario, forward, dynamic_config)
        final_records.append(dict(id=row["id"], flight_time=row["flight_time"], audit=audit))
        write_json(output / f"candidate_{row['id']}_final_audit.json", audit)
        if audit["trajectory_validation_pass"]:
            selected = row
            np.savez_compressed(output / "selected_trajectory.npz", x=row["x"],
                                coefficients=forward.trajectory.coefficients,
                                durations=forward.durations, crossing_times=forward.crossing_times)
            break
    reasons = Counter(row["screen"]["reason"] for row in rows)
    summary = dict(status="SAMPLED_FEASIBLE" if selected else "NO_FEASIBLE_CANDIDATE_FOUND",
                   baseline_flight_time=baseline["flight_time"], baseline_max_velocity=baseline["dynamics_diagnostic"]["max_velocity"],
                   front_end_evaluations=len(front_rows), cem_evaluations=len(cem_rows), total_evaluations=len(rows),
                   front_end_geometry_complete=len(candidates), hard_screen_feasible=len(ranked),
                   selected=selected, rejection_counts=reasons, cem_rounds=rounds,
                   final_audits=final_records, final_audit_seconds=time.perf_counter() - final_started,
                   total_seconds=time.perf_counter() - started)
    write_json(output / "result.json", summary)
    report = ["# Feasibility-Guided CEM：三窗口结果", "",
              f"- 状态：`{summary['status']}`。", f"- 相位前端 {len(front_rows)} 次，CEM {len(cem_rows)} 次，总计 {len(rows)} 次。",
              f"- 中间硬筛选通过 {len(ranked)} 条；最终整机审计 {len(final_records)} 条。",
              f"- 原 SC：T={baseline['flight_time']:.9f} s，vmax={baseline['dynamics_diagnostic']['max_velocity']:.9f} m/s。",
              f"- 最终选择：{('id='+str(selected['id'])+', T='+format(selected['flight_time'],'.9f')+' s, vmax='+format(selected['screen']['dynamics']['max_velocity'],'.9f')+' m/s') if selected else '无；未返回失败候选。'}", "",
              "失败样本只用于更新提议分布；最终集合和时间排序只包含全部中间硬约束通过者。整机检查仅用于最终候选。",
              "全部结论均为名义模型密集采样验证，不是连续时间认证。", ""]
    (output / "REPORT.md").write_text("\n".join(report), encoding="utf-8")
    print(json.dumps(jsonable({k: summary[k] for k in ("status", "total_evaluations", "hard_screen_feasible", "selected", "total_seconds")}), ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
