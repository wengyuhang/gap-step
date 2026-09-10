#!/usr/bin/env python3
"""Run dual-constraint CEM with SC maps fitted to physical, uninset apertures."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np

from nonconvex_timevarying_window.comparisons.seven_mixed_sc_fixed_cem.experiment import solve
from nonconvex_timevarying_window.comparisons.seven_unique_dual_constraint_cem.experiment import (
    dynamic_audit,
    plot_closed_route,
    safety_audit,
    scene_record,
    soft_metrics,
    source_manifest as inset_source_manifest,
    togt_config,
)
from nonconvex_timevarying_window.comparisons.seven_unique_sc_sphere.experiment import (
    build_seven_unique_track,
)
from nonconvex_timevarying_window.dual_constraint_cem_sc_dynatogt.safety_penalty import (
    SafetyPenaltyConfig,
)
from nonconvex_timevarying_window.dual_constraint_cem_sc_dynatogt.safety_augmented_objective import (
    SafetyAugmentedSCObjective,
)
from nonconvex_timevarying_window.dual_constraint_cem_sc_dynatogt.search import (
    DualCEMConfig,
    dual_constraint_cem,
)
from nonconvex_timevarying_window.random_dk_sc_dynatogt.experiment import jsonable, write_json
from nonconvex_timevarying_window.random_dk_sc_dynatogt.multi_window import MultiWindowObjective
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
RAW_MAP_ROOT = HERE / "preprocessed_raw_sc_maps"


class UninsetSCWindow:
    """Keep the physical window and rho, but map D into its raw aperture."""

    def __init__(self, source, sc_map):
        self.name = source.name
        self.gate = SimpleNamespace(sc_map=sc_map)
        self.center = source.center
        self.plane_basis = source.plane_basis
        self.normal = source.normal
        self.theta0 = source.theta0
        self.omega = source.omega
        self.thickness = source.thickness
        self.rho = source.rho
        self._source = source

    @property
    def safe_polygon(self):
        # This name is required by the common SC adapter.  In this ablation it
        # means the SC parameter domain and deliberately has zero geometric inset.
        return self.gate.sc_map.vertices

    @property
    def physical_polygon(self):
        return self._source.physical_polygon

    def rotated_basis(self, absolute_time):
        return self._source.rotated_basis(absolute_time)

    def world_point(self, local_point, absolute_time, z=0.0):
        return self._source.world_point(local_point, absolute_time, z)

    def boundary_at(self, absolute_time, *, safe=False, z=0.0):
        polygon = self.safe_polygon if safe else self.physical_polygon
        return (self.center[None, :] +
                (self.rotated_basis(absolute_time) @ np.asarray(polygon).T).T +
                self.normal[None, :] * float(z))


def remove_exact_collinear(vertices, relative_tolerance=1e-12):
    """Remove redundant samples on straight runs without changing the polygon."""
    points = np.asarray(vertices, dtype=float)
    incoming = points - np.roll(points, 1, axis=0)
    outgoing = np.roll(points, -1, axis=0) - points
    cross = incoming[:, 0] * outgoing[:, 1] - incoming[:, 1] * outgoing[:, 0]
    scale = np.linalg.norm(incoming, axis=1) * np.linalg.norm(outgoing, axis=1)
    same_direction = np.einsum("ij,ij->i", incoming, outgoing) > 0.0
    redundant = same_direction & (np.abs(cross) <= relative_tolerance * scale)
    result = points[~redundant]
    if len(result) < 3:
        raise ValueError("collinear cleanup removed the polygon")
    return result


def _raw_map(source, quadrature_order=64):
    started = time.perf_counter()
    sampled = np.asarray(source.gate.sampled_boundary.vertices, dtype=float)
    vertices = remove_exact_collinear(sampled)
    source_digest = hashlib.sha256(sampled.tobytes()).hexdigest()
    digest = hashlib.sha256(vertices.tobytes()).hexdigest()
    directory = RAW_MAP_ROOT / source.name
    map_path = directory / "sc_map.npz"
    manifest_path = directory / "manifest.json"
    if map_path.is_file() and manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        mapping = SCDiskMap.load(map_path)
        if (manifest.get("mapping_vertices_sha256") != digest or
                not np.array_equal(mapping.vertices, vertices)):
            raise ValueError(f"raw SC cache mismatch for {source.name}")
        return mapping, {**manifest, "cache_hit": True,
                         "elapsed_seconds": time.perf_counter() - started}
    mapping = SCDiskMap.fit(vertices, quadrature_order=quadrature_order, max_nfev=1200)
    directory.mkdir(parents=True, exist_ok=True)
    mapping.save(map_path)
    manifest = {
        "window": source.name,
        "mapping_polygon": "Chang-resampled physical aperture with exact straight-run cleanup; no inward offset",
        "source_sampled_vertices": len(sampled),
        "mapping_vertices": len(vertices),
        "source_sampled_vertices_sha256": source_digest,
        "mapping_vertices_sha256": digest,
        "quadrature_order": quadrature_order,
        "optimizer_success": bool(mapping.diagnostics.optimizer_success),
        "parameter_residual_inf": float(mapping.diagnostics.parameter_residual_inf),
        "vertex_reconstruction_inf": float(mapping.diagnostics.vertex_reconstruction_inf),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return mapping, {**manifest, "cache_hit": False,
                     "elapsed_seconds": time.perf_counter() - started}


def build_uninset_track():
    inset_scenario, inherited = build_seven_unique_track()
    windows, preprocessing = [], []
    for source in inset_scenario.windows:
        mapping, record = _raw_map(source)
        windows.append(UninsetSCWindow(source, mapping))
        preprocessing.append(record)
    scenario = type(inset_scenario)(
        inset_scenario.name + "_uninset_sc",
        inset_scenario.start_state,
        inset_scenario.goal_state,
        tuple(windows),
        "SC maps use physical apertures without inward offsets; safety remains a separate soft constraint.",
        inset_scenario.body,
        inset_scenario.difficulty + "-uninset-sc",
        inset_scenario.design_basis + ("no geometric inset before SC fitting",),
    )
    return scenario, togt_config(inherited), preprocessing


def source_manifest():
    # The shared comparison tree also contains an independent Gazebo adapter.
    # It is not imported by this numerical experiment and must not invalidate
    # a run when that adapter is edited concurrently.
    manifest = {key: value for key, value in inset_source_manifest().items()
                if "/gazebo/" not in key and "/tests/" not in key}
    for path in sorted(HERE.rglob("*.py")):
        manifest[str(path.relative_to(REPO))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--population", type=int, default=64)
    parser.add_argument("--maximum-rounds", type=int, default=300,
                        help="high fallback cap; normal stop is first strict double zero plus five rounds")
    parser.add_argument("--hard-audit-limit", type=int, default=96)
    args = parser.parse_args(argv)

    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    start_manifest = source_manifest()
    write_json(output / "code_manifest_start.json", start_manifest)
    started = time.perf_counter()
    preprocessing_started = time.perf_counter()
    print("Building/loading SC maps on raw physical apertures", flush=True)
    scenario, config, preprocessing = build_uninset_track()
    preprocessing_seconds = time.perf_counter() - preprocessing_started
    safety_config = SafetyPenaltyConfig()
    cem_config = DualCEMConfig(
        population=args.population,
        elite=max(4, min(24, args.population // 5)),
        memory=max(2, min(12, args.population // 10)),
        maximum_rounds=args.maximum_rounds,
    )
    write_json(output / "scene.json", scene_record(scenario))
    write_json(output / "protocol.json", {
        "sc_mapping_polygon": "Chang-resampled physical aperture with exact straight-run cleanup",
        "geometric_inset_before_sc": False,
        "planning_radius": float(scenario.windows[0].rho),
        "body_radius": float(scenario.body.circumscribed_radius),
        "safety_margin": float(scenario.windows[0].rho - scenario.body.circumscribed_radius),
        "safety_soft_integral_uses_planning_radius": True,
        "lbfgs_objective": "original TOGT objective + 100000 * differentiable whole-trajectory safety integral",
        "lbfgs_safety_quadrature": "all MINCO segments; duration-adaptive 8-32 trapezoid intervals",
        "hard_collision_uses_body_radius": True,
        "preprocessing": preprocessing,
        "cem": asdict(cem_config),
        "safety_penalty": asdict(safety_config),
        "code_frozen_during_run": True,
        "initialization": "native cold start: d=0 and geometry-derived TOGT durations",
        "uses_previous_experiment_result": False,
        "normal_stop": "first strict double zero followed by five complete CEM rounds",
        "fallback_stop": f"{args.maximum_rounds} CEM rounds without a strict double zero",
    })

    lbfgs_stage_started = time.perf_counter()
    base_objective = MultiWindowObjective(scenario, config)
    objective = SafetyAugmentedSCObjective(base_objective, scenario.windows, safety_config)
    initial = objective.initial_guess()
    initial_forward = objective.forward(initial)
    write_json(output / "cold_start.json", {
        "flight_time": float(initial_forward.trajectory.total_time),
        "local_points": initial_forward.local_points,
        "soft_metrics": soft_metrics(initial_forward, scenario, config, safety_config),
    })
    print("Optimizing safety-augmented SC-DynaTOGT from native cold start", flush=True)
    baseline_result, baseline_forward, optimizer_seconds, baseline_calls = solve(
        "Dual-Constraint-SC-DynaTOGT-uninset", objective, initial)
    lbfgs_seconds = time.perf_counter() - lbfgs_stage_started
    optimized_evaluation = objective.evaluate(baseline_result.x)
    baseline = {
        "objective_definition": "togt_plus_safety_soft_integral_v1",
        "flight_time": float(baseline_forward.trajectory.total_time),
        **soft_metrics(baseline_forward, scenario, config, safety_config),
        "lbfgs_base_objective": optimized_evaluation.base_cost,
        "lbfgs_safety_integral": optimized_evaluation.safety_integral,
        "lbfgs_safety_weight": safety_config.objective_weight,
        "dynamic_audit": dynamic_audit(baseline_forward.trajectory, config),
        "safety_audit": safety_audit(baseline_forward, scenario),
        "optimizer_success": bool(baseline_result.success),
        "optimizer_message": str(baseline_result.message),
        "iterations": int(baseline_result.nit),
        "objective_evaluations": baseline_calls,
        "optimizer_seconds": optimizer_seconds,
        "stage_seconds": lbfgs_seconds,
        "decision_vector": np.asarray(baseline_result.x),
    }
    baseline_x = np.asarray(baseline_result.x)
    write_json(output / "uninset_sc_baseline.json", baseline)
    np.savez_compressed(output / "uninset_sc_baseline_trajectory.npz",
                        x=baseline_x,
                        coefficients=baseline_forward.trajectory.coefficients,
                        durations=baseline_forward.durations,
                        crossing_times=baseline_forward.crossing_times,
                        local_points=baseline_forward.local_points)
    plot_closed_route(scenario, baseline_forward.trajectory,
                      output / "figures" / "uninset_sc_baseline_route.png")

    print("Running dual-constraint CEM without geometric inset", flush=True)
    cem_started = time.perf_counter()
    with (output / "candidates.jsonl").open("x", encoding="utf-8") as log:
        def record(row):
            log.write(json.dumps(jsonable(row), ensure_ascii=False, allow_nan=False) + "\n")
            log.flush()
            if row["id"] % max(1, args.population // 4) == 0:
                print(f"candidate={row['id']} round={row['round']} "
                      f"Jdyn={row['dynamic_integral']:.5g} Jsafe={row['safety_integral']:.5g} "
                      f"T={row['flight_time']:.5g}", flush=True)

        candidates, ranked, summaries = dual_constraint_cem(
            objective, baseline_x, scenario, config, cem_config, safety_config,
            on_record=record)
    cem_seconds = time.perf_counter() - cem_started
    random_candidate_count = len(candidates) - 1
    planning_seconds = preprocessing_seconds + lbfgs_seconds + cem_seconds

    audit_order = sorted((row for row in ranked if row["both_soft_zero"]),
                         key=lambda row: (row["flight_time"], row["id"]))
    hard_rows, selected = [], None
    print("Hard-auditing strict double-zero candidates", flush=True)
    audit_started = time.perf_counter()
    with (output / "hard_audits.jsonl").open("x", encoding="utf-8") as log:
        for audit_index, candidate in enumerate(audit_order[:args.hard_audit_limit], start=1):
            forward = objective.forward(candidate["x"])
            dynamics = dynamic_audit(forward.trajectory, config)
            safety = safety_audit(forward, scenario)
            row = {"audit_index": audit_index, "id": candidate["id"],
                   "flight_time": candidate["flight_time"], "dynamics": dynamics,
                   "safety": safety,
                   "passed": bool(dynamics["passed"] and safety["passed"])}
            hard_rows.append(row)
            log.write(json.dumps(jsonable(row), ensure_ascii=False, allow_nan=False) + "\n")
            log.flush()
            margin = min(item["minimum_margin"] for item in safety["per_window"])
            print(f"hard-audit {audit_index}: candidate={candidate['id']} "
                  f"T={candidate['flight_time']:.9f} dynamics={dynamics['passed']} "
                  f"safety={safety['passed']} min_margin={margin:.9g}", flush=True)
            if row["passed"]:
                selected = candidate
                np.savez_compressed(output / "selected_trajectory.npz", x=candidate["x"],
                                    coefficients=forward.trajectory.coefficients,
                                    durations=forward.durations,
                                    crossing_times=forward.crossing_times,
                                    local_points=forward.local_points)
                plot_closed_route(scenario, forward.trajectory,
                                  output / "figures" / "selected_route.png")
                break
    hard_audit_seconds = time.perf_counter() - audit_started

    end_manifest = source_manifest()
    unchanged = start_manifest == end_manifest
    write_json(output / "code_manifest_end.json", {"unchanged": unchanged, "files": end_manifest})
    if not unchanged:
        raise RuntimeError("source code changed during formal experiment")
    status = "SAMPLED_HARD_FEASIBLE" if selected is not None else "NO_HARD_FEASIBLE_CANDIDATE_FOUND"
    result = {
        "status": status,
        "geometric_inset_before_sc": False,
        "baseline": baseline,
        "candidate_count": random_candidate_count,
        "strict_double_zero_count": len(audit_order),
        "hard_audited": len(hard_rows),
        "selected": selected,
        "hard_results": hard_rows,
        "rounds": summaries,
        "timing_seconds": {
            "preprocessing": preprocessing_seconds,
            "lbfgs": lbfgs_seconds,
            "cem": cem_seconds,
            "planning_total": planning_seconds,
            "hard_audit": hard_audit_seconds,
            "wall_total": time.perf_counter() - started,
        },
        "total_seconds": time.perf_counter() - started,
        "code_unchanged_during_run": unchanged,
    }
    write_json(output / "result.json", result)
    lines = ["# 七异形闭环：不内缩 SC 的双约束 CEM", "",
             "SC 映射直接使用 Chang 重采样后的物理开口；没有在建图前做几何内缩。安全软积分仍使用外接球加 15 mm，硬检测使用真实外接球。", "",
             f"- 状态：`{status}`",
             f"- 安全增强无内缩 SC 初始解：T=`{baseline['flight_time']:.9f} s`，Jdyn=`{baseline['dynamic_integral']:.9g}`，Jsafe=`{baseline['safety_integral']:.9g}`",
             f"- 随机候选：`{random_candidate_count}`；严格双零：`{len(audit_order)}`；硬验收：`{len(hard_rows)}`",
             f"- 求解时间：预处理 `{preprocessing_seconds:.3f} s`；L-BFGS `{lbfgs_seconds:.3f} s`；CEM `{cem_seconds:.3f} s`；规划合计 `{planning_seconds:.3f} s`；硬验收 `{hard_audit_seconds:.3f} s`"]
    if selected is not None:
        hard = hard_rows[-1]
        margin = min(item["minimum_margin"] for item in hard["safety"]["per_window"])
        lines += [f"- 选中候选：id=`{selected['id']}`，T=`{selected['flight_time']:.9f} s`，Jdyn=`0`，Jsafe=`0`",
                  f"- 完整动力学检测：满足；真实外接球检测：满足；最小球体余量：`{margin:.9f} m`"]
    lines += ["", "硬检测是密集采样数值证据，不是连续域证书。", ""]
    (output / "REPORT.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(jsonable({"status": status, "baseline": baseline,
                               "candidate_count": random_candidate_count,
                               "strict_double_zero_count": len(audit_order),
                               "hard_audited": len(hard_rows), "selected": selected}),
                     ensure_ascii=False), flush=True)
    return 0 if selected is not None else 2


if __name__ == "__main__":
    raise SystemExit(main())
