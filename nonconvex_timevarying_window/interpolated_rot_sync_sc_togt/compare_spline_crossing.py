#!/usr/bin/env python3
"""Compare the linear and degree-7 spline SC crossing parameterizations."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from . import compare_fixed_wp_counterexample as common
from . import compare_fixed_wp_seeded as problem
from .optimizer import SplineRotSyncObjective


HERE = Path(__file__).resolve().parent
LINEAR_METHOD = "Linear-SC-Crossing"
SPLINE_METHOD = "Spline-SC-Crossing"
FIXED_REFERENCE = (
    HERE
    / "results"
    / "zero_thickness_fixed_wp_seeded_togt_code_unlimited_20260908"
    / "Fixed-WP"
    / "result.json"
)


class CodeTOGTSplineObjective(
    common._CodeTOGTObjectiveMixin, SplineRotSyncObjective
):
    """Spline crossing with TOGT dynamics plus an explicit control cost."""

    def __init__(self, scenario, config, *, snap_weight: float) -> None:
        super().__init__(scenario, config)
        if not np.isfinite(snap_weight) or snap_weight <= 0.0:
            raise ValueError("snap_weight must be finite and positive")
        self.snap_weight = float(snap_weight)

    def cost_breakdown(self, forward):
        trajectory = forward.trajectory
        dynamic = self.code_dynamic_breakdown(forward)
        total = float(trajectory.total_time)
        smoothness = float(trajectory.snap_energy())
        dynamic_total = float(np.real(dynamic.total))
        return common._EXPERIMENT.CostBreakdown(
            total,
            smoothness,
            dynamic_total,
            0.0,
            total + dynamic_total + self.snap_weight * smoothness,
        )

    def forward(self, x):
        result = super().forward(x)
        return SimpleNamespace(
            **result.__dict__, crossing_local_index=0, method=SPLINE_METHOD
        )


def _linear_solution_to_spline_initial(linear, spline, decision_vector):
    free_k, sync_k, entry, exit = linear.split(decision_vector)
    return spline.pack_linear(free_k, sync_k, entry, exit)


def _write_report(path: Path, rows, diagnostics, fixed_reference) -> None:
    linear, spline = rows
    fixed_time = float(fixed_reference["flight_time"])
    lines = [
        "# 七阶 SC 穿窗曲线试验",
        "",
        "本试验只把当前两点线性 SC 穿窗段扩展为单段七阶 Bézier；前后段仍为 degree-7 MINCO。",
        "面内 8 个控制点逐时刻经过 SC 映射，法向 8 个有序控制值由正增量生成，因此局部点天然位于安全开口内且法向天然单调。",
        "先独立优化线性方法，再把它逐点等价地嵌入样条参数化；Fixed-WP 结果只作保留基线，没有用于构造或初始化样条。",
        f"样条目标在 TOGT 动力学代价之外加入 {diagnostics['snap_weight']:.3g} * integral_snap_squared，使自由控制点具有与 MINCO 一致的控制代价。",
        "",
        f"- 线性解嵌入后的 0–4 阶最大误差：{diagnostics['maximum_subset_derivative_error']:.3e}",
        f"- 不计新增 snap 项时的嵌入目标误差：{diagnostics['subset_code_objective_error']:.3e}",
        f"- 保留 Fixed-WP 基线时间：{fixed_time:.12g} s",
        "",
        "|method|solver stop|trajectory pass|T|J|dynamic penalty|TOGT-code violations|C3 jump|vs Fixed-WP time|",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        relative = 100.0 * (float(row["flight_time"]) / fixed_time - 1.0)
        lines.append(
            f"|{row['method']}|{row['optimizer_success']}|{row['trajectory_pass']}|"
            f"{row['flight_time']:.12g}|{row['objective']:.12g}|"
            f"{row['dynamic_penalty']:.12g}|{row['togt_code_dynamic_violation_samples']}|"
            f"{row['maximum_c3_interface_jump']:.3e}|{relative:+.3f}%|"
        )
    lines.extend(
        (
            "",
            "动力学与碰撞结论来自最大 1 ms 网格的独立数值审计，不是连续域证书。",
            "",
        )
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--outdir",
        type=Path,
        default=HERE / "results" / "zero_thickness_spline_crossing",
    )
    parser.add_argument("--snap-weight", type=float, default=1.0e-6)
    args = parser.parse_args(argv)
    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)

    _, config, geometry, scenario = problem._zero_thickness_problem()
    linear = common.CodeTOGTInterpolatedObjective(scenario, config)
    linear_row, linear_payload = common._run_method(
        common._EXPERIMENT,
        objective=linear,
        scenario=scenario,
        geometry=geometry,
        method=LINEAR_METHOD,
        config=config,
        output=output / LINEAR_METHOD,
    )
    print(json.dumps(common._EXPERIMENT._jsonable(linear_row), ensure_ascii=False), flush=True)

    spline = CodeTOGTSplineObjective(scenario, config, snap_weight=args.snap_weight)
    spline_initial = _linear_solution_to_spline_initial(
        linear,
        spline,
        np.asarray(linear_payload["decision_vector"], dtype=float),
    )
    linear_forward = linear.forward(linear_payload["decision_vector"])
    spline_initial_forward = spline.forward(spline_initial)
    times = np.linspace(0.0, linear_forward.trajectory.total_time, 257)
    derivative_errors = {
        str(order): float(
            np.max(
                np.abs(
                    linear_forward.trajectory.evaluate(times, order)
                    - spline_initial_forward.trajectory.evaluate(times, order)
                )
            )
        )
        for order in range(5)
    }
    spline_code_cost = common._CodeTOGTObjectiveMixin.cost_breakdown(
        spline, spline_initial_forward
    )
    subset_code_objective_error = abs(
        linear.cost_breakdown(linear_forward).weighted_total
        - spline_code_cost.weighted_total
    )
    diagnostics = {
        "subset_derivative_errors": derivative_errors,
        "maximum_subset_derivative_error": max(derivative_errors.values()),
        "subset_code_objective_error": subset_code_objective_error,
        "snap_weight": args.snap_weight,
        "initial_regularized_objective": spline.cost_breakdown(
            spline_initial_forward
        ).weighted_total,
    }
    if diagnostics["maximum_subset_derivative_error"] > 1.0e-8:
        raise RuntimeError("linear trajectory did not embed in spline parameterization")
    if subset_code_objective_error > 1.0e-8:
        raise RuntimeError("linear objective did not embed in spline parameterization")

    spline_row, spline_payload = common._run_method(
        common._EXPERIMENT,
        objective=spline,
        scenario=scenario,
        geometry=geometry,
        method=SPLINE_METHOD,
        config=config,
        output=output / SPLINE_METHOD,
        initial_x=spline_initial,
    )
    print(json.dumps(common._EXPERIMENT._jsonable(spline_row), ensure_ascii=False), flush=True)

    fixed_reference = json.loads(FIXED_REFERENCE.read_text(encoding="utf-8"))["row"]
    rows = [linear_row, spline_row]
    common._write_comparison_csv(output / "comparison.csv", rows)
    common._EXPERIMENT.write_json(
        output / "comparison.json",
        {
            "scenario": scenario.name,
            "parameterization": {
                "latent_bezier_degree": 7,
                "latent_control_points": 8,
                "normal_bezier_degree": 7,
                "normal_control_points": 8,
                "normal_free_log_increment_ratios": 6,
                "latent_inner_offset_scale": spline.latent_offset_scale,
                "snap_weight": args.snap_weight,
            },
            "initial_embedding": diagnostics,
            "fixed_wp_retained_reference": fixed_reference,
            "fixed_wp_used_for_initialization": False,
            "rows": rows,
            "spline_result": spline_payload,
        },
    )
    _write_report(output / "REPORT.md", rows, diagnostics, fixed_reference)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
