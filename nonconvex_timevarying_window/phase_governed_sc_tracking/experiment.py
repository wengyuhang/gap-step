#!/usr/bin/env python3
"""Reproduce a shared U-window counterexample and run the phase governor."""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
from types import SimpleNamespace

import matplotlib.pyplot as plt
import numpy as np

from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt import (
    compare_fixed_wp_counterexample as common,
)
from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.compare_sc_dynatogt_fixed_wp import (
    FixedWaypointObjective,
    FreeSCWaypointObjective,
    _RotatingSCWindowAdapter,
    _build_problem,
)
from nonconvex_timevarying_window.rot_sync_sc_togt.collision import (
    _slab_cross_section,
    body_rotations,
)
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.optimizer import _minimize_togt_lbfgs

from .governor import DelaySearchConfig, WaitThenTrackTrajectory, find_safe_delay


HERE = Path(__file__).resolve().parent
OMEGA = 18.0
PHASE = 1.1
THICKNESS = 0.0


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if hasattr(value, "__dataclass_fields__"):
        return _jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(_jsonable(value), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def _build_counterexample():
    weights, config, geometry, scenario, _, fixed_local, fixed_d = _build_problem()
    window = replace(
        scenario.windows[0], theta0=PHASE, omega=OMEGA, thickness=THICKNESS
    )
    scenario = replace(
        scenario,
        name="balanced_U_zero_thickness_w18_p1p1",
        windows=(window,),
        description="Shared Fixed-WP and SC-DynaTOGT whole-body collision counterexample",
    )
    track = SCWindowTrack(
        name=scenario.name,
        start=scenario.start_state.position,
        goal=scenario.goal_state.position,
        windows=(_RotatingSCWindowAdapter(window),),
        order=(0,),
    )
    return weights, config, geometry, scenario, track, fixed_local, fixed_d


def _solve(objective, initial):
    result = _minimize_togt_lbfgs(objective.value_and_gradient, initial, objective.config)
    return result, objective.forward(np.asarray(result.x, dtype=float))


def _audit(scenario, forward, config):
    # At 18 rad/s, 1 ms is about one degree of window rotation.  Use a denser
    # independent audit while retaining the cheaper 1 ms governor preview.
    return common._EXPERIMENT.audit_solution(scenario, forward, config, dt=0.0002)


def _row(name, optimizer, forward, audit):
    return {
        "method": name,
        "optimizer_success": bool(optimizer.success) if optimizer is not None else None,
        "optimizer_iterations": int(optimizer.nit) if optimizer is not None else None,
        "flight_time": float(forward.trajectory.total_time),
        "crossing_time": float(forward.crossing_times[0]),
        "local_point": np.asarray(forward.local_points[0], dtype=float),
        "collision_free": bool(audit["collision_free"]),
        "colliding_samples": int(audit["colliding_samples"]),
        "first_collision_time": audit["first_collision_time"],
        "minimum_frame_clearance": audit["minimum_frame_clearance"],
        "ordered_exactly_once": bool(audit["ordered_exactly_once"]),
        "sampled_dynamic_limits_satisfied": bool(
            audit["sampled_dynamic_limits_satisfied"]
        ),
        "maximum_c3_interface_jump": float(audit["maximum_c3_interface_jump"]),
        "audit_samples": int(audit["audit_samples"]),
        "audit_dt_max": float(audit["audit_dt_max"]),
    }


def _plot_section(axis, scenario, trajectory, absolute_time, title, color):
    window = scenario.windows[0]
    position = np.asarray(trajectory.evaluate(absolute_time), dtype=float)
    rotation = body_rotations(trajectory, np.asarray((absolute_time,)))[0]
    section = _slab_cross_section(
        window,
        absolute_time,
        position,
        rotation,
        scenario.body,
        tolerance=1.0e-9,
    )
    polygon = np.asarray(window.physical_polygon)
    closed = np.vstack((polygon, polygon[0]))
    axis.fill(closed[:, 0], closed[:, 1], color="#dce7f2", alpha=0.8)
    axis.plot(closed[:, 0], closed[:, 1], color="#263238", linewidth=2)
    if section is not None and not section.is_empty:
        if section.geom_type == "Polygon":
            values = np.asarray(section.exterior.coords)
            axis.fill(values[:, 0], values[:, 1], color=color, alpha=0.55)
            axis.plot(values[:, 0], values[:, 1], color=color, linewidth=2)
    axis.set_aspect("equal")
    axis.set_title(title)
    axis.set_xlabel("window-local u [m]")
    axis.set_ylabel("window-local v [m]")
    axis.grid(alpha=0.2)


def _write_report(path: Path, rows, search, period) -> None:
    by_method = {row["method"]: row for row in rows}
    fixed = by_method["Fixed-WP"]
    sc = by_method["SC-DynaTOGT"]
    governed = by_method["Phase-governed SC tracking"]
    lines = [
        "# U 形旋转窗口：离线碰撞与执行时相位调节",
        "",
        "本实验使用同一个零厚度、固定中心/平面、绕法向匀速旋转的均衡 U 形窗口。",
        f"窗口角速度为 `{OMEGA:g} rad/s`，初相位为 `{PHASE:g} rad`，旋转周期为 `{period:.9f} s`。",
        "Fixed-WP 和 SC-DynaTOGT 共享两段 degree-7 MINCO 与原始 TOGT 目标；它们的离线目标不包含整段整机碰撞约束。",
        "",
        "|method|solver stop|T [s]|crossing [s]|collision samples|collision-free|ordered once|sampled dynamics|",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"|{row['method']}|{row['optimizer_success']}|{row['flight_time']:.9f}|"
            f"{row['crossing_time']:.9f}|{row['colliding_samples']}/{row['audit_samples']}|"
            f"{row['collision_free']}|{row['ordered_exactly_once']}|"
            f"{row['sampled_dynamic_limits_satisfied']}|"
        )
    lines.extend(
        [
            "",
            f"执行时调节器从 `0` 到一个旋转周期以 `{search['config']['delay_step']:.4g} s` 候选间隔预览整机碰撞，选择最早可行等待时间。",
            f"选中等待为 `{search['selected']['delay']:.9f} s`，空间路径与 SC 离线轨迹完全相同，穿越时间改变 `{search['selected']['delay']:.9f} s`。",
            f"最终最大网格步长为 `{governed['audit_dt_max'] * 1000:.6f} ms`，碰撞样本从 Fixed-WP 的 `{fixed['colliding_samples']}` 和 SC-DynaTOGT 的 `{sc['colliding_samples']}` 降为 `0`。",
            f"最终临界时刻加密审计的最小净距为 `{governed['minimum_frame_clearance'] * 1000:.6f} mm`。",
            "",
            "该结果是密集采样数值验收，不是 SIP 连续域证书。相位调节只支持当前固定中心/平面、匀速自旋简化模型；若限定时间内没有安全相位，它会明确失败。",
            "等待段为悬停，后续不重定时，因而离线轨迹原有的动力学超限不会被此方法修复；碰撞与动力学结论需要分开。",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--outdir",
        type=Path,
        default=HERE / "results" / "u_w18_p1p1",
    )
    args = parser.parse_args(argv)
    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)

    weights, config, geometry, scenario, track, fixed_local, fixed_d = (
        _build_counterexample()
    )
    fixed_objective = FixedWaypointObjective(track, config, fixed_d)
    fixed_optimizer, fixed_forward = _solve(
        fixed_objective, fixed_objective.initial_guess()
    )
    free_objective = FreeSCWaypointObjective(track, config)
    free_optimizer, free_forward = _solve(
        free_objective, np.r_[fixed_optimizer.x, fixed_d]
    )
    fixed_audit, fixed_data = _audit(scenario, fixed_forward, config)
    free_audit, free_data = _audit(scenario, free_forward, config)

    if not fixed_optimizer.success or not free_optimizer.success:
        raise RuntimeError("counterexample optimization did not converge")
    if fixed_audit["colliding_samples"] == 0 or free_audit["colliding_samples"] == 0:
        raise RuntimeError("configured scenario is no longer a shared collision counterexample")
    if not fixed_audit["ordered_exactly_once"] or not free_audit["ordered_exactly_once"]:
        raise RuntimeError("offline references must each cross the window exactly once")

    period = 2.0 * math.pi / abs(scenario.windows[0].omega)
    governor_config = DelaySearchConfig(max_delay=period)
    search = find_safe_delay(
        scenario,
        free_forward.trajectory,
        float(free_forward.crossing_times[0]),
        config=governor_config,
    )
    if search.selected is None:
        raise RuntimeError("no safe phase was found within one rotation period")
    delayed = WaitThenTrackTrajectory(
        free_forward.trajectory, search.selected.delay
    )
    governed_forward = SimpleNamespace(
        trajectory=delayed,
        crossing_times=np.asarray(
            (float(free_forward.crossing_times[0] + search.selected.delay),)
        ),
        local_points=search.selected.crossing_local_point[None, :],
        crossing_local_index=0,
        durations=delayed.durations,
        method="Phase-governed SC tracking",
    )
    governed_audit, governed_data = _audit(scenario, governed_forward, config)
    if (
        not governed_audit["collision_free"]
        or not governed_audit["ordered_exactly_once"]
        or float(governed_audit["minimum_frame_clearance"]) < 15.0e-3
    ):
        raise RuntimeError("governed trajectory failed the independent collision audit")

    rows = [
        _row("Fixed-WP", fixed_optimizer, fixed_forward, fixed_audit),
        _row("SC-DynaTOGT", free_optimizer, free_forward, free_audit),
        _row("Phase-governed SC tracking", None, governed_forward, governed_audit),
    ]
    search_payload = {
        "config": governor_config,
        "evaluated_candidates": search.evaluated_candidates,
        "selected": search.selected,
    }
    _write_json(
        output / "result.json",
        {
            "scenario": {
                "name": scenario.name,
                "shape": "balanced_U",
                "size_ratio": common.RATIO,
                "omega": OMEGA,
                "phase": PHASE,
                "thickness": THICKNESS,
                "rotation_period": period,
                "body_half_extents": scenario.body.half_extents,
                "planning_radius": scenario.windows[0].rho,
                "fixed_local_point": fixed_local,
            },
            "protocol": {
                "offline_objective": "native SC-DynaTOGT TOGT objective; no collision term",
                "fixed_to_free_warm_start": True,
                "audit": "<=0.2 ms plus critical-time refinement; sampled numerical validation",
                "weights_reference": weights,
            },
            "rows": rows,
            "governor_search": search_payload,
            "fixed_decision_vector": fixed_optimizer.x,
            "sc_decision_vector": free_optimizer.x,
        },
    )
    for name, data in (
        ("fixed_wp", fixed_data),
        ("sc_dynatogt", free_data),
        ("phase_governed", governed_data),
    ):
        common._EXPERIMENT.save_raw_trajectory(output / name, data)

    figure, axes = plt.subplots(1, 2, figsize=(10.5, 4.5), constrained_layout=True)
    _plot_section(
        axes[0],
        scenario,
        free_forward.trajectory,
        float(free_audit["first_collision_time"]),
        "SC-DynaTOGT: first sampled collision",
        "#d32f2f",
    )
    _plot_section(
        axes[1],
        scenario,
        delayed,
        float(governed_forward.crossing_times[0]),
        "Phase-governed execution: crossing",
        "#2e7d32",
    )
    figure.savefig(output / "comparison.png", dpi=180)
    plt.close(figure)
    _write_report(output / "REPORT.md", rows, _jsonable(search_payload), period)
    for row in rows:
        print(json.dumps(_jsonable(row), ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
