#!/usr/bin/env python3
"""Audit schedule-neutral local time warping on the shared rotating-U case."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.optimize import brentq

from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt import (
    compare_fixed_wp_counterexample as common,
)
from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.compare_sc_dynatogt_fixed_wp import (
    FixedWaypointObjective,
    FreeSCWaypointObjective,
)
from nonconvex_timevarying_window.phase_governed_sc_tracking.experiment import (
    _build_counterexample,
)

from .timewarp import LocalTimeWarpTrajectory, TimeWarpPatch


HERE = Path(__file__).resolve().parent
FIXED_DECISION = np.asarray((0.26069174120255073, 0.18271885797050175))
SC_DECISION = np.asarray(
    (0.27518933313546307, 0.09544511501033515, 11.581502152706596, 1.4453188984601368)
)
PATCH_HALF_DURATION = 0.5
PEAK_SHIFT = -0.017
AUDIT_STEP = 0.0002


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


def _row(method: str, forward, audit) -> dict:
    extrema = audit["constraint_extrema"]
    return {
        "method": method,
        "flight_time": float(forward.trajectory.total_time),
        "crossing_time": float(forward.crossing_times[0]),
        "local_point": np.asarray(forward.local_points[0]),
        "collision_free": bool(audit["collision_free"]),
        "colliding_samples": int(audit["colliding_samples"]),
        "minimum_frame_clearance": audit["minimum_frame_clearance"],
        "ordered_exactly_once": bool(audit["ordered_exactly_once"]),
        "sampled_dynamic_limits_satisfied": bool(
            audit["sampled_dynamic_limits_satisfied"]
        ),
        "max_velocity": float(extrema["max_velocity"]),
        "max_rotor_thrust": float(np.max(extrema["max_rotor_thrust"])),
        "maximum_c3_interface_jump": float(audit["maximum_c3_interface_jump"]),
        "audit_samples": int(audit["audit_samples"]),
        "audit_dt_max": float(audit["audit_dt_max"]),
    }


def _audit(scenario, forward, config):
    return common._EXPERIMENT.audit_solution(
        scenario, forward, config, dt=AUDIT_STEP
    )


def _write_report(path: Path, rows: list[dict], patch: TimeWarpPatch, recovery: dict):
    by_name = {row["method"]: row for row in rows}
    nominal = by_name["SC-DynaTOGT"]
    warped = by_name["Local time-warp"]
    lines = [
        "# U 形旋转窗口：不等待的局部时间回接",
        "",
        "这是对“起点等待”反例的修正实验。执行轨迹在有限区间内暂时减慢并追回名义进度，不增加总飞行时间；在恢复时刻后与原 SC 轨迹的绝对时间轴完全一致。",
        "",
        "|method|T [s]|crossing [s]|collision samples|collision-free|ordered once|sampled dynamics|",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"|{row['method']}|{row['flight_time']:.9f}|{row['crossing_time']:.9f}|"
            f"{row['colliding_samples']}/{row['audit_samples']}|{row['collision_free']}|"
            f"{row['ordered_exactly_once']}|{row['sampled_dynamic_limits_satisfied']}|"
        )
    lines.extend(
        [
            "",
            f"补丁区间为 `[{patch.start_time:.9f}, {patch.recovery_time:.9f}] s`，峰值时间偏移为 `{patch.peak_shift * 1000:.3f} ms`。",
            f"SC 穿越时刻局部推迟 `{(warped['crossing_time'] - nominal['crossing_time']) * 1000:.6f} ms`，但总时间仍为 `{warped['flight_time']:.9f} s`。",
            f"在恢复时刻以及之后的最大 PVAJ 误差为 `{recovery['maximum_downstream_pvaj_error']:.3e}`；因此只要下一窗名义穿越时刻晚于 `{patch.recovery_time:.9f} s`，它的时刻不受该补丁影响。",
            f"在最大步长 `{warped['audit_dt_max'] * 1000:.6f} ms` 的临界时刻加密审计中，碰撞样本由 SC-DynaTOGT 的 `{nominal['colliding_samples']}` 降为 `0`，最小门框净距为 `{warped['minimum_frame_clearance'] * 1000:.6f} mm`。",
            "",
            "这个结果只验证了“碰撞消除 + 下游时间回接”，且仍是密集采样数值验收，不是连续域证书。原 SC 参考轨迹已超出本实验的速度/旋翼推力上限，时间变形也未修复它，所以不能将本结果写成完整飞控可行。",
            "多窗中必须在进入补丁前联合检查当前窗碰撞、动力学与下一窗的时间余量。若恢复时刻不能早于下一窗，或找不到同时满足碰撞和动力学的标量偏移，则该局部方法明确失败并提前触发重规划。",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--outdir",
        type=Path,
        default=HERE / "results" / "u_w18_p1p1_local_rejoin",
    )
    args = parser.parse_args(argv)
    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)

    _, config, _, scenario, track, _, fixed_d = _build_counterexample()
    fixed = FixedWaypointObjective(track, config, fixed_d).forward(
        FIXED_DECISION
    )
    nominal = FreeSCWaypointObjective(track, config).forward(SC_DECISION)
    nominal_crossing = float(nominal.crossing_times[0])
    patch = TimeWarpPatch(
        nominal_crossing - PATCH_HALF_DURATION,
        nominal_crossing + PATCH_HALF_DURATION,
        PEAK_SHIFT,
    )
    warped_trajectory = LocalTimeWarpTrajectory(nominal.trajectory, (patch,))
    crossing = brentq(
        lambda time: float(warped_trajectory.warp_time(time)) - nominal_crossing,
        patch.start_time,
        patch.recovery_time,
        xtol=1.0e-13,
    )
    position = np.asarray(warped_trajectory.evaluate(crossing))
    local_point = (
        scenario.windows[0].rotated_basis(crossing).T
        @ (position - scenario.windows[0].center)
    )
    warped = SimpleNamespace(
        trajectory=warped_trajectory,
        crossing_times=np.asarray((crossing,)),
        local_points=local_point[None, :],
        crossing_local_index=0,
        durations=warped_trajectory.durations,
        method="Local time-warp",
    )

    records = []
    rows = []
    for name, forward in (
        ("Fixed-WP", fixed),
        ("SC-DynaTOGT", nominal),
        ("Local time-warp", warped),
    ):
        audit, data = _audit(scenario, forward, config)
        records.append((name, audit, data))
        rows.append(_row(name, forward, audit))

    by_name = {name: audit for name, audit, _ in records}
    if by_name["Fixed-WP"]["collision_free"]:
        raise RuntimeError("Fixed-WP is no longer a collision counterexample")
    if by_name["SC-DynaTOGT"]["collision_free"]:
        raise RuntimeError("SC-DynaTOGT is no longer a collision counterexample")
    if not by_name["Local time-warp"]["collision_free"]:
        raise RuntimeError("local time warp failed the dense collision audit")
    if not by_name["Local time-warp"]["ordered_exactly_once"]:
        raise RuntimeError("local time warp changed the required crossing count")

    probes = np.asarray(
        (patch.recovery_time, 0.5 * (patch.recovery_time + nominal.trajectory.total_time), nominal.trajectory.total_time)
    )
    downstream_error = max(
        float(
            np.max(
                np.abs(
                    warped_trajectory.evaluate(probes, derivative)
                    - nominal.trajectory.evaluate(probes, derivative)
                )
            )
        )
        for derivative in range(4)
    )
    recovery = {
        "recovery_time": patch.recovery_time,
        "downstream_probe_times": probes,
        "maximum_downstream_pvaj_error": downstream_error,
        "total_time_change": warped_trajectory.total_time - nominal.trajectory.total_time,
        "minimum_progress_rate": float(
            np.min(
                warped_trajectory.time_derivative(
                    np.linspace(patch.start_time, patch.recovery_time, 10001)
                )
            )
        ),
    }
    payload = {
        "scenario": {
            "name": scenario.name,
            "window_count": 1,
            "shape": "balanced_U",
            "size_ratio": common.RATIO,
            "omega": scenario.windows[0].omega,
            "phase": scenario.windows[0].theta0,
            "body_half_extents": scenario.body.half_extents,
        },
        "protocol": {
            "audit": "<=0.2 ms plus critical-time refinement; sampled numerical validation",
            "purpose": "collision removal and downstream schedule recovery",
            "multi_window_scope": "later gates are unchanged only when their nominal crossing times are after recovery_time",
        },
        "patch": patch,
        "recovery": recovery,
        "rows": rows,
        "full_audits": {name: audit for name, audit, _ in records},
        "fixed_decision_vector": FIXED_DECISION,
        "sc_decision_vector": SC_DECISION,
    }
    (output / "result.json").write_text(
        json.dumps(_jsonable(payload), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    for name, _, data in records:
        common._EXPERIMENT.save_raw_trajectory(
            output / name.lower().replace("-", "_").replace(" ", "_"), data
        )
    _write_report(output / "REPORT.md", rows, patch, recovery)
    for row in rows:
        print(json.dumps(_jsonable(row), ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
