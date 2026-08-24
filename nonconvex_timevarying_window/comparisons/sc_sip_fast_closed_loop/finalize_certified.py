"""Package the certified stress-test continuation and render final artifacts."""

from __future__ import annotations

import argparse
import json
import platform
from dataclasses import replace
from pathlib import Path

import numpy as np
import scipy
import matplotlib
import flint

from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from nonconvex_timevarying_window.sc_dynatogt.visualization import (
    export_dynamic_window_gif, export_trajectory_csv, plot_crossing_grid,
    plot_route_overview, plot_scale_profile,
)
from nonconvex_timevarying_window.sip_dynatogt.io import load_run, save_run
from nonconvex_timevarying_window.sip_dynatogt.model import (
    CertificateResult, CertificateStatus, ExchangeRecord, SIPProblem, SIPResult,
    Witness,
)

from .experiment import _sampled_clearance_profile, _sampled_dynamics
from .scenario import build_fast_closed_loop_scenario
from .visualization import plot_clearance_comparison, plot_contact_timeline, plot_time_comparison


def _certificate(data: dict) -> CertificateResult:
    return CertificateResult(
        CertificateStatus(data["status"]), data["reason"], data["precision_bits"],
        data["checked_cells"], data["maximum_depth"],
        data["minimum_safety_squared_margin"], data["minimum_dynamic_margin"],
        tuple(Witness(**item) for item in data.get("witnesses", [])),
    )


def _history(data: list[dict]) -> tuple[ExchangeRecord, ...]:
    return tuple(
        ExchangeRecord(
            item["iteration"], item["optimizer_success"], item["total_time"],
            item["active_witnesses"], CertificateStatus(item["certificate_status"]),
            item["certificate_cells"],
        )
        for item in data
    )


def finalize(output: str | Path, *, make_gif: bool = True) -> dict:
    repo = Path(__file__).resolve().parent
    results = repo / "results"
    v5 = results / "wide_scrambled_curves_v5"
    v7_run = results / "wide_scrambled_curves_v7" / "sip_dynatogt" / "run"
    v8_run = results / "wide_scrambled_curves_v8" / "sip_dynatogt" / "run"
    root = Path(output).expanduser()
    root.mkdir(parents=True, exist_ok=True)

    scenario = build_fast_closed_loop_scenario()
    problem = SIPProblem.from_track(scenario.track, boundaries=scenario.sip_boundaries)
    _, config, trajectory, stored = load_run(v8_run)
    deep = json.loads((v8_run / "deep_certificate.json").read_text(encoding="utf-8"))
    config = replace(config, precision_bits=(128,), max_depth=32, max_cells=4_000_000)
    report = _certificate(deep["certificate"])
    if report.status is not CertificateStatus.CERTIFIED_FEASIBLE:
        raise RuntimeError("deep certificate is not certified feasible")
    final_result = SIPResult(
        report.status,
        "continued stress-test candidate passed the complete interval certificate",
        np.asarray(stored["x"], dtype=float), trajectory,
        np.asarray(stored["durations"], dtype=float),
        np.asarray(stored["traversal_times"], dtype=float),
        np.asarray(stored["waypoints"], dtype=float), report,
        _history(stored["history"]), bool(stored["optimizer_success"]),
        int(stored["optimizer_iterations"]),
        tuple(Witness(**item) for item in stored["active_witnesses"]),
    )
    save_run(root / "sip_dynatogt" / "run", problem, config, final_result)

    v5_summary = json.loads((v5 / "summary.json").read_text(encoding="utf-8"))
    sc_data = v5_summary["sc_dynatogt"]
    sc_minco = MincoSnap(
        BoundaryState(scenario.track.start), BoundaryState(scenario.track.goal),
        np.asarray(sc_data["waypoints"], dtype=float),
        np.asarray(sc_data["durations"], dtype=float),
    )
    sip_minco = MincoSnap(
        BoundaryState(scenario.track.start), BoundaryState(scenario.track.goal),
        final_result.waypoints, final_result.durations,
    )
    sc_poly = final_result.trajectory.from_minco(sc_minco)
    sc_profile = _sampled_clearance_profile(problem, sc_poly, config, num_times=4001, boundary_samples=81)
    sip_profile = _sampled_clearance_profile(problem, trajectory, config, num_times=5001, boundary_samples=101)
    sip_dynamics = _sampled_dynamics(trajectory, config, num_times=10001)

    for name, value in (("sc_dynatogt", sc_minco), ("sip_dynatogt", sip_minco)):
        method = root / name
        label="SC-DynaTOGT" if name=="sc_dynatogt" else "SIP-DynaTOGT"
        plot_route_overview(scenario.track, value, method / "figures" / "route_overview.png",method_label=label)
        plot_crossing_grid(scenario.track, value, method / "figures" / "crossings_grid.png", columns=2,method_label=label)
        plot_scale_profile(scenario.track, value, method / "figures" / "scale_profile.png",method_label=label)
        export_trajectory_csv(value, method / "data" / "trajectory.csv", num_samples=2001)
        if make_gif:
            export_dynamic_window_gif(
                scenario.track, value, method / "media" / "dynamic_windows.gif",
                num_frames=96,
            )
    plot_clearance_comparison(sc_profile, sip_profile, config.clearance, root / "comparison" / "clearance_comparison.png")
    plot_contact_timeline(
        sc_profile, sip_profile, config.clearance,
        root / "comparison" / "contact_timeline.png",
        sc_confirmed_intersection_times=(8.801017673749083,),
    )
    plot_time_comparison(sc_minco, sip_minco, root / "comparison" / "flight_time_comparison.png")

    v7_resume = json.loads((v7_run / "resume_summary.json").read_text(encoding="utf-8"))
    v8_resume = json.loads((v8_run / "resume_summary.json").read_text(encoding="utf-8"))
    batched_separator_seconds = 265.07087326899637
    stages = {
        "sc_solve": sc_data["solve_wall_seconds"],
        "sip_initial_exchange_v5": v5_summary["sip_dynatogt"]["solve_wall_seconds"],
        "batched_collision_separator": batched_separator_seconds,
        "sip_collision_repair_v7": v7_resume["resume_wall_seconds"],
        "sip_margin_repair_v8": v8_resume["resume_wall_seconds"],
        "final_deep_certificate": deep["wall_seconds"],
    }
    scenario_summary = {
        "name": scenario.name,
        "start_goal": scenario.track.start.tolist(),
        "order_indices": list(scenario.track.order),
        "order_names": [scenario.track.windows[index].name for index in scenario.track.order],
        "center_span_xyz": np.ptp(np.asarray([w.center0 for w in scenario.track.windows]), axis=0).tolist(),
        "body_half_extents": list(scenario.body.half_extents),
        "net_clearance": scenario.net_clearance,
        "sc_center_inset": scenario.body.conservative_center_clearance(scenario.net_clearance),
        "exact_boundary_segments": sum(len(parts) for parts in scenario.sip_boundaries),
        "windows": [
            {
                "index": index, "name": window.name, "center0": window.center0.tolist(),
                "translation_amplitude": window.motion.translation_amplitude.tolist(),
                "rotation_amplitude_rpy": window.motion.rotation_amplitude.tolist(),
                "scale_range": [window.motion.minimum_scale, window.motion.maximum_scale],
                "translation_period": window.motion.translation_period,
                "rotation_period": window.motion.rotation_period,
                "scale_period": window.motion.scale_period,
            }
            for index, window in enumerate(scenario.track.windows)
        ],
    }
    summary = {
        "environment": {
            "conda_environment": "wyh",
            "python": platform.python_version(),
            "platform": platform.platform(),
            "machine": platform.machine(),
            "cpu": "12th Gen Intel(R) Core(TM) i9-12900KF, 24 logical CPUs",
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "matplotlib": matplotlib.__version__,
            "python_flint": flint.__version__,
        },
        "scenario": scenario_summary,
        "sc_dynatogt": {
            "total_time": sc_minco.total_time,
            "solve_wall_seconds": sc_data["solve_wall_seconds"],
            "optimizer_success": sc_data["optimizer_success"],
            "full_certificate": sc_data["full_certificate"],
            "geometry_certificate": sc_data["geometry_certificate"],
            "clearance_constraint_status": "CLEARANCE_VIOLATION_PROVED",
            "physical_collision_status": "PHYSICAL_INTERSECTION_CONFIRMED",
            "physical_intersection_audit": {
                "method": "direct evaluation of an original Bezier primitive against the oriented cuboid",
                "trajectory_segment": 4,
                "normalized_time": 0.0703125,
                "global_time": 8.801017673749083,
                "window_index": 5,
                "window_name": "bezier_diamond",
                "boundary_segment": 1,
                "boundary_parameter": 0.390625,
                "body_local_boundary_point": [
                    0.23115142233388697,
                    0.2580484304599515,
                    -0.04928236276910302,
                ],
                "axis_interior_margins": [
                    0.03388857766611303,
                    0.006991569540048481,
                    0.00961763723089698,
                ],
                "minimum_interior_margin": 0.006991569540048481,
                "point_to_cuboid_distance": 0.0,
            },
            "convergence_audit": {
                "purpose": "exclude the 400-iteration cutoff as the cause of collision",
                "additional_iterations": 50,
                "additional_evaluations": 70,
                "additional_wall_seconds": 15.667079815000761,
                "optimizer_success": True,
                "optimizer_message": "CONVERGENCE: TOGT PAST-32 RELATIVE COST REDUCTION 9.085e-06 < 1.000e-05",
                "total_time": 16.22751052679214,
                "physical_collision_status": "PHYSICAL_INTERSECTION_CONFIRMED",
                "intersection_global_time": 8.800647540202599,
                "window_index": 5,
                "window_name": "bezier_diamond",
                "boundary_segment": 1,
                "boundary_parameter": 0.39286823948096133,
                "body_local_boundary_point": [
                    0.25073811571478294,
                    0.25073811571514526,
                    -0.04459811571470164,
                ],
                "axis_interior_margins": [
                    0.014301884285217059,
                    0.014301884284854738,
                    0.014301884285298362,
                ],
                "minimum_interior_margin": 0.014301884284854738,
                "point_to_cuboid_distance": 0.0,
                "nearby_points_inside": "25/25",
                "minimum_nearby_interior_margin": 0.008556644940143465,
                "sampled_dynamics": {
                    "sample_count": 10007,
                    "max_velocity": 16.194354452278844,
                    "max_body_rate_xy": 2.8539473281816106,
                    "max_abs_body_rate_z": 1.4006707829350065,
                    "min_rotor_thrust": [0.17245234731908599, 1.6043591059578466, 0.3217070563990183, 1.4520078065145487],
                    "max_rotor_thrust": [5.0176870149308845, 4.997172227438087, 5.0292610270956635, 5.0132347388030505],
                },
            },
            "sampled_minimum_clearance": sc_profile["minimum"],
            "sampled_dynamics": sc_data["sampled_dynamics"],
        },
        "sip_dynatogt": {
            "total_time": final_result.total_time,
            "full_certificate": report.to_dict(),
            "collision_status": "NO_COLLISION_CERTIFIED",
            "sampled_minimum_clearance": sip_profile["minimum"],
            "sampled_dynamics": {key: value.tolist() if isinstance(value,np.ndarray) else value for key,value in sip_dynamics.items()},
            "planning_clearance_buffer": config.planning_clearance_buffer,
            "stage_wall_seconds": stages,
            "sip_only_wall_seconds": sum(value for key,value in stages.items() if key != "sc_solve"),
            "end_to_end_wall_seconds": sum(stages.values()),
        },
    }
    (root / "summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    (root / "EXPERIMENT_REPORT.md").write_text(
        f"""# 宽域乱序快速时变窗口对比（修订实验结果）

## 赛道与模型

- 起点=终点：`{scenario.track.start.tolist()}`。
- 穿越顺序：`{' -> '.join(scenario_summary['order_names'])}`（编号 `{list(scenario.track.order)}`）。
- 窗口中心跨度：`{scenario_summary['center_span_xyz']} m`；6 个窗口、24 个原始连续边界段。
- 整机长方体半尺寸：`{list(scenario.body.half_extents)} m`；原始净裕度 `0.015 m`。
- 所有窗口同时平移、完整 RPY 旋转和缩放，最小尺度不小于 `0.40`。

## 最终结果

| 指标 | SC-DynaTOGT | SIP-DynaTOGT |
|---|---:|---:|
| 飞行时间 | {sc_minco.total_time:.9f} s | {final_result.total_time:.9f} s |
| 优化器状态 | {sc_data['optimizer_success']} | {final_result.optimizer_success} |
| 15 mm 连续净距约束 | **CLEARANCE_VIOLATION_PROVED** | **CERTIFIED_FEASIBLE** |
| 实体相交 | **PHYSICAL_INTERSECTION_CONFIRMED** | **NO_COLLISION_CERTIFIED** |
| 全部硬约束 | {sc_data['full_certificate']['status']} | **{report.status.value}** |
| 高密度诊断最小净距 | {sc_profile['minimum']:.9f} m | {sip_profile['minimum']:.9f} m |
| 最大单旋翼推力 | {max(sc_data['sampled_dynamics']['max_rotor_thrust']):.9f} N | {max(sip_dynamics['max_rotor_thrust']):.9f} N |

SIP 最终证书：128-bit Arb，`{report.checked_cells}` 个区间单元，最大深度 `{report.maximum_depth}`，
最小安全平方裕量 `{report.minimum_safety_squared_margin:.12g}`，最小动力学裕量 `{report.minimum_dynamic_margin:.12g}`。

表中 SC 是原实验保存的 400 次迭代截停候选，不是已收敛解。SIP 候选通过的是原始
曲线×连续时间全域证书；表中的高密度距离只用于诊断与绘图。

## SC 实体相交审计

通用的正安全残差只能单独证明 15 mm 净距不足，不必然证明实体相交。因此实体结论
另外通过原始 Bezier 边界点在机体坐标系中的严格点内包含来建立。对原候选：

```text
轨迹段 / 归一化时间 = 4 / 0.0703125
全局时间              = 8.801017673749083 s
窗口 / 原始曲线段      = W5(bezier_diamond) / 1
Bezier 参数             = 0.390625
边界点的机体坐标      = (0.2311514223, 0.2580484305, -0.0492823628) m
长方体半尺寸            = (0.2650400000, 0.2650400000,  0.0589000000) m
三轴内部裕量            = (33.888578, 6.991570, 9.617637) mm
点到长方体距离          = 0 m
```

三轴坐标的绝对值均严格小于长方体半尺寸，所以这是实体相交，不只是净距小于 15 mm。

`comparison/contact_timeline.png` 给出了每个轨迹时刻对全部窗口、全部原始边界段的采样最小距离。第三行单独标出
采样 `d=0` 接触；深红色虚线及 `X` 是上述直接原始曲线点内包含见证。SIP 无采样零距离，且安全性由另外的连续域证书保证。因此图用于直观定位，而不代替证书。

## SC 收敛复核

从保存决策向量继续运行同一 SC 目标，50 次迭代、70 次函数评估后触发原 TOGT
`past-32` 收敛准则。收敛后总时间为 `16.227510527 s`，但在 W5 穿越后
`0.107619754 s` 仍发生实体相交；相交点最小内部深度为 `14.301884 mm`，
附近 25/25 个检查点也在机体内，附近点最小内部深度为 `8.556645 mm`。

收敛复核轨迹的单旋翼推力诊断范围为 `0.172452347–5.029261027 N`，同时违反
`0.25 N` 下限和 `5.0 N` 上限。

## 结论边界

这是专门设计的高强度压力测试，而非随机抽样的无偏基准。它证明该名义模型下存在 SC 收敛后
仍实体碰撞的失败案例，不能由单个场景外推为 SC 的一般碰撞率。

## SIP v7/v8 续跑来源

这不是一次从头开始的 SIP 运行。v5、v7、v8 的 `problem_sha256` 都是
`efca94b2658a8d1ae598e45907e9f8b65fbbbdb521d0b49d8b7de35b0401ac0e`，因而赛道、原始边界、机体、动力学硬界和最终 `15 mm` 验证约束没有改动。但求解过程修改了：

| 阶段 | 初始点 | 规划净距缓冲 | 活动见证 | 结果 |
|---|---|---:|---:|---|
| v5 | SC 热启动 | `1 mm` | 1,557 | `UNRESOLVED` |
| v7 | v5 候选 | `5 mm` | 1,557 + 7 个 Arb 安全见证，最终 1,611 | `UNRESOLVED` |
| v8 | v7 候选 | `20 mm` | 1,611，最终 1,613 | `UNRESOLVED` |

缓冲是 SLSQP 中使用的规划裕度，分别使为 `16/20/35 mm`；最终证书始终按原始 `15 mm` 判定。v7 的 7 个手工批量加入见证来自 v5 后验证发现的违例：4 个位于 W5 的第 2 段 Bezier，3 个位于 W0 的第 5 段边界。这是 SIP 约束生成的“发现违例→加入约束→重优化”步骤，但它确实是在看到 v5 结果后才继续的。

v8 之后还单独将认证预算从最大深度 26、2,000,000 个单元提升到深度 32、4,000,000 个单元，才得到最终 `CERTIFIED_FEASIBLE`。这只改证书预算，不改轨迹，但同样属于后续续跑。因此它的安全结论是有效的，但其计算时间必须累计入完整修复流程，不能报成单次 SIP 求解时间。

## 计算时间

```json
{json.dumps(stages,ensure_ascii=False,indent=2)}
```

SIP 阶段累计 `{summary['sip_dynatogt']['sip_only_wall_seconds']:.3f} s`，计入 SC 初值的端到端累计
`{summary['sip_dynatogt']['end_to_end_wall_seconds']:.3f} s`。这是压力实验的真实多阶段计算成本，不是单次理想化运行时间。

## 运行环境

- Conda：`wyh`；Python `{platform.python_version()}`。
- Linux `{platform.release()}` x86_64；Intel Core i9-12900KF，24 逻辑 CPU。
- NumPy `{np.__version__}`，SciPy `{scipy.__version__}`，Matplotlib `{matplotlib.__version__}`，python-flint `{flint.__version__}`。
""",
        encoding="utf-8",
    )
    return summary


def main(argv: list[str] | None = None) -> int:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir",type=Path,default=Path(__file__).resolve().parent/"results"/"wide_scrambled_certified_final")
    parser.add_argument("--no-gif",action="store_true")
    args=parser.parse_args(argv)
    result=finalize(args.outdir,make_gif=not args.no_gif)
    print(json.dumps({"outdir":str(args.outdir),"sip_status":result["sip_dynatogt"]["full_certificate"]["status"]},ensure_ascii=False))
    return 0


if __name__ == "__main__": raise SystemExit(main())


__all__=["finalize","main"]
