"""Non-overwriting result directories and honest Chinese reports."""

from __future__ import annotations

import csv
from dataclasses import asdict, is_dataclass
from datetime import datetime
import json
from pathlib import Path
from typing import Any, Iterable

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parent
RESULTS_ROOT = PACKAGE_ROOT / "results"


def jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        return jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(key): jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [jsonable(item) for item in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def timestamped_run_directory(parent: Path, suite: str) -> Path:
    """Create a fresh microsecond timestamp directory, never overwrite."""

    root = Path(parent).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    for suffix in range(1000):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        tail = "" if suffix == 0 else f"_{suffix:03d}"
        candidate = root / f"{stamp}_{suite}{tail}"
        try:
            candidate.mkdir()
        except FileExistsError:
            continue
        return candidate
    raise RuntimeError("could not allocate a unique timestamp result directory")


def write_json(path: Path, value: Any) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(jsonable(value), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return path


def write_csv(path: Path, rows: Iterable[dict[str, Any]]) -> Path:
    values = list(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = list(dict.fromkeys(key for row in values for key in row))
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        for row in values:
            encoded = {}
            for key in keys:
                value = jsonable(row.get(key, ""))
                encoded[key] = (
                    json.dumps(value, ensure_ascii=False)
                    if isinstance(value, (list, dict))
                    else value
                )
            writer.writerow(encoded)
    return path


def summarize_runs(rows: list[dict[str, Any]]) -> dict[str, Any]:
    groups: dict[str, dict[str, Any]] = {}
    protocols = sorted({str(row["comparison_protocol"]) for row in rows})
    methods = ("A0", "A1", "A2", "A3")
    for protocol in protocols:
        for method in methods:
            selected = [
                row
                for row in rows
                if row["comparison_protocol"] == protocol and row["method"] == method
            ]
            if not selected:
                continue
            key = f"{protocol}/{method}"
            success = np.asarray([bool(row["success"]) for row in selected], dtype=float)
            legal = np.asarray(
                [
                    bool(row["window_order_legal"])
                    and bool(row["window_internal_legal"])
                    for row in selected
                ],
                dtype=float,
            )
            dynamic = np.asarray(
                [bool(row["sampled_dynamic_limits_satisfied"]) for row in selected],
                dtype=float,
            )
            groups[key] = {
                "protocol": protocol,
                "method": method,
                "runs": len(selected),
                "success_rate": float(np.mean(success)),
                "window_legality_rate": float(np.mean(legal)),
                "sampled_dynamic_feasibility_rate": float(np.mean(dynamic)),
                "mean_total_time": float(np.mean([float(row["total_time"]) for row in selected])),
                "mean_wall_clock_seconds": float(
                    np.mean([float(row["wall_clock_seconds"]) for row in selected])
                ),
                "mean_iterations": float(np.mean([float(row["iterations"]) for row in selected])),
                "mean_evaluations": float(np.mean([float(row["evaluations"]) for row in selected])),
                "repair_trigger_rate": float(
                    np.mean([bool(row["repair_triggered"]) for row in selected])
                ),
                "mean_repair_scale_factor": float(
                    np.mean([float(row["repair_scale_factor"]) for row in selected])
                ),
            }

    native = {
        method: groups.get(f"native/{method}", {}) for method in methods
    }
    baseline = native.get("A0", {})
    complete = native.get("A3", {})
    performance = {}
    if baseline and complete:
        performance = {
            "sampled_dynamic_feasibility_rate_change": (
                complete["sampled_dynamic_feasibility_rate"]
                - baseline["sampled_dynamic_feasibility_rate"]
            ),
            "mean_total_time_change_seconds": (
                complete["mean_total_time"] - baseline["mean_total_time"]
            ),
            "mean_wall_clock_multiplier": (
                complete["mean_wall_clock_seconds"]
                / max(baseline["mean_wall_clock_seconds"], 1.0e-12)
            ),
        }
    failures = [
        {
            "method": row["method"],
            "scene": row["scene"],
            "seed": row["seed"],
            "reason": row["failure_reasons"] or "sampled feasibility not satisfied",
        }
        for row in rows
        if row["comparison_protocol"] == "native" and not bool(row["success"])
    ]
    return {
        "claim_scope": "所有可行性结论仅指高密度采样节点，不是连续时间严格证明。",
        "groups": groups,
        "native_performance_change_A3_vs_A0": performance,
        "failed_native_runs": failures,
    }


def _percent(value: float) -> str:
    return f"{100.0 * value:.1f}%"


def _integer_ranges(values: Iterable[int]) -> str:
    ordered = sorted(set(int(value) for value in values))
    if not ordered:
        return ""
    ranges: list[str] = []
    start = previous = ordered[0]
    for value in ordered[1:]:
        if value == previous + 1:
            previous = value
            continue
        ranges.append(str(start) if start == previous else f"{start}-{previous}")
        start = previous = value
    ranges.append(str(start) if start == previous else f"{start}-{previous}")
    return ",".join(ranges)


def write_report(
    path: Path,
    *,
    suite: str,
    summary: dict[str, Any],
    rows: list[dict[str, Any]],
    atlas_note: dict[str, Any],
    figure_explanations: str | None = None,
) -> Path:
    groups = summary["groups"]
    native = {method: groups[f"native/{method}"] for method in ("A0", "A1", "A2", "A3")}
    lines = [
        f"# MSR-DynaTOGT {suite} 实验报告",
        "",
        "## 方法差异",
        "",
        "SC-DynaTOGT 对一个初值执行一次局部 L-BFGS-B，动力学限制为软惩罚，优化后主要做诊断。MSR-DynaTOGT 系统生成中心、随机扰动、转弯感知和区域分散初值，维护去重候选池；对超限候选进行高密度采样检查、时间修复、二分缩放和一次完整联合再优化。SC 映射、动态窗口、degree-7 MINCO、四旋翼微分平坦性及单旋翼推力/角速度代价保持一致。",
        "",
        "> 本报告中的“可行”仅表示高密度采样可行，不是连续时间严格证明。`optimizer_success` 也没有被当作动力学可行。",
        "",
        "## 原生 A0–A3 结果",
        "",
        "| 方法 | 运行数 | 窗口合法率 | 采样动力学可行率 | 平均总时间 / s | 平均计算时间 / s |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for method in ("A0", "A1", "A2", "A3"):
        item = native[method]
        lines.append(
            f"| {method} | {item['runs']} | {_percent(item['window_legality_rate'])} | "
            f"{_percent(item['sampled_dynamic_feasibility_rate'])} | "
            f"{item['mean_total_time']:.6f} | {item['mean_wall_clock_seconds']:.3f} |"
        )

    change = summary.get("native_performance_change_A3_vs_A0", {})
    lines.extend(
        [
            "",
            "## 公平性",
            "",
            "输出同时包含 `matched_starts`（四个消融使用完全相同的初值列表和启动次数）与 `matched_time`（按实测单次耗时回放，在共同时间预算内保留可完成的启动，至少接纳第一次启动）。原生 `native` 行用于展示各方法按定义运行时的实际代价。时间预算比较复用真实已测运行，不用估算值替代墙钟时间。",
            "",
            "AtlasDynaTOGT 仅作辅助结构比较：它使用三次 Hermite、各向异性缩放及速度/加速度/jerk 指标，没有本实验的 degree-7 MINCO、单旋翼推力和角速度接口，场景定义也不同。因此没有把它的数值与 A0–A3 直接排名，也没有宣称 MSR 对 Atlas 性能优越。",
            "",
            "## 改善与计算成本",
            "",
        ]
    )
    if change:
        lines.append(
            f"A3 相对 A0 的高密度采样动力学可行率变化为 "
            f"{change['sampled_dynamic_feasibility_rate_change']:+.3f}，平均总时间变化 "
            f"{change['mean_total_time_change_seconds']:+.6f} s，平均计算时间倍率 "
            f"{change['mean_wall_clock_multiplier']:.2f}×。"
        )
        if native["A0"]["sampled_dynamic_feasibility_rate"] < 1.0:
            lines.append(
                "A0 含采样动力学不可行轨迹，因此上述总时间差只是原始数值差，不能解释为可行解之间的性能优越性。"
            )
    else:
        lines.append("当前没有足够的 A0/A3 行计算改善量。")

    lines.extend(["", "## 分场景表现", ""])
    for scene in sorted({str(row["scene"]) for row in rows}):
        scene_rows = [
            row for row in rows if row["scene"] == scene and row["comparison_protocol"] == "native"
        ]
        a0_rows = [row for row in scene_rows if row["method"] == "A0"]
        a3_rows = [row for row in scene_rows if row["method"] == "A3"]
        a0_time = float(np.mean([float(row["total_time"]) for row in a0_rows]))
        a3_time = float(np.mean([float(row["total_time"]) for row in a3_rows]))
        a0_feasible = float(
            np.mean([bool(row["sampled_dynamic_limits_satisfied"]) for row in a0_rows])
        )
        a3_feasible = float(
            np.mean([bool(row["sampled_dynamic_limits_satisfied"]) for row in a3_rows])
        )
        relation = "更短" if a3_time < a0_time else "更长或相同"
        qualifier = (
            "；A0 存在不可行运行，时间差仅作数值记录"
            if a0_feasible < 1.0
            else ""
        )
        lines.append(
            f"- `{scene}`（{len(a0_rows)} 个种子）：A0/A3 采样动力学可行率分别为 "
            f"`{_percent(a0_feasible)}`/`{_percent(a3_feasible)}`；"
            f"A3 平均总时间相对 A0 {relation}（{a0_time:.4f} -> {a3_time:.4f} s）"
            f"{qualifier}。"
        )

    lines.extend(["", "## 失败种子及原因", ""])
    failures = summary["failed_native_runs"]
    if not failures:
        lines.append("原生 A0–A3 没有高密度采样失败运行。")
    else:
        grouped_failures: dict[tuple[str, str, str], list[int]] = {}
        for failure in failures:
            key = (
                str(failure["method"]),
                str(failure["scene"]),
                str(failure["reason"]),
            )
            grouped_failures.setdefault(key, []).append(int(failure["seed"]))
        for (method, scene, reason), failed_seeds in sorted(grouped_failures.items()):
            lines.append(
                f"- {method} / `{scene}` / seeds={_integer_ranges(failed_seeds)} "
                f"（{len(failed_seeds)} 个）：{reason}"
            )

    lines.extend(
        [
            "",
            "## 已知限制",
            "",
            "- 高密度采样可能漏掉采样点之间的瞬时峰值，不能替代连续时间认证。",
            "- 局部时间修复沿固定的分段缩放方向二分，不保证得到全局最小可行时间。",
            "- 多初值和修复会增加计算成本；L-BFGS-B 仍是非凸局部优化。",
            "- 当前范围仍是无洞、无自交、偏置后单连通的窗口，按指定顺序各穿越一次。",
            "- smoke 为控制快速验证成本，将每次 L-BFGS-B 上限设为 24 次迭代；`optimizer_success=false` 可能表示达到该烟测上限，必须与独立采样可行性分开解读。" if suite == "smoke" else "- formal 使用 SC-DynaTOGT 的历史代价停止准则，不设置人为迭代上限。",
            "- Atlas 辅助说明：" + str(atlas_note.get("comparability", "接口不同，不直接数值排名。")),
            "",
        ]
    )
    if figure_explanations:
        explanation_body = figure_explanations.splitlines()
        if explanation_body and explanation_body[0].startswith("# "):
            explanation_body = explanation_body[1:]
        lines.extend(
            [
                "",
                "## 每张结果图的通俗解释",
                "",
                "以下解释与独立文件 `FIGURE_EXPLANATIONS.md` 内容一致。",
                "",
                *explanation_body,
            ]
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def figure_explanations_markdown(
    rows: list[dict[str, Any]],
    *,
    selected_scenes: Iterable[str],
    representative_seed: int,
) -> str:
    """Explain every generated result figure without overstating evidence."""

    native = [row for row in rows if row["comparison_protocol"] == "native"]
    lines = [
        "# 实验结果图通俗解释",
        "",
        "先记住两点：图中的 `sampled=True` 只表示高密度采样节点通过限制，不是连续时间数学证明；只有窗口合法且采样动力学可行的轨迹，才适合继续比较飞行时间。",
        "",
    ]
    for scene in selected_scenes:
        figure_name = f"figures/trajectory_comparison_{scene}.png"
        scene_rows = [
            row
            for row in native
            if row["scene"] == scene and int(row["seed"]) == representative_seed
        ]
        by_method = {str(row["method"]): row for row in scene_rows}
        lines.extend(
            [
                f"## `{figure_name}`",
                "",
                f"这张三维图展示 `{scene}` 在代表种子 `{representative_seed}` 下的四条轨迹。黑色圆点/叉号是起终点，橙色轮廓是 A3 实际穿越时刻的物理窗口，A0–A3 彩线是四种方法的飞行路线。",
                "",
                "通俗地看：先检查图例中的 `sampled`。如果是 `False`，即使曲线更短，也像一辆抄近路但发动机超载的车，不能排在可行轨迹前面；都为 `True` 时，才看 `T`，越小表示飞得越快。",
                "",
            ]
        )
        if len(by_method) == 4:
            for method in ("A0", "A1", "A2", "A3"):
                row = by_method[method]
                lines.append(
                    f"- {method}：总时间 `{float(row['total_time']):.6f} s`，"
                    f"窗口合法=`{bool(row['window_order_legal']) and bool(row['window_internal_legal'])}`，"
                    f"高密度采样动力学可行=`{bool(row['sampled_dynamic_limits_satisfied'])}`。"
                )
            a0 = by_method["A0"]
            a3 = by_method["A3"]
            if not bool(a0["sampled_dynamic_limits_satisfied"]):
                lines.append(
                    "- 本图的 A0 没有通过采样动力学限制，因此 A0 与 A3 的时间差只能描述数值，不能称为可行轨迹之间的速度优势。"
                )
            elif bool(a3["sampled_dynamic_limits_satisfied"]):
                delta = float(a3["total_time"]) - float(a0["total_time"])
                lines.append(f"- A0 与 A3 都可行时，A3 相对 A0 的时间差为 `{delta:+.6f} s`。")
        lines.extend(
            [
                "",
                "图的限制：它只画一个代表种子，不能替代所有正式种子的统计；窗口轮廓取自 A3 的穿越时刻，不表示四种方法在完全相同时刻看到的窗口姿态。",
                "",
            ]
        )

    method_rows = {
        method: [row for row in native if row["method"] == method]
        for method in ("A0", "A1", "A2", "A3")
    }
    mean_times = {
        method: float(np.mean([float(row["total_time"]) for row in values]))
        for method, values in method_rows.items()
    }
    mean_compute = {
        method: float(np.mean([float(row["wall_clock_seconds"]) for row in values]))
        for method, values in method_rows.items()
    }
    feasible_rates = {
        method: float(
            np.mean([bool(row["sampled_dynamic_limits_satisfied"]) for row in values])
        )
        for method, values in method_rows.items()
    }
    repaired = [
        row
        for row in native
        if row["method"] in {"A2", "A3"} and bool(row["repair_triggered"])
    ]
    time_values = "、".join(
        f"{method}={mean_times[method]:.6f} s" for method in ("A0", "A1", "A2", "A3")
    )
    compute_values = "、".join(
        f"{method}={mean_compute[method]:.3f} s" for method in ("A0", "A1", "A2", "A3")
    )
    feasibility_values = "、".join(
        f"{method}={100.0 * feasible_rates[method]:.1f}%"
        for method in ("A0", "A1", "A2", "A3")
    )
    if repaired:
        thrust_values = (
            f"共 {len(repaired)} 个触发修复的 A2/A3 原生运行，"
            f"最大单旋翼推力的组合平均由 "
            f"{np.mean([float(row['repair_before_max_rotor_thrust']) for row in repaired]):.4f} N "
            f"变为 {np.mean([float(row['repair_after_max_rotor_thrust']) for row in repaired]):.4f} N。"
        )
    else:
        thrust_values = "本次原生运行没有触发修复，因此没有修复前后数值可比。"

    aggregate_explanations = [
        (
            "figures/total_time_comparison.png",
            f"四根柱是 A0–A3 在全部运行上的平均飞行时间：{time_values}。柱越低表示轨迹平均越快。但必须同时对照动力学可行率图：如果某方法大量不可行，低时间可能只是靠超推力换来的，不能单独宣布它更好。",
        ),
        (
            "figures/computation_time_comparison.png",
            f"柱高表示求解器实际花费的平均墙钟时间：{compute_values}。A1/A3 要尝试多个初值，A2/A3 还要修复和再次优化，所以通常更高。它回答的是“为了得到结果多等多久”，不是无人机飞多久。",
        ),
        (
            "figures/sampled_dynamic_feasibility_rate.png",
            f"纵轴是通过高密度动力学检查的运行比例：{feasibility_values}。越接近 100% 越稳定。它直接反映时间修复是否减少推力、角速度或速度超限，但仍然只是采样证据。",
        ),
        (
            "figures/repair_thrust_before_after.png",
            f"每组红柱是一个“方法×场景”组的修复前平均最大单旋翼推力，绿柱是修复后；默认上限是 5 N。{thrust_values}绿色降到 5 N 附近或以下，表示放慢轨迹有效削弱峰值。若某次修复由单旋翼下限而非上限触发，仅看最大值不能说明全部原因，应回到 runs.csv 查看最小推力和 failure_reasons。",
        ),
    ]
    for name, explanation in aggregate_explanations:
        lines.extend([f"## `{name}`", "", explanation, ""])
    return "\n".join(lines).rstrip() + "\n"


def write_figure_explanations(
    path: Path,
    rows: list[dict[str, Any]],
    *,
    selected_scenes: Iterable[str],
    representative_seed: int,
) -> tuple[Path, str]:
    content = figure_explanations_markdown(
        rows,
        selected_scenes=selected_scenes,
        representative_seed=representative_seed,
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    return path, content


__all__ = [
    "PACKAGE_ROOT",
    "RESULTS_ROOT",
    "jsonable",
    "figure_explanations_markdown",
    "summarize_runs",
    "timestamped_run_directory",
    "write_csv",
    "write_figure_explanations",
    "write_json",
    "write_report",
]
