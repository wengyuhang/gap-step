"""A0--A3 figures and an explicit AtlasDynaTOGT comparability note."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack

from .candidate_pool import Candidate


plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": [
            "Noto Sans CJK SC",
            "Microsoft YaHei",
            "SimHei",
            "DejaVu Sans",
        ],
        "axes.unicode_minus": False,
    }
)


METHOD_COLORS = {
    "A0": "#7f8c8d",
    "A1": "#3498db",
    "A2": "#f39c12",
    "A3": "#16a085",
}


def atlas_compatibility_note() -> dict[str, Any]:
    return {
        "role": "辅助方法结构比较",
        "direct_numeric_ranking": False,
        "comparability": (
            "AtlasDynaTOGT 使用三次 Hermite、各向异性窗口缩放及速度/加速度/jerk指标；"
            "MSR/SC 使用 degree-7 MINCO、均匀缩放、四旋翼微分平坦性、单旋翼推力和角速度。"
            "动力学模型、场景和接口不同，因此不能直接宣称性能优越。"
        ),
        "shared_scope": "均处理无洞简单非凸动态窗口与指定顺序穿越。",
    }


def _native(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [row for row in rows if row["comparison_protocol"] == "native"]


def _method_means(rows: list[dict[str, Any]], key: str) -> list[float]:
    values = []
    for method in ("A0", "A1", "A2", "A3"):
        selected = [float(row[key]) for row in rows if row["method"] == method]
        values.append(float(np.mean(selected)))
    return values


def _bar_figure(
    values: list[float],
    ylabel: str,
    title: str,
    path: Path,
    *,
    percent: bool = False,
) -> Path:
    methods = ("A0", "A1", "A2", "A3")
    figure, axis = plt.subplots(figsize=(7.2, 4.4), constrained_layout=True)
    bars = axis.bar(methods, values, color=[METHOD_COLORS[item] for item in methods])
    axis.set_ylabel(ylabel)
    axis.set_title(title)
    axis.grid(axis="y", alpha=0.25)
    for bar, value in zip(bars, values):
        label = f"{100.0 * value:.1f}%" if percent else f"{value:.3f}"
        axis.text(
            bar.get_x() + bar.get_width() / 2.0,
            bar.get_height(),
            label,
            ha="center",
            va="bottom",
            fontsize=9,
        )
    if percent:
        axis.set_ylim(0.0, max(1.05, max(values, default=1.0) * 1.15))
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=170)
    plt.close(figure)
    return path


def plot_aggregate_comparisons(rows: list[dict[str, Any]], output: Path) -> list[Path]:
    native = _native(rows)
    output.mkdir(parents=True, exist_ok=True)
    paths = [
        _bar_figure(
            _method_means(native, "total_time"),
            "平均总飞行时间 / s",
            "A0–A3 总时间对比（原生配置）",
            output / "total_time_comparison.png",
        ),
        _bar_figure(
            _method_means(native, "wall_clock_seconds"),
            "平均墙钟时间 / s",
            "A0–A3 计算时间对比（原生配置）",
            output / "computation_time_comparison.png",
        ),
        _bar_figure(
            _method_means(native, "sampled_dynamic_limits_satisfied"),
            "高密度采样动力学可行率",
            "A0–A3 动力学可行率",
            output / "sampled_dynamic_feasibility_rate.png",
            percent=True,
        ),
    ]

    repaired = [
        row
        for row in native
        if row["method"] in {"A2", "A3"} and bool(row["repair_triggered"])
    ]
    figure, axis = plt.subplots(figsize=(10.5, 5.2), constrained_layout=True)
    if repaired:
        groups = sorted({(str(row["method"]), str(row["scene"])) for row in repaired})
        x = np.arange(len(groups))
        width = 0.38
        before = [
            float(
                np.mean(
                    [
                        float(row["repair_before_max_rotor_thrust"])
                        for row in repaired
                        if (str(row["method"]), str(row["scene"])) == group
                    ]
                )
            )
            for group in groups
        ]
        after = [
            float(
                np.mean(
                    [
                        float(row["repair_after_max_rotor_thrust"])
                        for row in repaired
                        if (str(row["method"]), str(row["scene"])) == group
                    ]
                )
            )
            for group in groups
        ]
        axis.bar(x - width / 2.0, before, width, label="修复前", color="#c0392b")
        axis.bar(x + width / 2.0, after, width, label="修复后", color="#16a085")
        axis.set_xticks(x, [f"{method}\n{scene}" for method, scene in groups], rotation=30, ha="right")
        axis.legend()
    else:
        axis.text(0.5, 0.5, "本次原生运行未触发修复", ha="center", va="center", transform=axis.transAxes)
        axis.set_xticks([])
    axis.set_ylabel("四旋翼中最大单旋翼推力 / N")
    axis.set_title("动力学修复前后平均峰值推力对比")
    axis.grid(axis="y", alpha=0.25)
    thrust_path = output / "repair_thrust_before_after.png"
    figure.savefig(thrust_path, dpi=170)
    plt.close(figure)
    paths.append(thrust_path)
    return paths


def plot_trajectory_comparison(
    track: SCWindowTrack,
    candidates: Mapping[str, Candidate],
    path: Path,
) -> Path:
    figure = plt.figure(figsize=(9.0, 6.6), constrained_layout=True)
    axis = figure.add_subplot(111, projection="3d")
    for method in ("A0", "A1", "A2", "A3"):
        candidate = candidates[method]
        samples = candidate.result.trajectory.sample(num_samples=401)
        axis.plot(
            np.real(samples.position[:, 0]),
            np.real(samples.position[:, 1]),
            np.real(samples.position[:, 2]),
            color=METHOD_COLORS[method],
            linewidth=1.8,
            label=(
                f"{method}  T={candidate.result.total_time:.3f}s  "
                f"sampled={candidate.feasibility.sampled_dynamic_limits_satisfied}"
            ),
        )
    representative = candidates["A3"].result
    for crossing, window_index in enumerate(track.order):
        instant = float(representative.traversal_times[crossing])
        window = track.windows[window_index]
        boundary = window.physical_boundary_at(instant)
        if boundary is None:
            boundary = window.polygon_at(instant)
        closed = np.vstack((boundary, boundary[0]))
        axis.plot(closed[:, 0], closed[:, 1], closed[:, 2], color="#d35400", linewidth=1.0)
    axis.scatter(*track.start, color="black", marker="o", s=35, label="起点")
    axis.scatter(*track.goal, color="black", marker="x", s=45, label="终点")
    axis.set_xlabel("x / m")
    axis.set_ylabel("y / m")
    axis.set_zlabel("z / m")
    axis.set_title(f"{track.name}：A0–A3 轨迹对比")
    axis.legend(loc="best", fontsize=8)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=170)
    plt.close(figure)
    return path


__all__ = [
    "atlas_compatibility_note",
    "plot_aggregate_comparisons",
    "plot_trajectory_comparison",
]
