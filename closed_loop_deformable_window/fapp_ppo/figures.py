"""Generate readable training and paper figures for independent-window FAPP-PPO."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch, Patch

from .config import load_config
from .scenario import build_scenario


COLORS = {
    "blue": "#1565c0",
    "teal": "#00897b",
    "orange": "#ef6c00",
    "red": "#c62828",
    "purple": "#6a1b9a",
    "green": "#2e7d32",
    "gray": "#546e7a",
    "light": "#f5f7f8",
}

plt.rcParams.update(
    {
        "font.sans-serif": ["Microsoft YaHei", "Noto Sans CJK SC", "DejaVu Sans"],
        "axes.unicode_minus": False,
    }
)


def _read_csv(path: str | Path) -> list[dict[str, str]]:
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def _finite_column(rows: list[dict[str, str]], key: str) -> np.ndarray:
    values = []
    for row in rows:
        try:
            values.append(float(row[key]))
        except (KeyError, TypeError, ValueError):
            values.append(float("nan"))
    return np.asarray(values, dtype=float)


def _moving_average(values: np.ndarray, window: int = 5) -> np.ndarray:
    output = np.full_like(values, np.nan, dtype=float)
    for index in range(len(values)):
        segment = values[max(0, index - window + 1) : index + 1]
        finite = segment[np.isfinite(segment)]
        if len(finite):
            output[index] = finite.mean()
    return output


def _stage_moving_average(
    values: np.ndarray, stages: list[str], window: int = 5
) -> np.ndarray:
    """Smooth within a curriculum stage without blending different tasks."""

    output = np.full_like(values, np.nan, dtype=float)
    start = 0
    while start < len(stages):
        end = start + 1
        while end < len(stages) and stages[end] == stages[start]:
            end += 1
        output[start:end] = _moving_average(values[start:end], window=window)
        start = end
    return output


def _stage_background(axis, rows: list[dict[str, str]]) -> None:
    updates = _finite_column(rows, "global_update")
    stages = [row["stage"] for row in rows]
    palette = {
        "static": "#e8f5e9",
        "moving": "#e3f2fd",
        "deforming": "#fff3e0",
        "full": "#f3e5f5",
    }
    stage_names = {
        "static": "静态",
        "moving": "运动",
        "deforming": "形变",
        "full": "完整难度",
    }
    start = 0
    while start < len(stages):
        end = start
        while end + 1 < len(stages) and stages[end + 1] == stages[start]:
            end += 1
        left = updates[start] - 0.5
        right = updates[end] + 0.5
        axis.axvspan(left, right, color=palette.get(stages[start], "#eeeeee"), alpha=0.55)
        axis.text(
            0.5 * (left + right),
            0.98,
            stage_names.get(stages[start], stages[start]),
            transform=axis.get_xaxis_transform(),
            ha="center",
            va="top",
            fontsize=8,
            color="#455a64",
        )
        start = end + 1


def plot_training_curves(
    metrics_csv: str | Path,
    output: str | Path,
    *,
    title: str = "FAPP-PPO 训练诊断",
) -> Path:
    """Plot actual recorded episode reward and PPO diagnostics."""

    rows = _read_csv(metrics_csv)
    if not rows:
        raise ValueError("training metrics CSV is empty")
    updates = _finite_column(rows, "global_update")
    stages = [row["stage"] for row in rows]
    episodes = _finite_column(rows, "episodes")
    returns = _finite_column(rows, "mean_episode_return")
    success = _finite_column(rows, "success_rate")
    returns[episodes <= 0] = np.nan
    success[episodes <= 0] = np.nan
    kl = _finite_column(rows, "approx_kl")
    value_loss = _finite_column(rows, "value_loss")
    prior_loss = _finite_column(rows, "residual_prior_loss")

    figure, axes = plt.subplots(2, 2, figsize=(12.5, 7.5), sharex=True)
    for axis in axes.flat:
        _stage_background(axis, rows)
        axis.grid(alpha=0.25)

    axes[0, 0].plot(
        updates, returns, marker="o", markersize=3, linewidth=0.8,
        color=COLORS["blue"], alpha=0.45, label="单次更新均值",
    )
    axes[0, 0].plot(
        updates, _stage_moving_average(returns, stages), linewidth=2.3,
        color=COLORS["blue"], label="5 次更新滑动平均",
    )
    axes[0, 0].axhline(0.0, color="#455a64", linewidth=0.8)
    axes[0, 0].set_ylabel("平均回合总回报")
    axes[0, 0].set_title("A. 奖励曲线（仅统计已结束回合）")
    axes[0, 0].legend(fontsize=8)

    axes[0, 1].plot(
        updates, success, marker="o", markersize=3, linewidth=1.0,
        color=COLORS["green"], alpha=0.5,
    )
    axes[0, 1].plot(
        updates, _stage_moving_average(success, stages),
        linewidth=2.3,
        color=COLORS["green"],
    )
    axes[0, 1].set_ylim(-0.04, 1.04)
    axes[0, 1].set_ylabel("采样批次闭环成功率")
    axes[0, 1].set_title("B. 闭环成功率")

    axes[1, 0].plot(updates, kl, color=COLORS["purple"], linewidth=1.6)
    if np.any(np.isfinite(kl) & (kl > 0.0)):
        axes[1, 0].set_yscale("log")
    axes[1, 0].set_ylabel("近似 KL")
    axes[1, 0].set_xlabel("PPO 更新次数")
    axes[1, 0].set_title("C. 策略更新幅度")

    axes[1, 1].plot(
        updates, value_loss, color=COLORS["orange"], linewidth=1.6,
        label="价值损失",
    )
    axes[1, 1].plot(
        updates, prior_loss, color=COLORS["teal"], linewidth=1.6,
        label="残差先验损失",
    )
    axes[1, 1].set_yscale("symlog", linthresh=1.0e-3)
    axes[1, 1].set_ylabel("损失")
    axes[1, 1].set_xlabel("PPO 更新次数")
    axes[1, 1].set_title("D. 价值网络拟合与残差正则")
    axes[1, 1].legend(fontsize=8)

    figure.suptitle(title, fontsize=15, weight="bold")
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=190, bbox_inches="tight")
    plt.close(figure)
    return path


def _box(
    axis,
    xy: tuple[float, float],
    width: float,
    height: float,
    text: str,
    *,
    color: str,
    fontsize: float = 10,
) -> None:
    patch = FancyBboxPatch(
        xy,
        width,
        height,
        boxstyle="round,pad=0.018,rounding_size=0.02",
        facecolor=color,
        edgecolor="#263238",
        linewidth=1.2,
    )
    axis.add_patch(patch)
    axis.text(
        xy[0] + width / 2,
        xy[1] + height / 2,
        text,
        ha="center",
        va="center",
        fontsize=fontsize,
        color="#102027",
        wrap=True,
    )


def _arrow(axis, start, end, *, text: str = "", dashed: bool = False) -> None:
    axis.annotate(
        "",
        xy=end,
        xytext=start,
        arrowprops={
            "arrowstyle": "-|>",
            "color": "#455a64",
            "linewidth": 1.5,
            "linestyle": "--" if dashed else "-",
        },
    )
    if text:
        axis.text(
            0.5 * (start[0] + end[0]),
            0.5 * (start[1] + end[1]) + 0.018,
            text,
            ha="center",
            va="bottom",
            fontsize=8,
            color="#455a64",
        )


def plot_algorithm_overview(output: str | Path) -> Path:
    figure, axis = plt.subplots(figsize=(15.0, 7.8))
    axis.set_xlim(0.0, 1.0)
    axis.set_ylim(0.0, 1.0)
    axis.axis("off")
    axis.set_title(
        "FAPP-PPO：独立外生窗口 + 特权预览残差控制",
        fontsize=16,
        weight="bold",
        pad=18,
    )

    _box(
        axis, (0.03, 0.68), 0.18, 0.18,
        "独立窗口库\n每窗独立随机流\n整回合生成后固定",
        color="#e3f2fd",
    )
    _box(
        axis, (0.27, 0.68), 0.17, 0.18,
        "有限未来预览\n位置、法向、形状\n安全面积、开关时间",
        color="#e0f2f1",
    )
    _box(
        axis, (0.50, 0.68), 0.15, 0.18,
        "策略网络\n有界残差\nΔ CTBR",
        color="#ede7f6",
    )
    _box(
        axis, (0.70, 0.68), 0.12, 0.18,
        "名义 CTBR\n+\nRL 残差",
        color="#fff3e0",
    )
    _box(
        axis, (0.87, 0.68), 0.10, 0.18,
        "旋翼混控\n与刚体动力学",
        color="#ffebee",
        fontsize=9,
    )
    _arrow(axis, (0.21, 0.77), (0.27, 0.77), text="查询未来")
    _arrow(axis, (0.44, 0.77), (0.50, 0.77))
    _arrow(axis, (0.65, 0.77), (0.70, 0.77))
    _arrow(axis, (0.82, 0.77), (0.87, 0.77))

    _box(
        axis, (0.27, 0.33), 0.20, 0.17,
        "特权价值网络（仅训练）\n绝对时间 + 完整剩余日程",
        color="#f3e5f5",
    )
    _box(
        axis, (0.54, 0.33), 0.17, 0.17,
        "PPO 目标\n截断策略 + 价值\n+ 熵 + 残差先验",
        color="#fce4ec",
    )
    _box(
        axis, (0.78, 0.33), 0.17, 0.17,
        "采样缓冲区\n奖励、成功、碰撞\n穿越时间裕度",
        color="#e8f5e9",
    )
    _arrow(axis, (0.47, 0.415), (0.54, 0.415), text="V(s)")
    _arrow(axis, (0.78, 0.415), (0.71, 0.415), text="训练批次")
    _arrow(axis, (0.925, 0.68), (0.865, 0.50), text="状态/奖励")
    _arrow(axis, (0.62, 0.50), (0.575, 0.68), text="梯度更新")

    axis.annotate(
        "",
        xy=(0.50, 0.73),
        xytext=(0.92, 0.68),
        arrowprops={
            "arrowstyle": "-|>",
            "color": COLORS["blue"],
            "linewidth": 1.8,
            "connectionstyle": "arc3,rad=0.26",
        },
    )
    axis.text(0.75, 0.91, "闭环状态反馈", color=COLORS["blue"], fontsize=9)
    axis.text(
        0.03,
        0.15,
        "因果规则：",
        fontsize=12,
        weight="bold",
        color=COLORS["red"],
    )
    axis.text(
        0.14,
        0.15,
        "无人机可以观察并适应窗口日程，但不能改变日程。",
        fontsize=12,
        color="#263238",
    )
    axis.plot([0.05, 0.20], [0.58, 0.58], color=COLORS["red"], linestyle="--", linewidth=2)
    axis.text(
        0.215,
        0.58,
        "不存在“无人机状态 → 窗口生成”的箭头",
        va="center",
        fontsize=10,
        color=COLORS["red"],
    )

    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=190, bbox_inches="tight")
    plt.close(figure)
    return path


def _fill_polygon(axis, polygon, *, color: str, alpha: float) -> None:
    if polygon.is_empty:
        return
    coordinates = np.asarray(polygon.exterior.coords)
    axis.fill(
        coordinates[:, 0],
        coordinates[:, 1],
        color=color,
        alpha=alpha,
        edgecolor=color,
        linewidth=1.4,
    )


def plot_window_modeling(config_path: str | Path, output: str | Path, *, seed: int) -> Path:
    config = load_config(config_path)
    scenario = build_scenario(
        seed=seed,
        stage="full",
        environment=config.environment,
        quadrotor=config.quadrotor,
    )
    window = scenario.windows[0]
    first_start, first_end = window.planned_opportunities[0]
    sample_times = (
        max(0.0, first_start - config.environment.opportunity_transition - 0.25),
        max(0.0, first_start - 0.5 * config.environment.opportunity_transition),
        0.5 * (first_start + first_end),
    )
    labels = ("闭合：不可通行", "连续打开过程", "开放：可以通行")

    figure = plt.figure(figsize=(14.5, 8.3))
    grid = figure.add_gridspec(2, 3, height_ratios=(1.0, 1.15), hspace=0.34)
    for index, (time, label) in enumerate(zip(sample_times, labels)):
        axis = figure.add_subplot(grid[0, index])
        state = window.state(time)
        axis.fill(
            state.boundary[:, 0],
            state.boundary[:, 1],
            color="#ffcc80",
            edgecolor=COLORS["orange"],
            alpha=0.55,
            linewidth=1.8,
            label="物理开口",
        )
        _fill_polygon(axis, state.safe_polygon, color=COLORS["teal"], alpha=0.55)
        axis.axhline(0.0, color="#b0bec5", linewidth=0.5)
        axis.axvline(0.0, color="#b0bec5", linewidth=0.5)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlim(-1.45, 1.45)
        axis.set_ylim(-1.25, 1.25)
        axis.set_title(
            f"{label}\nt={time:.2f} s，安全面积={state.safe_polygon.area:.3f} m²",
            fontsize=10,
        )
        axis.grid(alpha=0.2)
        if index == 0:
            axis.legend(
                handles=[
                    Patch(
                        facecolor="#ffcc80",
                        edgecolor=COLORS["orange"],
                        label="物理开口",
                    ),
                    Patch(
                        facecolor=COLORS["teal"],
                        edgecolor=COLORS["teal"],
                        label="安全内缩区",
                    ),
                ],
                fontsize=8,
                loc="upper right",
            )

    timeline = figure.add_subplot(grid[1, :])
    times = np.linspace(0.0, scenario.horizon, 441)
    for index, candidate in enumerate(scenario.windows):
        areas = np.asarray(
            [candidate.state(float(time)).safe_polygon.area for time in times]
        )
        timeline.plot(times, areas, linewidth=1.8, label=candidate.name)
        starts = [interval[0] for interval in candidate.planned_opportunities]
        timeline.scatter(
            starts,
            np.full(len(starts), -0.10 - 0.07 * index),
            marker="|",
            s=90,
            color=f"C{index}",
        )
    timeline.axhline(
        config.environment.minimum_safe_area,
        color="black",
        linestyle="--",
        linewidth=1.0,
        label="可通行阈值",
    )
    timeline.set_ylim(bottom=-0.42)
    timeline.set_xlabel("仿真开始时已固定的绝对时间 [s]")
    timeline.set_ylabel("安全穿越面积 [m²]")
    timeline.set_title(
        "相互独立的非周期日程：每扇窗的相位、宽度和重现间隔均不同"
    )
    timeline.grid(alpha=0.25)
    timeline.legend(ncol=5, fontsize=8)
    timeline.text(
        0.99,
        0.96,
        "竖线＝独立采样的完全开放起点\n曲线提前上升是因为窗口连续打开\n生成器不读取任何无人机状态",
        transform=timeline.transAxes,
        ha="right",
        va="top",
        fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.88, "edgecolor": "#b0bec5"},
    )
    figure.suptitle(
        "时变窗口建模：中心、姿态、形状与开放日程均为独立外生过程"
        f" · 随机种子 {seed}",
        fontsize=15,
        weight="bold",
    )
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=190, bbox_inches="tight")
    plt.close(figure)
    return path


def plot_experiment_protocol(output: str | Path) -> Path:
    figure, axis = plt.subplots(figsize=(15.0, 7.0))
    axis.set_xlim(0.0, 1.0)
    axis.set_ylim(0.0, 1.0)
    axis.axis("off")
    axis.set_title("预注册学术实验方案", fontsize=16, weight="bold")

    boxes = (
        ((0.03, 0.62), 0.16, "训练库\n5 个独立训练种子\n每种子 204.8 万步", "#e3f2fd"),
        ((0.23, 0.62), 0.17, "方法\nFAPP-PPO\n2 个名义基线\n6 项消融", "#e8f5e9"),
        ((0.44, 0.62), 0.18, "配对场景库\n200 个未见种子\n所有方法使用同一窗口", "#fff3e0"),
        ((0.66, 0.62), 0.14, "9 个条件\n同分布 + 开放宽度\n运动/形变/抖动分布外", "#f3e5f5"),
        ((0.84, 0.62), 0.13, "几何审计\n物理形状有效\n同时存在开/闭状态", "#ffebee"),
    )
    for (xy, width, text, color) in boxes:
        _box(axis, xy, width, 0.20, text, color=color, fontsize=9)
    for left, right in zip(boxes[:-1], boxes[1:]):
        _arrow(
            axis,
            (left[0][0] + left[1], 0.72),
            (right[0][0], 0.72),
        )

    _box(
        axis, (0.18, 0.25), 0.24, 0.20,
        "主要指标\n闭环成功率\n四窗按序穿越 + 全状态返回",
        color="#e0f2f1",
    )
    _box(
        axis, (0.47, 0.25), 0.20, 0.20,
        "次要指标\n时间裕度、错失、能耗\n饱和率、失败类型",
        color="#fce4ec",
    )
    _box(
        axis, (0.72, 0.25), 0.23, 0.20,
        "统计方法\nWilson 95% 置信区间\n配对精确 McNemar + Holm\n配对自助法时间差区间",
        color="#ede7f6",
    )
    _arrow(axis, (0.54, 0.62), (0.30, 0.45))
    _arrow(axis, (0.62, 0.62), (0.57, 0.45))
    _arrow(axis, (0.67, 0.35), (0.72, 0.35))
    axis.text(
        0.5,
        0.08,
        "主结论必须通过校正后的显著性检验，且至少 4/5 个训练种子的效果方向一致。",
        ha="center",
        fontsize=11,
        color="#37474f",
    )

    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=190, bbox_inches="tight")
    plt.close(figure)
    return path


def plot_pilot_results(summary_csv: str | Path, output: str | Path) -> Path:
    rows = _read_csv(summary_csv)
    conditions = list(dict.fromkeys(row["condition"] for row in rows))
    methods = list(dict.fromkeys(row["method"] for row in rows))
    condition_labels = {
        "id": "同分布",
        "tight": "窄机会",
        "wide_1p80": "宽机会 1.80 秒",
        "tight_1p10": "窄机会 1.10 秒",
        "tight_0p80": "窄机会 0.80 秒",
        "motion_1p20": "低运动幅度",
        "motion_2p30": "高运动幅度",
        "deform_2p60": "高形变幅度",
        "jitter_0p45": "高日程抖动",
        "single_shot_stress": "独立单次机会",
    }
    method_labels = {
        "FAPP-PPO": "FAPP-PPO",
        "Nominal-Reactive": "名义控制（反应式）",
        "Nominal-Schedule": "名义控制（日程感知）",
    }
    lookup = {(row["condition"], row["method"]): row for row in rows}
    metrics: tuple[tuple[str, str, float], ...] = (
        ("success_rate", "闭环成功率", 1.0),
        ("mean_crossings", "平均合法穿越数 ÷ 4", 4.0),
        ("mean_missed_opportunities", "平均错失机会数", 1.0),
    )
    figure, axes = plt.subplots(1, 3, figsize=(14.5, 4.8))
    x = np.arange(len(conditions), dtype=float)
    width = 0.78 / max(len(methods), 1)
    palette = [COLORS["blue"], COLORS["orange"], COLORS["teal"], COLORS["purple"]]
    for axis, (key, label, normalization) in zip(axes, metrics):
        for method_index, method in enumerate(methods):
            values = []
            for condition in conditions:
                row = lookup.get((condition, method))
                values.append(
                    float(row[key]) / normalization if row is not None else np.nan
                )
            offset = (method_index - 0.5 * (len(methods) - 1)) * width
            bars = axis.bar(
                x + offset,
                values,
                width=width,
                label=method_labels.get(method, method),
                color=palette[method_index % len(palette)],
                alpha=0.88,
            )
            axis.bar_label(bars, fmt="%.2f", fontsize=7, padding=2)
        axis.set_xticks(
            x,
            [condition_labels.get(condition, condition) for condition in conditions],
            rotation=12,
        )
        axis.set_ylabel(label)
        axis.grid(axis="y", alpha=0.25)
    axes[0].set_ylim(0.0, 1.05)
    axes[1].set_ylim(0.0, 1.05)
    axes[0].legend(fontsize=8)
    figure.suptitle(
        "先导评估（仅作描述，不是五训练种子的论文正式结果）",
        fontsize=15,
        weight="bold",
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=190, bbox_inches="tight")
    plt.close(figure)
    return path


def generate_figure_set(
    *,
    config: str | Path,
    metrics: str | Path,
    academic_dir: str | Path,
    outdir: str | Path,
    seed: int,
) -> list[Path]:
    output = Path(outdir)
    output.mkdir(parents=True, exist_ok=True)
    academic = Path(academic_dir)
    return [
        plot_training_curves(metrics, output / "figure_1_training_reward.png"),
        plot_algorithm_overview(output / "figure_2_algorithm_overview.png"),
        plot_window_modeling(
            config, output / "figure_3_independent_window_model.png", seed=seed
        ),
        plot_experiment_protocol(output / "figure_4_experiment_protocol.png"),
        plot_pilot_results(
            academic / "summary.csv", output / "figure_5_pilot_results.png"
        ),
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--metrics", required=True)
    parser.add_argument("--academic-dir", required=True)
    parser.add_argument("--outdir", required=True)
    parser.add_argument("--seed", type=int, default=50_000)
    args = parser.parse_args()
    paths = generate_figure_set(
        config=args.config,
        metrics=args.metrics,
        academic_dir=args.academic_dir,
        outdir=args.outdir,
        seed=args.seed,
    )
    for path in paths:
        print(path)


if __name__ == "__main__":
    main()
