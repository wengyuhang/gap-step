"""Generate plain-language Chinese figures for SC-DynaTOGT.

The scientific visualizations in :mod:`visualization` are intended for
measurement.  This module adds a second, presentation-oriented layer that
explains what the pipeline, source modules, gradients, and E0--E5 outputs
mean without changing or recomputing the algorithm.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Arc, Circle, FancyArrowPatch, FancyBboxPatch, Polygon  # noqa: E402
import numpy as np  # noqa: E402


COLORS = {
    "ink": "#183153",
    "muted": "#60758a",
    "blue": "#2f6fed",
    "cyan": "#34a7c1",
    "green": "#2a9d6f",
    "orange": "#ef8a3a",
    "red": "#d9534f",
    "purple": "#7656a5",
    "paper": "#f7f9fc",
    "line": "#d8e0ea",
    "warning": "#fff4df",
}


def _configure_chinese_font() -> None:
    """Select an available CJK font while retaining portable fallbacks."""

    plt.rcParams.update(
        {
            "font.sans-serif": [
                "Microsoft YaHei",
                "Noto Sans CJK SC",
                "Noto Sans CJK JP",
                "Droid Sans Fallback",
                "DejaVu Sans",
            ],
            "axes.unicode_minus": False,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "text.color": COLORS["ink"],
            "axes.labelcolor": COLORS["ink"],
            "axes.titlecolor": COLORS["ink"],
        }
    )


def _save(figure: plt.Figure, output_path: str | Path, *, dpi: int = 160) -> Path:
    path = Path(output_path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=dpi, bbox_inches="tight", facecolor="white")
    plt.close(figure)
    return path


def _box(
    axis: plt.Axes,
    x: float,
    y: float,
    width: float,
    height: float,
    title: str,
    detail: str,
    *,
    color: str,
    title_size: float = 14,
    detail_size: float = 10.5,
) -> None:
    patch = FancyBboxPatch(
        (x, y),
        width,
        height,
        boxstyle="round,pad=0.012,rounding_size=0.018",
        linewidth=1.7,
        edgecolor=color,
        facecolor="white",
        zorder=2,
    )
    axis.add_patch(patch)
    axis.add_patch(
        FancyBboxPatch(
            (x, y + height - 0.036),
            width,
            0.036,
            boxstyle="round,pad=0.0,rounding_size=0.012",
            linewidth=0,
            facecolor=color,
            zorder=3,
        )
    )
    axis.text(
        x + width / 2,
        y + height * 0.64,
        title,
        ha="center",
        va="center",
        fontsize=title_size,
        fontweight="bold",
        color=COLORS["ink"],
        zorder=4,
    )
    axis.text(
        x + width / 2,
        y + height * 0.29,
        detail,
        ha="center",
        va="center",
        fontsize=detail_size,
        color=COLORS["muted"],
        linespacing=1.35,
        zorder=4,
    )


def _arrow(
    axis: plt.Axes,
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    color: str = COLORS["muted"],
    connectionstyle: str = "arc3",
    width: float = 1.8,
) -> None:
    axis.add_patch(
        FancyArrowPatch(
            start,
            end,
            arrowstyle="-|>",
            mutation_scale=14,
            linewidth=width,
            color=color,
            connectionstyle=connectionstyle,
            zorder=5,
        )
    )


def plot_algorithm_overview(output_path: str | Path) -> Path:
    """Draw the complete method as one offline/online story."""

    _configure_chinese_font()
    figure, axis = plt.subplots(figsize=(16, 8.5))
    axis.set_xlim(0.0, 1.0)
    axis.set_ylim(0.0, 1.0)
    axis.axis("off")
    axis.text(
        0.5,
        0.955,
        "SC-DynaTOGT 一张图：先把窗口做成“可安全取点”的地图，再优化何时从哪里穿过",
        ha="center",
        va="center",
        fontsize=22,
        fontweight="bold",
    )

    axis.add_patch(
        FancyBboxPatch(
            (0.025, 0.47), 0.95, 0.37,
            boxstyle="round,pad=0.012,rounding_size=0.025",
            facecolor="#eef5ff", edgecolor="#b8d0f5", linewidth=1.2,
        )
    )
    axis.add_patch(
        FancyBboxPatch(
            (0.025, 0.09), 0.95, 0.30,
            boxstyle="round,pad=0.012,rounding_size=0.025",
            facecolor="#effaf5", edgecolor="#b7e0cd", linewidth=1.2,
        )
    )
    axis.text(0.045, 0.805, "离线：每种窗口几何只做一次", fontsize=14, fontweight="bold", color=COLORS["blue"])
    axis.text(0.045, 0.355, "在线：每次轨迹优化反复计算", fontsize=14, fontweight="bold", color=COLORS["green"])

    offline = [
        (0.055, "1  原始边界", "折线 / 光滑曲线 /\n直线–曲线混合", COLORS["purple"]),
        (0.285, "2  Chang 采样", "沿边界均匀排点\n真角点原样保留", COLORS["cyan"]),
        (0.515, "3  安全内缩", "Clipper2 向内 0.315 m\n得到真安全多边形", COLORS["orange"]),
        (0.745, "4  SC 地图", "圆盘 → 非凸内部\n保存为 sc_map.npz", COLORS["blue"]),
    ]
    for x, title, detail, color in offline:
        _box(axis, x, 0.535, 0.185, 0.20, title, detail, color=color)
    for left, right in zip(offline, offline[1:]):
        _arrow(axis, (left[0] + 0.187, 0.635), (right[0] - 0.007, 0.635), color=COLORS["blue"])

    online = [
        (0.055, "5  两类变量", "D：窗口内的位置\nK：每段飞行时间", COLORS["purple"]),
        (0.285, "6  动态窗口", "平移 + 旋转 + 缩放\n得到三维穿越点", COLORS["cyan"]),
        (0.515, "7  MINCO + 动力学", "用 7 次多项式连成轨迹\n计算时间和软约束代价", COLORS["orange"]),
        (0.745, "8  L-BFGS 反复改进", "梯度返回 K,D\n直到代价不再明显下降", COLORS["green"]),
    ]
    for x, title, detail, color in online:
        _box(axis, x, 0.145, 0.185, 0.16, title, detail, color=color, title_size=13.5, detail_size=10)
    for left, right in zip(online, online[1:]):
        _arrow(axis, (left[0] + 0.187, 0.225), (right[0] - 0.007, 0.225), color=COLORS["green"])
    _arrow(axis, (0.838, 0.535), (0.37, 0.305), color=COLORS["blue"], connectionstyle="arc3,rad=0.18")
    axis.text(0.665, 0.405, "载入离线 SC 地图", fontsize=11, color=COLORS["blue"], rotation=-15)
    _arrow(axis, (0.838, 0.145), (0.838, 0.115), color=COLORS["green"])
    axis.text(
        0.5,
        0.035,
        "最终输出：平滑三维轨迹 + 每个窗口的穿越时刻 + 真安全区域内的穿越点",
        ha="center", va="center", fontsize=14, fontweight="bold", color=COLORS["ink"],
    )
    return _save(figure, output_path)


def plot_component_map(output_path: str | Path) -> Path:
    """Explain which source file owns each responsibility."""

    _configure_chinese_font()
    figure, axis = plt.subplots(figsize=(16, 9.2))
    axis.set_xlim(0.0, 1.0)
    axis.set_ylim(0.0, 1.0)
    axis.axis("off")
    axis.text(0.5, 0.96, "组件地图：每个 Python 文件管什么", ha="center", fontsize=22, fontweight="bold")

    rows = [
        (
            0.73,
            "几何预处理",
            COLORS["blue"],
            [
                ("boundary.py", "边界读入、加密\nChang 采样与角点"),
                ("offset.py", "Clipper2 安全内缩\n连通/无孔检查"),
                ("sc_mapping.py", "求预顶点和 SC 常数\n值、雅可比、逆映射"),
                ("preprocessing.py", "串起整条离线流程\n保存/加载 artifact"),
            ],
        ),
        (
            0.43,
            "轨迹与优化",
            COLORS["green"],
            [
                ("environment.py", "窗口位置、旋转、缩放\n空间/时间梯度"),
                ("time_mapping.py", "无约束 K 与正时间 T 互换\n穿越时刻前缀和"),
                ("minco.py", "degree-7 MINCO\n把穿越点连成平滑轨迹"),
                ("dynamics.py", "四旋翼平坦性\n速度/角速度/推力软惩罚"),
            ],
        ),
        (
            0.13,
            "验证与输出",
            COLORS["purple"],
            [
                ("optimizer.py", "组装 x=[K,D]\nL-BFGS 和完整链式梯度"),
                ("experiments.py", "E0–E5 协议\n统计、JSON、CSV"),
                ("validation.py", "百万点映射合法性\n中心差分梯度检查"),
                ("visualization.py", "预处理图、轨迹图\nCSV 和动画 GIF"),
            ],
        ),
    ]
    for y, label, color, modules in rows:
        axis.text(0.035, y + 0.09, label, fontsize=14, fontweight="bold", color=color, rotation=90, va="center")
        for index, (name, detail) in enumerate(modules):
            x = 0.095 + index * 0.225
            _box(axis, x, y, 0.19, 0.20, name, detail, color=color, title_size=13, detail_size=10)
            if index < len(modules) - 1:
                _arrow(axis, (x + 0.192, y + 0.10), (x + 0.222, y + 0.10), color=color)

    axis.plot([0.84, 0.84, 0.19], [0.73, 0.685, 0.685], color=COLORS["blue"], linewidth=2.0)
    _arrow(axis, (0.19, 0.685), (0.19, 0.635), color=COLORS["blue"])
    axis.text(0.52, 0.665, "artifact 为在线优化提供安全区域地图", fontsize=11, color=COLORS["blue"], ha="center")
    axis.plot([0.84, 0.84, 0.19], [0.43, 0.385, 0.385], color=COLORS["green"], linewidth=2.0)
    _arrow(axis, (0.19, 0.385), (0.19, 0.335), color=COLORS["green"])
    axis.text(0.53, 0.365, "优化器调用轨迹与动力学，再把梯度传回 K,D", fontsize=11, color=COLORS["green"], ha="center")
    axis.text(
        0.5, 0.04,
        "看代码的快速路线：几何问题看第一行；轨迹/梯度问题看第二行；结果是否可信看第三行。",
        ha="center", fontsize=13, fontweight="bold",
    )
    return _save(figure, output_path)


def _l_shape() -> np.ndarray:
    return np.asarray([[-1.0, -0.9], [1.0, -0.9], [1.0, -0.2], [0.2, -0.2], [0.2, 0.9], [-1.0, 0.9]])


def plot_dynamic_gradient_story(output_path: str | Path) -> Path:
    """Show the geometric maps and the two gradient return paths."""

    _configure_chinese_font()
    figure = plt.figure(figsize=(16, 9.2))
    grid = figure.add_gridspec(2, 2, height_ratios=(1.05, 0.95), hspace=0.23, wspace=0.18)
    ax_map = figure.add_subplot(grid[0, 0])
    ax_motion = figure.add_subplot(grid[0, 1])
    ax_grad = figure.add_subplot(grid[1, :])
    figure.suptitle("两个关键问题：穿越点怎么永远在窗口里？窗口在动时梯度怎么传？", fontsize=21, fontweight="bold")

    ax_map.set_title("A. 只用 2 个无约束数，得到非凸窗口内部点", fontsize=15, fontweight="bold")
    ax_map.set_xlim(-3.2, 3.4)
    ax_map.set_ylim(-1.7, 1.7)
    ax_map.axis("off")
    ax_map.add_patch(Circle((-2.4, 0), 0.07, color=COLORS["purple"]))
    ax_map.text(-2.4, -0.35, "d ∈ R²\n可以任意大", ha="center", fontsize=11)
    ax_map.add_patch(Circle((-0.65, 0), 0.85, fill=False, linewidth=2, edgecolor=COLORS["blue"]))
    ax_map.add_patch(Circle((-0.35, 0.25), 0.06, color=COLORS["purple"]))
    ax_map.text(-0.65, -1.15, "B(d) 在开单位圆盘内", ha="center", fontsize=11)
    shape = _l_shape() * 0.75 + np.array([2.25, 0.0])
    ax_map.add_patch(Polygon(shape, closed=True, facecolor="#e9e4f5", edgecolor=COLORS["purple"], linewidth=2))
    ax_map.add_patch(Circle((1.75, -0.15), 0.06, color=COLORS["orange"]))
    ax_map.text(2.25, -1.15, "q = SC(B(d))\n必然在真非凸区域内", ha="center", fontsize=11)
    _arrow(ax_map, (-2.15, 0.0), (-1.55, 0.0), color=COLORS["blue"])
    _arrow(ax_map, (0.35, 0.0), (1.15, 0.0), color=COLORS["blue"])
    ax_map.text(-1.85, 0.18, "压入圆盘", ha="center", color=COLORS["blue"], fontsize=10)
    ax_map.text(0.75, 0.18, "SC 映射", ha="center", color=COLORS["blue"], fontsize=10)

    ax_motion.set_title("B. 同一个局部点 q，会跟着窗口平移、旋转和缩放", fontsize=15, fontweight="bold")
    ax_motion.set_xlim(-2.2, 3.5)
    ax_motion.set_ylim(-1.8, 2.2)
    ax_motion.set_aspect("equal")
    ax_motion.axis("off")
    base = _l_shape() * 0.85 + np.array([-0.9, 0.0])
    angle = np.deg2rad(24.0)
    rotation = np.asarray([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    moved = (_l_shape() * 1.12) @ rotation.T + np.array([2.0, 0.55])
    ax_motion.add_patch(Polygon(base, closed=True, fill=False, linestyle="--", edgecolor=COLORS["muted"], linewidth=1.7))
    ax_motion.add_patch(Polygon(moved, closed=True, facecolor="#dff4eb", edgecolor=COLORS["green"], linewidth=2.2, alpha=0.9))
    q0 = np.array([-1.25, -0.2])
    q1 = (np.array([-0.4, -0.2]) * 1.12) @ rotation.T + np.array([2.0, 0.55])
    ax_motion.scatter(*q0, color=COLORS["orange"], s=48, zorder=5)
    ax_motion.scatter(*q1, color=COLORS["orange"], s=48, zorder=5)
    _arrow(ax_motion, (-0.1, 0.05), (1.0, 0.4), color=COLORS["green"])
    ax_motion.add_patch(Arc((1.45, 1.35), 1.0, 0.75, theta1=10, theta2=125, color=COLORS["purple"], linewidth=2))
    _arrow(ax_motion, (1.45, 1.72), (1.16, 1.62), color=COLORS["purple"])
    ax_motion.text(0.45, -0.35, "平移 c(t)", fontsize=11, color=COLORS["green"])
    ax_motion.text(1.7, 1.75, "旋转 E(t)", fontsize=11, color=COLORS["purple"])
    ax_motion.text(2.35, -0.85, "缩放 s(t)", fontsize=11, color=COLORS["blue"])
    ax_motion.text(0.6, -1.45, "p(d,t) = c(t) + E(t) s(t) q(d)", fontsize=14, fontweight="bold", ha="center")

    ax_grad.set_xlim(0.0, 1.0)
    ax_grad.set_ylim(0.0, 1.0)
    ax_grad.axis("off")
    ax_grad.set_title("C. 前向算轨迹，反向把代价分成“改位置”和“改时间”两条路", fontsize=15, fontweight="bold")
    chain = [
        (0.03, "K, D", "时间 + 局部位置"),
        (0.22, "T, t", "段时间 + 穿越时刻"),
        (0.41, "p(d,t)", "三维穿越点"),
        (0.60, "MINCO", "平滑轨迹"),
        (0.79, "J", "时间 + 动力学代价"),
    ]
    for x, title, detail in chain:
        _box(ax_grad, x, 0.57, 0.15, 0.19, title, detail, color=COLORS["ink"], title_size=13, detail_size=9.5)
    for left, right in zip(chain, chain[1:]):
        _arrow(ax_grad, (left[0] + 0.152, 0.665), (right[0] - 0.005, 0.665), color=COLORS["ink"])
    ax_grad.text(0.50, 0.83, "前向：用当前 K,D 算一条轨迹和它的代价", ha="center", fontsize=12, fontweight="bold")
    _arrow(ax_grad, (0.86, 0.53), (0.48, 0.36), color=COLORS["blue"], connectionstyle="arc3,rad=-0.12", width=2.4)
    _arrow(ax_grad, (0.48, 0.34), (0.12, 0.23), color=COLORS["blue"], connectionstyle="arc3,rad=-0.08", width=2.4)
    ax_grad.text(0.64, 0.32, "空间梯度：grad p → J_SC J_B → grad D", fontsize=11.5, color=COLORS["blue"], fontweight="bold")
    _arrow(ax_grad, (0.86, 0.51), (0.48, 0.15), color=COLORS["orange"], connectionstyle="arc3,rad=0.10", width=2.4)
    _arrow(ax_grad, (0.48, 0.14), (0.12, 0.12), color=COLORS["orange"], connectionstyle="arc3,rad=0.03", width=2.4)
    ax_grad.text(0.56, 0.075, "时间梯度：grad p → c/E/s 的时间导数 → grad t → grad T → grad K", fontsize=11.5, color=COLORS["orange"], fontweight="bold")
    return _save(figure, output_path)


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def plot_reading_guide(results_root: str | Path, output_path: str | Path) -> Path:
    """Annotate the two main scientific figures already produced by E4."""

    _configure_chinese_font()
    root = Path(results_root)
    preprocessing_path = root / "E4" / "preprocessed_gates" / "00_L" / "preprocessing.png"
    trajectory_path = root / "E4" / "trajectory.png"
    if not preprocessing_path.is_file() or not trajectory_path.is_file():
        raise FileNotFoundError(
            "E4 preprocessing/trajectory PNG is missing; run an experiment suite first"
        )
    preprocessing = plt.imread(preprocessing_path)
    trajectory = plt.imread(trajectory_path)
    figure = plt.figure(figsize=(16, 12))
    grid = figure.add_gridspec(2, 2, height_ratios=(1.0, 0.82), width_ratios=(1.15, 0.85), hspace=0.14, wspace=0.06)
    top = figure.add_subplot(grid[0, :])
    left = figure.add_subplot(grid[1, 0])
    right = figure.add_subplot(grid[1, 1])
    figure.suptitle("现有输出图怎么看：先确认“点在哪里取”，再看“飞行路径如何穿过”", fontsize=21, fontweight="bold")
    top.imshow(preprocessing)
    top.axis("off")
    top.set_title("预处理图：左=边界与安全区，中=圆盘坐标，右=SC 把网格弯成非凸形状", fontsize=14, fontweight="bold")
    left.imshow(trajectory)
    left.axis("off")
    left.set_title("轨迹图：原始边界和安全区都是“穿越时刻”的位姿", fontsize=14, fontweight="bold")
    right.axis("off")
    right.add_patch(FancyBboxPatch((0.02, 0.04), 0.96, 0.92, boxstyle="round,pad=0.02", facecolor=COLORS["paper"], edgecolor=COLORS["line"]))
    notes = [
        ("① 灰线 / 蓝点", "灰线是稠密真边界；蓝点是 Chang 均匀采样后的多边形。"),
        ("② 橙色内轮廓", "向内缩 0.315 m 后的安全区。优化点使用这个区域，不使用原边界。"),
        ("③ 圆盘网格 → 弯曲网格", "每条蓝线的对应关系由 SC 映射给出；网格不穿出黑色安全边界。"),
        ("④ 蓝色三维线", "MINCO 生成的平滑轨迹；绿点是起点，红叉是终点。"),
        ("⑤ 黑色虚线 / 彩色安全区", "虚线是原始物理窗口；彩色半透明区是内缩 0.315 m 后的优化可用区。"),
        ("⑥ 橙色菱形中心", "规定穿越时刻的轨迹点；判定的是标记中心，它必须在当时安全区内。"),
    ]
    y = 0.91
    for title, detail in notes:
        right.text(0.07, y, title, fontsize=12.5, fontweight="bold", color=COLORS["blue"], va="top")
        right.text(0.07, y - 0.052, detail, fontsize=10.5, color=COLORS["ink"], va="top", wrap=True, linespacing=1.3)
        y -= 0.145
    return _save(figure, output_path)


def _e1_max_errors(rows: list[dict[str, Any]]) -> tuple[list[str], np.ndarray]:
    names = {
        "l_shape": "L 形",
        "u_shape": "U 形",
        "five_point_star": "五角星",
        "limacon": "limaçon",
        "wavy": "波浪",
        "line_bezier_mixed": "直线–Bézier",
    }
    ordered = list(names)
    values = [
        max(float(row["max_boundary_error_m"]) for row in rows if row["boundary"] == key) * 1000.0
        for key in ordered
    ]
    return [names[key] for key in ordered], np.asarray(values)


def plot_experiment_results(results_root: str | Path, output_path: str | Path) -> Path:
    """Turn smoke or default JSON into a plain-language E0--E5 dashboard."""

    _configure_chinese_font()
    root = Path(results_root)
    groups = {name: _load_json(root / name / "summary.json") for name in ("E0", "E1", "E2", "E3", "E4", "E5")}
    suite_summary = _load_json(root / "summary.json") if (root / "summary.json").is_file() else {}
    suite = str(suite_summary.get("settings", {}).get("suite", "smoke"))
    suite_label = "default 完整实验" if suite == "default" else "smoke 烟测"
    figure, axes = plt.subplots(2, 3, figsize=(17, 10.5), constrained_layout=True)
    figure.suptitle(f"E0–E5 到底在验证什么？（{suite_label}）", fontsize=22, fontweight="bold")

    ax = axes[0, 0]
    e0_percent = 100.0 * float(groups["E0"]["relative_total_time_error"])
    ax.barh(["SC 替换后"], [max(e0_percent, 1e-14)], color=COLORS["green"], height=0.42)
    ax.axvline(1.0, color=COLORS["red"], linestyle="--", linewidth=2, label="允许上限 1%")
    ax.set_xscale("log")
    ax.set_xlim(1e-12, 2.0)
    ax.set_xlabel("总时间相对差 [%]（越小越好）")
    ax.set_title("E0：换成 SC 后，旧 TOGT 结果有没有被破坏？", fontweight="bold")
    ax.text(max(e0_percent, 1e-14), 0, f"  {e0_percent:.2e}%", va="center", fontsize=10)
    ax.legend(loc="lower right", fontsize=9)

    ax = axes[0, 1]
    labels, errors = _e1_max_errors(groups["E1"]["rows"])
    colors = [COLORS["green"] if value <= 5.0 else COLORS["red"] for value in errors]
    bars = ax.barh(labels[::-1], errors[::-1], color=colors[::-1])
    ax.axvline(5.0, color=COLORS["red"], linestyle="--", linewidth=2, label="5 mm 上限")
    ax.set_xlabel("最大边界误差 [mm]")
    ax.set_title("E1：Chang 采样后的多边形够不够准？", fontweight="bold")
    ax.legend(fontsize=9)
    for bar, value in zip(bars, errors[::-1]):
        ax.text(max(value, 0.03) + 0.06, bar.get_y() + bar.get_height() / 2, f"{value:.2f}", va="center", fontsize=9)

    ax = axes[0, 2]
    e2_rows = groups["E2"]["rows"]
    method_names = {"fixed_center": "固定中心", "convex_hull": "凸包", "sc": "SC 非凸"}
    times = [
        float(np.mean([row["total_time"] for row in e2_rows if row["method"] == method]))
        for method in method_names
    ]
    ax.bar(list(method_names.values()), times, color=[COLORS["muted"], COLORS["orange"], COLORS["blue"]])
    ax.set_ylabel("总飞行时间 [s]")
    ax.set_title("E2：静态非凸窗口的三种取点方法", fontweight="bold")
    ax.tick_params(axis="x", rotation=12)
    e2_repetitions = sum(row["method"] == "sc" for row in e2_rows)
    note = (
        "smoke 仅 1 个种子，只看调用链，不下统计结论"
        if suite == "smoke"
        else f"default：{e2_repetitions} 个种子的平均总时间"
    )
    ax.text(0.02, 0.96, note, transform=ax.transAxes, va="top", fontsize=9, color=COLORS["muted"])

    ax = axes[1, 0]
    labels_rate = ["E2 SC", "E3 平移", "E4 全动态", "E5 完整梯度", "E5 去掉时间梯度"]
    convergence = [
        float(groups["E2"]["sc_convergence_rate"]),
        float(groups["E3"]["convergence_rate"]),
        float(groups["E4"]["convergence_rate"]),
        float(groups["E5"]["method_rates"]["full_time_gradient"]["convergence_rate"]),
        float(groups["E5"]["method_rates"]["zero_window_time_gradient"]["convergence_rate"]),
    ]
    legal = [
        float(groups["E2"]["sc_legal_rate"]),
        float(groups["E3"]["designated_order_legal_rate"]),
        float(groups["E4"]["designated_order_legal_rate"]),
        float(groups["E5"]["method_rates"]["full_time_gradient"]["designated_order_legal_rate"]),
        float(groups["E5"]["method_rates"]["zero_window_time_gradient"]["designated_order_legal_rate"]),
    ]
    x = np.arange(len(labels_rate))
    ax.bar(x - 0.18, convergence, 0.36, label="优化收敛", color=COLORS["blue"])
    ax.bar(x + 0.18, legal, 0.36, label="指定穿越点合法", color=COLORS["green"])
    ax.set_ylim(0.0, 1.12)
    ax.set_xticks(x, labels_rate, rotation=16, ha="right")
    ax.set_ylabel("比率")
    ax.set_title("E2–E5：能否收敛，且穿越点是否真在窗口里？", fontweight="bold")
    ax.legend(fontsize=9, loc="lower left")

    ax = axes[1, 1]
    gradient_labels = ["E3 窗口", "E3 联合", "E4 窗口", "E4 联合"]
    gradient_values = [
        max(float(report["p99_relative_error"]) for report in groups["E3"]["gradient_reports"]),
        float(groups["E3"]["joint_objective_gradient_report"]["p99_relative_error"]),
        max(float(report["p99_relative_error"]) for report in groups["E4"]["gradient_reports"]),
        float(groups["E4"]["joint_objective_gradient_report"]["p99_relative_error"]),
    ]
    ax.barh(gradient_labels[::-1], gradient_values[::-1], color=[COLORS["green"]] * 4)
    ax.axvline(1e-3, color=COLORS["red"], linestyle="--", linewidth=2, label="p99 上限 1e-3")
    ax.set_xscale("log")
    ax.set_xlabel("99% 分位相对误差（越小越好）")
    ax.set_title("E3/E4：解析梯度和中心差分是否一致？", fontweight="bold")
    ax.legend(fontsize=9)

    ax = axes[1, 2]
    ax.axis("off")
    validation_path = root / "E4" / "preprocessed_gates" / "00_L" / "validation_1m.json"
    if validation_path.is_file():
        mapping = _load_json(validation_path)
    else:
        candidate = groups["E4"]["mapping_validation"]
        mapping = candidate[0] if isinstance(candidate, list) else candidate
    max_rotor_e3 = max(float(row["sampled_max_rotor_thrust"]) for row in groups["E3"]["rows"])
    max_rotor_e4 = max(float(row["sampled_max_rotor_thrust"]) for row in groups["E4"]["rows"])
    ax.add_patch(FancyBboxPatch((0.03, 0.53), 0.94, 0.42, boxstyle="round,pad=0.02", facecolor="#eefaf5", edgecolor="#b7e0cd"))
    ax.text(0.07, 0.88, "映射合法性", fontsize=14, fontweight="bold", color=COLORS["green"])
    ax.text(0.07, 0.77, f"{int(mapping['inside_count']):,} / {int(mapping['sample_count']):,} 点位于真安全多边形内", fontsize=12)
    ax.text(0.07, 0.67, f"outside / NaN / Inf / 退化雅可比 = {mapping['outside_count']} / {mapping['nan_count']} / {mapping['inf_count']} / {mapping['degenerate_jacobian_count']}", fontsize=10.5)
    ax.add_patch(FancyBboxPatch((0.03, 0.06), 0.94, 0.38, boxstyle="round,pad=0.02", facecolor=COLORS["warning"], edgecolor="#e5bd70"))
    ax.text(0.07, 0.37, "重要：“实验通过” ≠ “硬动力学约束证明”", fontsize=13.5, fontweight="bold", color=COLORS["red"])
    ax.text(0.07, 0.25, f"原 TOGT 使用软惩罚；5.0 N 单旋翼上限下，E3/E4 采样峰值为 {max_rotor_e3:.3f} / {max_rotor_e4:.3f} N。", fontsize=11)
    ax.text(0.07, 0.13, "所以 JSON 如实记录 sampled_dynamic_limits_satisfied=false；这是已知限制，不是隐藏的成功。", fontsize=10.5)
    ax.set_title("数值稳定性与结果边界", fontweight="bold")
    return _save(figure, output_path)


def generate_explanation_figures(
    results_root: str | Path,
    output_directory: str | Path,
) -> tuple[Path, ...]:
    """Generate the complete five-figure explanation set."""

    root = Path(output_directory).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    outputs = (
        plot_algorithm_overview(root / "01_algorithm_overview.png"),
        plot_component_map(root / "02_component_map.png"),
        plot_dynamic_gradient_story(root / "03_dynamic_and_gradients.png"),
        plot_reading_guide(results_root, root / "04_how_to_read_outputs.png"),
        plot_experiment_results(results_root, root / "05_experiment_results.png"),
    )
    return outputs


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate plain-language SC-DynaTOGT figures")
    parser.add_argument(
        "--results",
        type=Path,
        default=Path("nonconvex_timevarying_window/sc_dynatogt/results/smoke"),
        help="completed smoke result directory",
    )
    parser.add_argument(
        "--outdir",
        type=Path,
        default=Path("nonconvex_timevarying_window/sc_dynatogt/algorithm_figures"),
    )
    args = parser.parse_args(argv)
    outputs = generate_explanation_figures(args.results, args.outdir)
    print(json.dumps({"figures": [str(path) for path in outputs]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "generate_explanation_figures",
    "main",
    "plot_algorithm_overview",
    "plot_component_map",
    "plot_dynamic_gradient_story",
    "plot_experiment_results",
    "plot_reading_guide",
]
