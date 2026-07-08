from __future__ import annotations

import csv
from pathlib import Path

import imageio.v2 as imageio
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

from .environment import WindowTrack
from .optimizer import DynaTOGTPlan

COLORS = ["#e41a1c", "#0b8f28", "#c98900", "#6a1fb0", "#1267d8", "#cc0b72", "#444444"]


def _configure_chinese_font() -> None:
    """为 Matplotlib 配置可用的中文字体。

    导出的 PNG/GIF 需要显示“穿越成功”等中文提示；函数会按常见字体名查找，
    找到后设置为 sans-serif 首选字体，并修正负号显示。
    """
    preferred = ["Microsoft YaHei", "Droid Sans Fallback", "Noto Sans CJK SC", "SimHei", "WenQuanYi Micro Hei"]
    available = {font.name for font in font_manager.fontManager.ttflist}
    for name in preferred:
        if name in available:
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            return


_configure_chinese_font()


def export_plan_csv(plan: DynaTOGTPlan, track: WindowTrack, path: Path) -> None:
    """把计划导出为包含穿越证据和轨迹采样的 CSV。

    `crossing` 行记录每个窗口穿越时刻的局部坐标、平面误差、窗口裕度和 contains 结果；
    `sample` 行记录密集轨迹采样，用于后续分析速度、加速度和 yaw。
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["section", "index", "name", "t", "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az", "yaw", "local_u", "local_v", "plane_error", "gate_margin", "contains"]
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(fields)
        for i, (idx, t, p) in enumerate(zip(plan.order, plan.crossing_times, plan.crossing_points), start=1):
            local, plane_error = track.windows[idx].point_to_local(p, float(t), dynamic=True)
            margin = track.windows[idx].local_margin(local, float(t), dynamic=True)
            contains = track.windows[idx].contains(p, float(t), dynamic=True)
            writer.writerow(["crossing", i, track.windows[idx].name, f"{t:.6f}", f"{p[0]:.6f}", f"{p[1]:.6f}", f"{p[2]:.6f}", "", "", "", "", "", "", "", f"{local[0]:.6f}", f"{local[1]:.6f}", f"{plane_error:.9f}", f"{margin:.9f}", contains])
        traj = plan.trajectory
        for i, (t, p, v, a, yaw) in enumerate(zip(traj.times, traj.positions, traj.velocities, traj.accelerations, traj.yaws)):
            writer.writerow(["sample", i, "drone", f"{t:.6f}", f"{p[0]:.6f}", f"{p[1]:.6f}", f"{p[2]:.6f}", f"{v[0]:.6f}", f"{v[1]:.6f}", f"{v[2]:.6f}", f"{a[0]:.6f}", f"{a[1]:.6f}", f"{a[2]:.6f}", f"{yaw:.6f}", "", "", "", "", ""])


def draw_plan_png(track: WindowTrack, plan: DynaTOGTPlan, path: Path, title: str | None = None) -> None:
    """绘制一张中文组会展示风格的静态结果图。

    图中包含完整轨迹、起终点、每个穿越时刻的窗口姿态、过去/未来虚线姿态以及侧边说明。
    该图用于快速展示 DynaTOGT 如何利用动态窗口几何完成穿越。
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(15.5, 9.2), facecolor="white")
    ax = fig.add_axes([0.04, 0.11, 0.72, 0.80])
    side = fig.add_axes([0.78, 0.16, 0.19, 0.72])
    _setup_schematic_axes(ax, track, plan)

    traj2 = _project(plan.trajectory.positions)
    ax.plot(traj2[:, 0], traj2[:, 1], color="#23272d", lw=4.2, alpha=0.16, solid_capstyle="round", zorder=1)
    ax.plot(traj2[:, 0], traj2[:, 1], color="#20242a", lw=2.25, solid_capstyle="round", zorder=2, label="无人机连续轨迹")

    start2 = _project(track.start[None, :])[0]
    goal2 = _project(track.goal[None, :])[0]
    ax.scatter([start2[0]], [start2[1]], color="#009e73", s=78, zorder=8, label="起点")
    ax.scatter([goal2[0]], [goal2[1]], color="#d55e00", s=78, zorder=8, label="终点")

    for rank, (idx, crossing_t, point) in enumerate(zip(plan.order, plan.crossing_times, plan.crossing_points), start=1):
        color = COLORS[idx % len(COLORS)]
        window = track.windows[idx]
        for offset in (-1.4, -0.7, 0.7, 1.4):
            _draw_window_2d(ax, window.polygon_at(max(0.0, float(crossing_t + offset)), dynamic=True), color=color, lw=1.25, alpha=0.30, linestyle="--", fill_alpha=0.0)
        _draw_window_2d(ax, window.polygon_at(float(crossing_t), dynamic=True), color=color, lw=3.0, alpha=1.0, linestyle="-", fill_alpha=0.060)
        point2 = _project(np.asarray(point)[None, :])[0]
        ax.scatter([point2[0]], [point2[1]], color="#00a651", s=84, edgecolors="black", linewidths=1.0, zorder=9)
        ax.text(point2[0], point2[1] - 0.40, f"t{rank}", color="black", fontsize=13, fontstyle="italic", ha="center", zorder=9)
        center2 = _project(window.center_at(float(crossing_t), dynamic=True)[None, :])[0]
        ax.text(center2[0], center2[1] + 0.72, window.name, color=color, fontsize=16, fontweight="bold", ha="center", zorder=9)
        ax.text(point2[0] + 0.30, point2[1] + 0.28, "穿越成功", color="#007a2f", fontsize=8.5, fontweight="bold", zorder=10)

    _draw_drone_2d(ax, _project(plan.trajectory.positions[-1:])[0], float(plan.trajectory.yaws[-1]))
    ax.set_title(title or "DynaTOGT 动态时变窗口穿越演示", fontsize=20, fontweight="bold", pad=18)
    ax.legend(loc="upper left", fontsize=9, frameon=True)
    _draw_side_panel(side, plan)
    fig.savefig(path, dpi=155)
    plt.close(fig)


def draw_plan_gif(track: WindowTrack, plan: DynaTOGTPlan, path: Path, frames: int = 32) -> None:
    """绘制动态 GIF，展示无人机轨迹和窗口随时间变化的过程。

    帧时间会合并均匀采样时刻和精确穿越时刻，保证 GIF 中一定出现每个“穿越成功”瞬间。
    临时帧写到隐藏目录，合成后立即清理。
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    frame_dir = path.parent / f".frames_{path.stem}"
    frame_dir.mkdir(parents=True, exist_ok=True)
    images = []
    traj = plan.trajectory
    base_times = np.linspace(0.0, plan.duration, frames)
    frame_times = np.unique(np.round(np.concatenate([base_times, plan.crossing_times]), decimals=6))
    for frame, t in enumerate(frame_times):
        upto = max(2, int(np.searchsorted(traj.times, t, side="right")))
        fig = plt.figure(figsize=(10.8, 7.0), facecolor="white")
        ax = fig.add_axes([0.05, 0.08, 0.90, 0.84])
        _setup_schematic_axes(ax, track, plan)
        full2 = _project(traj.positions)
        path2 = _project(traj.positions[:upto])
        ax.plot(full2[:, 0], full2[:, 1], color="#4b5159", lw=1.2, alpha=0.20, zorder=1)
        ax.plot(path2[:, 0], path2[:, 1], color="#20242a", lw=2.6, solid_capstyle="round", zorder=3)
        active_crossing = _active_crossing(plan, float(t))
        for rank, idx in enumerate(plan.order):
            color = COLORS[idx % len(COLORS)]
            window = track.windows[idx]
            for offset in (-0.8, 0.8):
                _draw_window_2d(ax, window.polygon_at(max(0.0, float(t + offset)), dynamic=True), color=color, lw=1.0, alpha=0.24, linestyle="--", fill_alpha=0.0)
            _draw_window_2d(ax, window.polygon_at(float(t), dynamic=True), color=color, lw=3.3 if active_crossing == rank else 2.0, alpha=0.95, linestyle="-", fill_alpha=0.14 if active_crossing == rank else 0.045)
            center2 = _project(window.center_at(float(t), dynamic=True)[None, :])[0]
            ax.text(center2[0], center2[1] + 0.62, window.name, color=color, fontsize=11, fontweight="bold", ha="center", zorder=8)
            if active_crossing == rank:
                p = plan.crossing_points[rank]
                p2 = _project(p[None, :])[0]
                ax.scatter([p2[0]], [p2[1]], color="#00a651", s=135, marker="o", edgecolors="black", linewidths=1.2, zorder=10)
                ax.text(p2[0] + 0.32, p2[1] + 0.32, "穿越成功", color="#007a2f", fontsize=9, weight="bold", zorder=11)
        drone_idx = min(upto - 1, len(traj.positions) - 1)
        _draw_drone_2d(ax, _project(traj.positions[drone_idx : drone_idx + 1])[0], float(traj.yaws[drone_idx]))
        status = "" if active_crossing is None else " | 穿越成功"
        ax.set_title(f"t={t:.2f}s{status} | {' -> '.join(plan.chosen_order)}", fontsize=13, fontweight="bold")
        frame_path = frame_dir / f"frame_{frame:03d}.png"
        fig.savefig(frame_path, dpi=110)
        plt.close(fig)
        images.append(imageio.imread(frame_path))
    imageio.mimsave(path, images, duration=0.14)
    for frame_path in frame_dir.glob("frame_*.png"):
        frame_path.unlink()
    frame_dir.rmdir()


def _project(points: np.ndarray) -> np.ndarray:
    """把三维点投影到二维示意图平面。

    这里使用 x 作为横轴，`y + 0.45 z` 作为纵轴，既保留前进方向，也让高度变化在图中可见。
    """
    pts = np.asarray(points, dtype=np.float64)
    return np.stack([pts[..., 0], pts[..., 1] + 0.45 * pts[..., 2]], axis=-1)


def _setup_schematic_axes(ax, track: WindowTrack, plan: DynaTOGTPlan) -> None:
    """设置示意图坐标范围、等比例显示和背景网格。

    坐标范围根据轨迹、起终点和窗口在穿越时刻前后的姿态自动计算，避免动态图形被裁切。
    """
    ax.set_aspect("equal")
    ax.axis("off")
    points = [plan.trajectory.positions, track.start[None, :], track.goal[None, :]]
    for idx, t in zip(plan.order, plan.crossing_times):
        window = track.windows[idx]
        points.extend([window.polygon_at(max(0.0, float(t - 1.4)), dynamic=True), window.polygon_at(float(t), dynamic=True), window.polygon_at(float(t + 1.4), dynamic=True)])
    pts2 = _project(np.vstack(points))
    lo = pts2.min(axis=0) - np.asarray([1.4, 1.2])
    hi = pts2.max(axis=0) + np.asarray([1.4, 1.2])
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    _draw_grid(ax, lo, hi)


def _draw_grid(ax, lo: np.ndarray, hi: np.ndarray) -> None:
    """绘制倾斜浅色网格背景，增强空间感但不干扰轨迹和窗口主体。"""
    for x in np.linspace(lo[0] - 2.0, hi[0] + 2.0, 24):
        ax.plot([x, x + 3.6], [lo[1], hi[1]], color="#e8edf3", lw=0.65, zorder=0)
    for y in np.linspace(lo[1] - 2.0, hi[1] + 2.0, 18):
        ax.plot([lo[0], hi[0]], [y, y + 1.9], color="#eef2f5", lw=0.65, zorder=0)


def _draw_window_2d(ax, polygon: np.ndarray, color: str, lw: float, alpha: float, linestyle: str, fill_alpha: float) -> None:
    """把三维窗口多边形投影后绘制到二维示意图中。"""
    poly = _project(polygon)
    closed = np.vstack([poly, poly[0]])
    if fill_alpha > 0.0:
        ax.fill(poly[:, 0], poly[:, 1], color=color, alpha=fill_alpha, zorder=4)
    ax.plot(closed[:, 0], closed[:, 1], color=color, lw=lw, linestyle=linestyle, alpha=alpha, zorder=5)


def _draw_drone_2d(ax, center: np.ndarray, yaw: float) -> None:
    """在示意图中绘制一个按 yaw 朝向旋转的简化无人机图标。"""
    body = np.asarray([[0.36, 0.0], [-0.20, 0.17], [-0.10, 0.0], [-0.20, -0.17]])
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.asarray([[c, -s], [s, c]])
    body = body @ rot.T + center[None, :]
    ax.fill(body[:, 0], body[:, 1], color="#101820", alpha=0.98, zorder=12)
    arms = np.asarray([[0.0, -0.32], [0.0, 0.32], [-0.32, 0.0], [0.32, 0.0]]) @ rot.T + center[None, :]
    ax.plot(arms[:2, 0], arms[:2, 1], color="#101820", lw=2.0, zorder=11)
    ax.plot(arms[2:, 0], arms[2:, 1], color="#101820", lw=2.0, zorder=11)
    ax.scatter(arms[:, 0], arms[:, 1], s=28, facecolors="white", edgecolors="#101820", linewidths=1.2, zorder=13)


def _active_crossing(plan: DynaTOGTPlan, t: float) -> int | None:
    """判断当前 GIF 帧是否正好对应某个精确穿越时刻。"""
    if not len(plan.crossing_times):
        return None
    nearest = int(np.argmin(np.abs(plan.crossing_times - t)))
    return nearest if abs(float(plan.crossing_times[nearest]) - t) < 1e-5 else None


def _draw_side_panel(ax, plan: DynaTOGTPlan) -> None:
    """绘制静态 PNG 右侧的信息面板。"""
    ax.axis("off")
    ax.text(0.5, 0.96, "DynaTOGT", ha="center", va="top", fontsize=18, fontweight="bold", color="#0649c9", transform=ax.transAxes)
    items = [("动态窗口约束", r"$p(t_i)\in G_i(t_i)$"), ("穿越顺序", _format_order(plan.chosen_order)), ("验证方式", "绿色点 + CSV 裕度")]
    y = 0.78
    for title, text in items:
        patch = plt.Rectangle((0.05, y - 0.10), 0.90, 0.15, transform=ax.transAxes, fill=False, lw=1.4, ec="#0649c9")
        ax.add_patch(patch)
        ax.text(0.10, y, title, fontsize=9.5, color="#0649c9", fontweight="bold", transform=ax.transAxes)
        ax.text(0.50, y - 0.060, text, fontsize=10.5, ha="center", transform=ax.transAxes)
        y -= 0.21
    ax.text(0.06, 0.19, "绿色点表示\n精确穿越位置。", fontsize=10, color="#007a2f", transform=ax.transAxes)
    ax.text(0.06, 0.08, "虚线表示窗口\n过去/未来姿态。", fontsize=10, transform=ax.transAxes)


def _format_order(order: list[str], per_line: int = 4) -> str:
    """把窗口顺序格式化成多行文本，避免侧边栏过长溢出。"""
    return "\n".join(" -> ".join(order[i : i + per_line]) for i in range(0, len(order), per_line))
