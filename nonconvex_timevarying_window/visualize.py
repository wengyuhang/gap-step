from __future__ import annotations

import csv
from pathlib import Path

import imageio.v2 as imageio
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

from .environment import NonConvexWindowTrack
from .optimizer import AtlasDynaTOGTPlan

COLORS = ["#e41a1c", "#0b8f28", "#c98900", "#6a1fb0", "#1267d8", "#cc0b72"]


def _configure_chinese_font() -> None:
    """为 PNG/GIF 中文标注选择可用字体。"""
    names = ["Microsoft YaHei", "Droid Sans Fallback", "Noto Sans CJK SC", "SimHei", "WenQuanYi Micro Hei"]
    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in names:
        if name in available:
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            break


_configure_chinese_font()


def export_plan_csv(plan: AtlasDynaTOGTPlan, track: NonConvexWindowTrack, path: Path) -> None:
    """导出两类行：crossing 是穿越证据，sample 是轨迹采样。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["section", "index", "name", "chart_id", "t", "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az", "yaw", "local_u", "local_v", "plane_error", "boundary_margin", "chart_contains", "contains"]
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(fields)
        for i, (idx, cid, t, p) in enumerate(zip(plan.order, plan.chart_ids, plan.crossing_times, plan.crossing_points), start=1):
            w = track.windows[idx]
            local, err = w.point_to_local(p, float(t), dynamic=True)
            writer.writerow(["crossing", i, w.name, cid, f"{t:.6f}", f"{p[0]:.6f}", f"{p[1]:.6f}", f"{p[2]:.6f}", "", "", "", "", "", "", "", f"{local[0]:.6f}", f"{local[1]:.6f}", f"{err:.9f}", f"{w.local_margin(local, float(t), True):.9f}", w.chart_contains(cid, local, float(t), True), w.contains(p, float(t), True)])
        for i, (t, p, v, a, yaw) in enumerate(zip(plan.trajectory.times, plan.trajectory.positions, plan.trajectory.velocities, plan.trajectory.accelerations, plan.trajectory.yaws)):
            writer.writerow(["sample", i, "drone", "", f"{t:.6f}", f"{p[0]:.6f}", f"{p[1]:.6f}", f"{p[2]:.6f}", f"{v[0]:.6f}", f"{v[1]:.6f}", f"{v[2]:.6f}", f"{a[0]:.6f}", f"{a[1]:.6f}", f"{a[2]:.6f}", f"{yaw:.6f}", "", "", "", "", "", ""])


def draw_plan_png(track: NonConvexWindowTrack, plan: AtlasDynaTOGTPlan, path: Path, title: str | None = None) -> None:
    """导出一张完整轨迹图。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(12.5, 7.5), facecolor="white")
    _draw_frame(ax, track, plan, float(plan.duration), title or "非凸动态窗口穿越", full=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def draw_plan_gif(track: NonConvexWindowTrack, plan: AtlasDynaTOGTPlan, path: Path, frames: int = 32, playback_speed: float = 1.0) -> None:
    """按真实轨迹时间导出 GIF；playback_speed=0.5 表示半速。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    frame_dir = path.parent / f".frames_{path.stem}"
    frame_dir.mkdir(parents=True, exist_ok=True)
    times = np.unique(np.round(np.concatenate([np.linspace(0.0, plan.duration, frames), plan.crossing_times]), 6))
    images = []
    for i, t in enumerate(times):
        fig, ax = plt.subplots(figsize=(10.8, 7.0), facecolor="white")
        _draw_frame(ax, track, plan, float(t), f"t={t:.2f}s | {' -> '.join(plan.chosen_order)}", full=False)
        frame = frame_dir / f"frame_{i:03d}.png"
        fig.savefig(frame, dpi=110)
        plt.close(fig)
        images.append(imageio.imread(frame))
    imageio.mimsave(path, images, duration=_frame_durations(times, playback_speed))
    for frame in frame_dir.glob("frame_*.png"):
        frame.unlink()
    frame_dir.rmdir()


def _draw_frame(ax, track: NonConvexWindowTrack, plan: AtlasDynaTOGTPlan, t: float, title: str, full: bool) -> None:
    """画一帧：窗口取当前姿态，轨迹取 0..t；PNG 则画完整轨迹。"""
    _setup_axes(ax, track, plan)
    traj = plan.trajectory
    upto = len(traj.positions) if full else max(2, int(np.searchsorted(traj.times, t, side="right")))
    ax.plot(*_project(traj.positions).T, color="#4b5159", lw=1.1, alpha=0.22)
    ax.plot(*_project(traj.positions[:upto]).T, color="#20242a", lw=2.5, solid_capstyle="round", label="trajectory")
    for rank, idx in enumerate(plan.order):
        w, color = track.windows[idx], COLORS[idx % len(COLORS)]
        wt = float(plan.crossing_times[rank]) if full else t
        _draw_window(ax, w.polygon_at(max(0.0, wt), dynamic=True), color, fill=full)
        c = _project(w.center_at(wt, True)[None, :])[0]
        ax.text(c[0], c[1] + 0.55, w.name, color=color, fontsize=10, weight="bold", ha="center")
    for rank, p in enumerate(plan.crossing_points):
        if full or abs(float(plan.crossing_times[rank]) - t) < 1e-5:
            q = _project(p[None, :])[0]
            ax.scatter([q[0]], [q[1]], s=90, c="#00a651", edgecolors="black", zorder=9)
            ax.text(q[0] + 0.25, q[1] + 0.22, "非凸内穿越", fontsize=8, color="#007a2f", weight="bold")
    ax.scatter(*_project(track.start[None, :])[0], c="#009e73", s=70, label="start")
    ax.scatter(*_project(track.goal[None, :])[0], c="#d55e00", s=70, label="goal")
    ax.set_title(title, fontsize=14, weight="bold")
    ax.legend(loc="upper left", fontsize=8)


def _setup_axes(ax, track: NonConvexWindowTrack, plan: AtlasDynaTOGTPlan) -> None:
    """根据轨迹和窗口边界自动设置 2D 投影范围。"""
    ax.set_aspect("equal")
    ax.axis("off")
    pts = [plan.trajectory.positions, track.start[None, :], track.goal[None, :]]
    pts += [track.windows[i].polygon_at(float(t), True) for i, t in zip(plan.order, plan.crossing_times)]
    p2 = _project(np.vstack(pts))
    lo, hi = p2.min(axis=0) - [1.4, 1.2], p2.max(axis=0) + [1.4, 1.2]
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    for x in np.linspace(lo[0] - 2, hi[0] + 2, 20):
        ax.plot([x, x + 3.6], [lo[1], hi[1]], color="#edf1f5", lw=0.6, zorder=0)


def _project(points: np.ndarray) -> np.ndarray:
    """简易斜投影：[x,y,z] -> [x, y+0.45z]，让高度变化可见。"""
    p = np.asarray(points, dtype=np.float64)
    return np.stack([p[..., 0], p[..., 1] + 0.45 * p[..., 2]], axis=-1)


def _draw_window(ax, polygon: np.ndarray, color: str, fill: bool) -> None:
    """绘制非凸窗口边界。"""
    p = _project(polygon)
    if fill:
        ax.fill(p[:, 0], p[:, 1], color=color, alpha=0.06, zorder=3)
    closed = np.vstack([p, p[0]])
    ax.plot(closed[:, 0], closed[:, 1], color=color, lw=2.0, zorder=4)


def _frame_durations(frame_times: np.ndarray, playback_speed: float) -> list[float]:
    """每帧时长 = 相邻仿真时间差 / playback_speed。"""
    dt = np.diff(np.asarray(frame_times, dtype=np.float64))
    last = float(np.median(dt)) if len(dt) else 0.5
    return [float(max(0.04, d / max(playback_speed, 1e-6))) for d in np.r_[dt, last]]
