from __future__ import annotations

import csv
from pathlib import Path

import imageio.v2 as imageio
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .environment import WindowTrack
from .optimizer import DynaTOGTPlan

COLORS = ["#e41a1c", "#0b8f28", "#c98900", "#6a1fb0", "#1267d8", "#cc0b72", "#444444"]


def export_plan_csv(plan: DynaTOGTPlan, track: WindowTrack, path: Path) -> None:
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
    path.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(15.5, 9.2), facecolor="white")
    ax = fig.add_axes([0.04, 0.11, 0.72, 0.80])
    side = fig.add_axes([0.78, 0.16, 0.19, 0.72])
    _setup_schematic_axes(ax, track, plan)

    traj2 = _project(plan.trajectory.positions)
    ax.plot(traj2[:, 0], traj2[:, 1], color="#23272d", lw=4.2, alpha=0.16, solid_capstyle="round", zorder=1)
    ax.plot(traj2[:, 0], traj2[:, 1], color="#20242a", lw=2.25, solid_capstyle="round", zorder=2, label="continuous drone trajectory")

    start2 = _project(track.start[None, :])[0]
    goal2 = _project(track.goal[None, :])[0]
    ax.scatter([start2[0]], [start2[1]], color="#009e73", s=78, zorder=8, label="start")
    ax.scatter([goal2[0]], [goal2[1]], color="#d55e00", s=78, zorder=8, label="goal")

    for rank, (idx, crossing_t, point) in enumerate(zip(plan.order, plan.crossing_times, plan.crossing_points), start=1):
        color = COLORS[idx % len(COLORS)]
        window = track.windows[idx]
        for offset in (-1.4, -0.7, 0.7, 1.4):
            _draw_window_2d(ax, window.polygon_at(max(0.0, float(crossing_t + offset)), dynamic=True), color=color, lw=1.25, alpha=0.30, linestyle="--", fill_alpha=0.0)
        _draw_window_2d(ax, window.polygon_at(float(crossing_t), dynamic=True), color=color, lw=3.0, alpha=1.0, linestyle="-", fill_alpha=0.060)
        point2 = _project(np.asarray(point)[None, :])[0]
        local, plane_error = window.point_to_local(point, float(crossing_t), dynamic=True)
        margin = window.local_margin(local, float(crossing_t), dynamic=True)
        ax.scatter([point2[0]], [point2[1]], color="#00a651", s=84, edgecolors="black", linewidths=1.0, zorder=9)
        ax.text(point2[0], point2[1] - 0.40, f"t{rank}", color="black", fontsize=13, fontstyle="italic", ha="center", zorder=9)
        center2 = _project(window.center_at(float(crossing_t), dynamic=True)[None, :])[0]
        ax.text(center2[0], center2[1] + 0.72, window.name, color=color, fontsize=16, fontweight="bold", ha="center", zorder=9)
        ax.text(point2[0] + 0.30, point2[1] + 0.28, f"PASS\nm={margin:.2f}", color="#007a2f", fontsize=8.5, fontweight="bold", zorder=10)

    _draw_drone_2d(ax, _project(plan.trajectory.positions[-1:])[0], float(plan.trajectory.yaws[-1]))
    ax.set_title(title or "DynaTOGT Dynamic Time-Varying Window Traversal", fontsize=20, fontweight="bold", pad=18)
    ax.legend(loc="upper left", fontsize=9, frameon=True)
    _draw_side_panel(side, plan)
    fig.savefig(path, dpi=155)
    plt.close(fig)


def draw_plan_gif(track: WindowTrack, plan: DynaTOGTPlan, path: Path, frames: int = 32) -> None:
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
                local, plane_error = window.point_to_local(p, float(t), dynamic=True)
                margin = window.local_margin(local, float(t), dynamic=True)
                p2 = _project(p[None, :])[0]
                ax.scatter([p2[0]], [p2[1]], color="#00a651", s=135, marker="o", edgecolors="black", linewidths=1.2, zorder=10)
                ax.text(p2[0] + 0.32, p2[1] + 0.32, f"PASS {window.name}\nmargin={margin:.3f}\nplane={plane_error:.1e}", color="#007a2f", fontsize=9, weight="bold", zorder=11)
        drone_idx = min(upto - 1, len(traj.positions) - 1)
        _draw_drone_2d(ax, _project(traj.positions[drone_idx : drone_idx + 1])[0], float(traj.yaws[drone_idx]))
        status = "" if active_crossing is None else f" | PASS {plan.chosen_order[active_crossing]}"
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
    pts = np.asarray(points, dtype=np.float64)
    return np.stack([pts[..., 0], pts[..., 1] + 0.45 * pts[..., 2]], axis=-1)


def _setup_schematic_axes(ax, track: WindowTrack, plan: DynaTOGTPlan) -> None:
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
    for x in np.linspace(lo[0] - 2.0, hi[0] + 2.0, 24):
        ax.plot([x, x + 3.6], [lo[1], hi[1]], color="#e8edf3", lw=0.65, zorder=0)
    for y in np.linspace(lo[1] - 2.0, hi[1] + 2.0, 18):
        ax.plot([lo[0], hi[0]], [y, y + 1.9], color="#eef2f5", lw=0.65, zorder=0)


def _draw_window_2d(ax, polygon: np.ndarray, color: str, lw: float, alpha: float, linestyle: str, fill_alpha: float) -> None:
    poly = _project(polygon)
    closed = np.vstack([poly, poly[0]])
    if fill_alpha > 0.0:
        ax.fill(poly[:, 0], poly[:, 1], color=color, alpha=fill_alpha, zorder=4)
    ax.plot(closed[:, 0], closed[:, 1], color=color, lw=lw, linestyle=linestyle, alpha=alpha, zorder=5)


def _draw_drone_2d(ax, center: np.ndarray, yaw: float) -> None:
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
    if not len(plan.crossing_times):
        return None
    nearest = int(np.argmin(np.abs(plan.crossing_times - t)))
    return nearest if abs(float(plan.crossing_times[nearest]) - t) < 1e-5 else None


def _draw_side_panel(ax, plan: DynaTOGTPlan) -> None:
    ax.axis("off")
    ax.text(0.5, 0.96, "DynaTOGT", ha="center", va="top", fontsize=18, fontweight="bold", color="#0649c9", transform=ax.transAxes)
    items = [("Dynamic constraint", r"$p(t_i)\in G_i(t_i)$"), ("Chosen order", _format_order(plan.chosen_order)), ("Evidence", "PASS frames + CSV margins")]
    y = 0.78
    for title, text in items:
        patch = plt.Rectangle((0.05, y - 0.10), 0.90, 0.15, transform=ax.transAxes, fill=False, lw=1.4, ec="#0649c9")
        ax.add_patch(patch)
        ax.text(0.10, y, title, fontsize=9.5, color="#0649c9", fontweight="bold", transform=ax.transAxes)
        ax.text(0.50, y - 0.060, text, fontsize=10.5, ha="center", transform=ax.transAxes)
        y -= 0.21
    ax.text(0.06, 0.19, "Green PASS dots are\nexact crossing points.", fontsize=10, color="#007a2f", transform=ax.transAxes)
    ax.text(0.06, 0.08, "Dashed outlines show\npast / future poses.", fontsize=10, transform=ax.transAxes)


def _format_order(order: list[str], per_line: int = 4) -> str:
    return "\n".join(" -> ".join(order[i : i + per_line]) for i in range(0, len(order), per_line))
