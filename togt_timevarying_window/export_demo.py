from __future__ import annotations

import argparse
import csv
from pathlib import Path

import imageio.v2 as imageio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from .environment import demo_track
from .planner import DynamicTOGTPlanner, PlannerConfig


def _set_axes_equal(ax, points: np.ndarray) -> None:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = 0.5 * (mins + maxs)
    radius = 0.55 * float(np.max(maxs - mins) + 1e-6)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(max(0.0, center[2] - radius), center[2] + radius)


def _draw_gate(ax, gate, t: float, alpha: float = 0.18) -> None:
    poly = gate.polygon_at(t)
    closed = np.vstack([poly, poly[0]])
    ax.plot(closed[:, 0], closed[:, 1], closed[:, 2], color="#1b9e77", lw=1.8)
    ax.add_collection3d(Poly3DCollection([poly], facecolor="#1b9e77", edgecolor="none", alpha=alpha))
    c = gate.center_at(t)
    ax.text(c[0], c[1], c[2], gate.name, fontsize=8, ha="center", va="center")


def _draw_scene(track, traj, png_path: Path, camera_angle: float = -58.0) -> None:
    fig = plt.figure(figsize=(10, 5.8))
    ax = fig.add_subplot(111, projection="3d")
    pts = np.asarray(traj.points)
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "o-", color="#d95f02", lw=2.2, ms=4, label="planned trajectory")
    for idx, gate in enumerate(track.gates):
        _draw_gate(ax, gate, traj.gate_times[idx])
    ax.scatter([track.start[0]], [track.start[1]], [track.start[2]], c="#2ca02c", s=70, label="start")
    ax.scatter([track.goal[0]], [track.goal[1]], [track.goal[2]], c="#d62728", s=70, label="goal")
    all_gate_points = np.vstack([gate.polygon_at(traj.gate_times[idx]) for idx, gate in enumerate(track.gates)])
    _set_axes_equal(ax, np.vstack([pts, all_gate_points, track.start[None, :], track.goal[None, :]]))
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.view_init(elev=22.0, azim=camera_angle)
    ax.set_title(f"3D dynamic gate TOGT demo lap={traj.lap_time:.2f}s")
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(png_path, dpi=160)
    plt.close(fig)


def _draw_gif(track, traj, gif_path: Path, frames: int = 36) -> None:
    tmp_dir = gif_path.parent / ".frames"
    tmp_dir.mkdir(parents=True, exist_ok=True)
    images = []
    pts = np.asarray(traj.points)
    gate_times = np.asarray(traj.gate_times)
    for frame in range(frames):
        t = traj.lap_time * frame / max(1, frames - 1)
        fig = plt.figure(figsize=(7.2, 4.8))
        ax = fig.add_subplot(111, projection="3d")
        upto = max(2, int(np.searchsorted(traj.times, t, side="right")))
        ax.plot(pts[:upto, 0], pts[:upto, 1], pts[:upto, 2], "o-", color="#d95f02", lw=2.0, ms=3)
        for idx, gate in enumerate(track.gates):
            draw_t = min(float(gate_times[idx]), t) if t >= gate_times[idx] else t
            _draw_gate(ax, gate, draw_t, alpha=0.13)
        all_gate_points = np.vstack([gate.polygon_at(t) for gate in track.gates])
        _set_axes_equal(ax, np.vstack([pts, all_gate_points, track.start[None, :], track.goal[None, :]]))
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.view_init(elev=24.0, azim=-62.0 + 18.0 * frame / max(1, frames - 1))
        ax.set_title(f"t={t:.2f}s / {traj.lap_time:.2f}s")
        fig.tight_layout()
        frame_path = tmp_dir / f"frame_{frame:03d}.png"
        fig.savefig(frame_path, dpi=120)
        plt.close(fig)
        images.append(imageio.imread(frame_path))
    imageio.mimsave(gif_path, images, duration=0.08)
    for frame_path in tmp_dir.glob("frame_*.png"):
        frame_path.unlink()
    tmp_dir.rmdir()


def main() -> None:
    parser = argparse.ArgumentParser(description="Export 3D dynamic TOGT demo trajectory, figure, and GIF.")
    parser.add_argument("--static", action="store_true")
    parser.add_argument("--outdir", default="togt_timevarying_window/outputs")
    parser.add_argument("--max-time", type=float, default=70.0)
    parser.add_argument("--frames", type=int, default=36)
    args = parser.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    track = demo_track(dynamic=not args.static)
    traj = DynamicTOGTPlanner(PlannerConfig(max_time=args.max_time, max_speed=2.35, wait_steps=8, gate_samples_per_axis=1)).plan(track)
    if traj is None:
        raise SystemExit("planning_failed")

    tag = "static" if args.static else "dynamic"
    csv_path = outdir / f"{tag}_trajectory.csv"
    png_path = outdir / f"{tag}_trajectory.png"
    gif_path = outdir / f"{tag}_trajectory.gif"

    with csv_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["stage", "t", "x", "y", "z"])
        for stage, (t, point) in enumerate(zip(traj.times, traj.points)):
            writer.writerow([stage - 1, f"{t:.6f}", f"{point[0]:.6f}", f"{point[1]:.6f}", f"{point[2]:.6f}"])

    _draw_scene(track, traj, png_path)
    _draw_gif(track, traj, gif_path, frames=args.frames)

    print(f"csv={csv_path}")
    print(f"png={png_path}")
    print(f"gif={gif_path}")
    print(f"lap_time={traj.lap_time:.2f}")
    print(f"length={traj.length:.2f}")


if __name__ == "__main__":
    main()
