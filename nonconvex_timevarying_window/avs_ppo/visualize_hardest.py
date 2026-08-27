"""Visualize the AVS-PPO cuboid rollout on the hardest comparison track."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import imageio.v2 as imageio
import matplotlib

matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon as PolygonPatch  # noqa: E402
import numpy as np

from nonconvex_timevarying_window.comparisons.sc_sip_fast_closed_loop.cuboid_replay import (
    _draw_cuboid_3d,
    cuboid_vertices,
)

from .hardest_comparison import load_hardest_config
from .hardest_evaluate import (
    dense_whole_body_audit,
    load_hardest_checkpoint,
    rollout_policy,
)
from .ppo import device_from_config


def _boundary_world(environment, window_index: int, time: float, samples: int = 100) -> np.ndarray:
    window = environment.problem.windows[window_index]
    pieces = []
    for index, segment in enumerate(window.boundary):
        values = np.linspace(0.0, 1.0, samples, endpoint=index == len(window.boundary) - 1)
        pieces.append(np.asarray([segment.evaluate(float(value)) for value in values]))
    local = np.vstack(pieces)
    center, basis, scale = window.state_at(time)
    return center + (basis @ np.column_stack((scale * local, np.zeros(len(local)))).T).T


def plot_route(environment, output: Path) -> None:
    positions = np.asarray([record["position"] for record in environment.history] + [environment.state.position])
    reference_times = np.linspace(0.0, environment.reference.total_time, 1000)
    reference = np.asarray([environment._reference_values(float(time))[0] for time in reference_times])
    figure = plt.figure(figsize=(12.0, 8.2), constrained_layout=True)
    axis = figure.add_subplot(111, projection="3d")
    axis.plot(*reference.T, color="0.60", linewidth=1.1, linestyle="--", label="certified SIP reference")
    axis.plot(*positions.T, color="#1565c0", linewidth=2.1, label="AVS-PPO rollout")
    for crossing in environment.crossing_records:
        boundary = _boundary_world(environment, crossing["window_index"], crossing["time"])
        axis.plot(*np.vstack((boundary, boundary[0])).T, color="#d95319", linewidth=1.4)
        vertices = cuboid_vertices(crossing["position"], crossing["rotation"], environment.body.half_extents)
        _draw_cuboid_3d(axis, vertices, alpha=0.48)
        axis.text(*crossing["position"], f" {crossing['route_index'] + 1}:{crossing['window_name']}", fontsize=7)
    axis.scatter(*positions[0], color="#2e7d32", s=55, label="start / finish")
    axis.set_xlabel("x [m]"); axis.set_ylabel("y [m]"); axis.set_zlabel("z [m]")
    axis.set_title("AVS-PPO on wide_scrambled_fast_closed_loop_6 — full oriented cuboid")
    axis.view_init(elev=23, azim=-58)
    axis.legend(loc="upper left")
    figure.savefig(output, dpi=185, bbox_inches="tight")
    plt.close(figure)


def plot_crossings(environment, output: Path) -> None:
    figure, axes = plt.subplots(2, 3, figsize=(14.2, 8.8), constrained_layout=True)
    for axis, crossing in zip(axes.flat, environment.crossing_records):
        window = environment.problem.windows[crossing["window_index"]]
        center, basis, scale = window.state_at(crossing["time"])
        local = environment._local_polygons[crossing["window_index"]] * scale
        axis.plot(*np.vstack((local, local[0])).T, color="#c62828", linewidth=2.0)
        axis.fill(local[:, 0], local[:, 1], color="#ffccbc", alpha=0.40)
        section = crossing["section"]
        if len(section) >= 3:
            projected = (section - center) @ basis[:, :2]
            centroid = projected.mean(axis=0)
            order = np.argsort(np.arctan2(projected[:, 1] - centroid[1], projected[:, 0] - centroid[0]))
            axis.add_patch(PolygonPatch(
                projected[order], closed=True, facecolor="#2878b5",
                edgecolor="#123f62", alpha=0.58,
            ))
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, color="0.88", linewidth=0.55)
        axis.set_xlabel("gate local u [m]"); axis.set_ylabel("gate local v [m]")
        axis.set_title(
            f"{crossing['route_index'] + 1}. {crossing['window_name']}  t={crossing['time']:.3f}s\n"
            f"cuboid-to-frame clearance={crossing['clearance']:.4f}m"
        )
    figure.suptitle(
        "AVS-PPO cuboid/plane sections at all six legal traversals\n"
        f"half-extents={environment.body.half_extents} m; required clearance={environment.required_clearance:.3f} m",
        fontsize=14,
    )
    figure.savefig(output, dpi=185, bbox_inches="tight")
    plt.close(figure)


def plot_safety(environment, audit: dict, learning_curve: Path, output: Path) -> None:
    times = np.asarray([record["time"] + environment.config.dt for record in environment.history])
    clearance = np.asarray([record["clearance"] for record in environment.history])
    finite = np.isfinite(clearance)
    with learning_curve.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    updates = np.asarray([int(row["update"]) for row in rows])
    success = np.asarray([float(row["success_rate"]) for row in rows])
    entropy = np.asarray([float(row.get("entropy") or np.nan) for row in rows])
    figure, axes = plt.subplots(2, 1, figsize=(11.5, 7.7), constrained_layout=True)
    clipped_clearance = np.minimum(clearance[finite] * 1000.0, 120.0)
    axes[0].plot(times[finite], clipped_clearance, color="#1565c0", linewidth=1.7, label="shield sampling (>120 mm clipped)")
    axes[0].axhline(environment.required_clearance * 1000.0, color="#c62828", linestyle="--", label="15 mm requirement")
    refined = audit["locally_refined_minimum"]
    axes[0].scatter([refined["time"]], [refined["distance"] * 1000.0], color="#c62828", zorder=4,
                    label=f"local refinement: {refined['distance'] * 1000.0:.2f} mm")
    axes[0].set_xlabel("time [s]"); axes[0].set_ylabel("whole-body clearance [mm]")
    axes[0].set_ylim(0.0, 125.0)
    axes[0].set_title("Full-cuboid safety audit")
    axes[0].grid(True, color="0.88", linewidth=0.55); axes[0].legend()
    axes[1].plot(updates, success * 100.0, marker="o", color="#2e7d32", label="deterministic success")
    axes[1].set_ylim(-3, 103); axes[1].set_xlabel("PPO update"); axes[1].set_ylabel("success [%]")
    axes[1].set_title("Training result: the strict shield dominates the policy support")
    axes[1].grid(True, color="0.88", linewidth=0.55)
    if np.isfinite(entropy).any():
        twin = axes[1].twinx()
        twin.plot(updates[np.isfinite(entropy)], entropy[np.isfinite(entropy)], marker="s", color="#8e44ad", label="masked entropy")
        twin.set_ylabel("masked policy entropy [nat]")
    figure.savefig(output, dpi=185, bbox_inches="tight")
    plt.close(figure)


def export_gif(environment, output: Path, frames: int = 84) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.parent / "_hardest_avs_frames"
    temporary.mkdir(parents=True, exist_ok=True)
    indices = np.linspace(0, len(environment.history) - 1, frames).astype(int)
    paths = []
    try:
        all_positions = np.asarray([record["position"] for record in environment.history])
        for frame_index, history_index in enumerate(indices):
            record = environment.history[int(history_index)]
            time = float(record["time"])
            figure = plt.figure(figsize=(9.2, 6.4), constrained_layout=True)
            axis = figure.add_subplot(111, projection="3d")
            for window_index in range(len(environment.problem.windows)):
                boundary = _boundary_world(environment, window_index, time, 55)
                axis.plot(*np.vstack((boundary, boundary[0])).T, color="#d95319", linewidth=1.0, alpha=0.82)
            axis.plot(*all_positions[:history_index + 1].T, color="#1565c0", linewidth=1.8)
            vertices = cuboid_vertices(record["position"], record["rotation"], environment.body.half_extents)
            _draw_cuboid_3d(axis, vertices, alpha=0.74)
            axis.set_xlim(-16, 17); axis.set_ylim(-20, 17); axis.set_zlim(0, 16)
            axis.set_xlabel("x [m]"); axis.set_ylabel("y [m]"); axis.set_zlabel("z [m]")
            axis.set_title(f"AVS-PPO cuboid replay — t={time:.2f} / {environment.state.time:.2f} s")
            axis.view_init(elev=24, azim=-58)
            path = temporary / f"frame_{frame_index:04d}.png"
            figure.savefig(path, dpi=105, bbox_inches="tight")
            plt.close(figure)
            paths.append(path)
        with imageio.get_writer(output, mode="I", duration=0.10, loop=0) as writer:
            for path in paths:
                writer.append_data(imageio.imread(path))
    finally:
        for path in paths:
            path.unlink(missing_ok=True)
        temporary.rmdir()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--learning-curve", required=True)
    parser.add_argument("--outdir", required=True)
    parser.add_argument("--no-gif", action="store_true")
    args = parser.parse_args()
    config = load_hardest_config(args.config)
    device = device_from_config(config.ppo.device)
    model = load_hardest_checkpoint(args.checkpoint, config, device)
    environment, rollout = rollout_policy(model, config, seed=41000)
    audit = dense_whole_body_audit(environment)
    output = Path(args.outdir)
    output.mkdir(parents=True, exist_ok=True)
    plot_route(environment, output / "avs_cuboid_route.png")
    plot_crossings(environment, output / "avs_cuboid_crossings.png")
    plot_safety(environment, audit, Path(args.learning_curve), output / "avs_safety_and_training.png")
    if not args.no_gif:
        export_gif(environment, output / "avs_cuboid_replay.gif")
    summary = {
        "scenario": environment.problem.name,
        "method": "AVS-PPO",
        "rollout": rollout,
        "dense_whole_body_audit": audit,
        "body_half_extents": list(environment.body.half_extents),
        "shield_support_mean": float(np.mean([record["mask_fraction"] for record in environment.history])),
    }
    (output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
