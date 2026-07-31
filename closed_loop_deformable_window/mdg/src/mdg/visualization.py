"""Static diagnostics and MP4 rendering for solved MDG instances."""

from __future__ import annotations

from pathlib import Path

import imageio.v2 as imageio
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle

from nonconvex_timevarying_window.sc_dynatogt.dynamics import sample_flatness

from .config import MDGConfig
from .dynamic_gate import Scenario
from .models import PlanResult


def _selected_gate_nodes(result: PlanResult):
    if result.graph_fine is None:
        return {}
    return {
        node.gate_index: node
        for node in result.graph_fine.selected_nodes
        if node.kind == "gate"
    }


def plot_overview(
    scenario: Scenario, result: PlanResult, path: str | Path
) -> Path:
    if result.backend is None:
        raise ValueError("overview requires a backend trajectory")
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    samples = result.backend.trajectory.sample(samples_per_segment=80)
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")
    position = np.real(samples.position)
    ax.plot(position[:, 0], position[:, 1], position[:, 2], color="#1368aa", lw=2.0)
    selected = _selected_gate_nodes(result)
    for index, gate in enumerate(scenario.gates):
        time = (
            result.backend.traversal_times[index]
            if index < len(result.backend.traversal_times)
            else 0.0
        )
        boundary = gate.boundary_world(float(time))
        closed = np.vstack((boundary, boundary[0]))
        ax.plot(closed[:, 0], closed[:, 1], closed[:, 2], color="#e07a1f", lw=1.5)
        if index in selected:
            node = selected[index]
            ax.scatter(
                node.center_world[0],
                node.center_world[1],
                node.center_world[2],
                color="#2a9d8f",
                s=30,
            )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title(f"{result.method}: {scenario.name}")
    fig.tight_layout()
    fig.savefig(target, dpi=180)
    plt.close(fig)
    return target


def plot_gate_diagnostics(
    scenario: Scenario,
    result: PlanResult,
    config: MDGConfig,
    path: str | Path,
) -> Path:
    if result.backend is None:
        raise ValueError("gate diagnostics require a backend result")
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    count = len(scenario.gates)
    columns = min(4, count)
    rows = int(np.ceil(count / columns))
    fig, axes = plt.subplots(rows, columns, figsize=(4 * columns, 3.8 * rows), squeeze=False)
    selected = _selected_gate_nodes(result)
    for index, gate in enumerate(scenario.gates):
        ax = axes[index // columns][index % columns]
        time = float(result.backend.traversal_times[index])
        physical = gate.local_polygon(time)
        safe = gate.safe_polygon(time, config.safety.safety_radius)
        xy = np.asarray(physical.exterior.coords)
        ax.plot(xy[:, 0], xy[:, 1], color="black", lw=1.5, label="physical")
        for component in getattr(safe, "geoms", (safe,)):
            if component.is_empty:
                continue
            safe_xy = np.asarray(component.exterior.coords)
            ax.fill(safe_xy[:, 0], safe_xy[:, 1], color="#8ecae6", alpha=0.45)
        for track in result.disc_tracks.get(index, []):
            if track.active_at(time):
                center, radius, _, _ = track.evaluate(time)
                ax.add_patch(Circle(center, radius, fill=False, color="#999999", lw=0.8))
        node = selected.get(index)
        if node is not None:
            ax.add_patch(
                Circle(node.center_local, node.radius, fill=False, color="#2a9d8f", lw=2.0)
            )
        point = result.backend.local_points[index]
        ax.scatter(point[0], point[1], marker="x", color="#d62828", s=45)
        ax.set_aspect("equal")
        ax.set_title(f"G{index + 1}  t={time:.2f}s")
    for index in range(count, rows * columns):
        axes[index // columns][index % columns].axis("off")
    fig.tight_layout()
    fig.savefig(target, dpi=180)
    plt.close(fig)
    return target


def plot_profiles(result: PlanResult, path: str | Path) -> Path:
    if result.backend is None:
        raise ValueError("profiles require a backend result")
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    samples = result.backend.trajectory.sample(samples_per_segment=100)
    speed = np.linalg.norm(np.real(samples.velocity), axis=1)
    acceleration = np.linalg.norm(np.real(samples.acceleration), axis=1)
    flatness = sample_flatness(
        result.backend.trajectory,
        samples.time,
    )
    body_rate = np.linalg.norm(np.real(flatness.body_rate), axis=1)
    rotor_peak = np.max(np.real(flatness.rotor_thrusts), axis=1)
    fig, axes = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    axes[0].plot(samples.time, speed, color="#1368aa")
    axes[0].set_ylabel("speed [m/s]")
    axes[1].plot(samples.time, acceleration, color="#e07a1f")
    axes[1].set_ylabel("acceleration [m/s²]")
    axes[2].plot(flatness.time, body_rate, color="#6a4c93")
    axes[2].set_ylabel("body rate [rad/s]")
    axes[3].plot(flatness.time, rotor_peak, color="#2a9d8f")
    axes[3].set_ylabel("max rotor [N]")
    axes[3].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(target, dpi=180)
    plt.close(fig)
    return target


def make_video(
    scenario: Scenario,
    result: PlanResult,
    config: MDGConfig,
    path: str | Path,
) -> Path:
    if result.backend is None:
        raise ValueError("video requires a backend result")
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    frame_count = max(2, int(config.runtime.video_fps * config.runtime.video_duration))
    times = np.linspace(0.0, result.backend.total_time, frame_count)
    selected = _selected_gate_nodes(result)
    with imageio.get_writer(
        target,
        fps=config.runtime.video_fps,
        codec="libx264",
        quality=7,
        macro_block_size=2,
    ) as writer:
        for time in times:
            # 1080 x 608 pixels: both dimensions are codec-friendly.
            fig = plt.figure(figsize=(12, 608.0 / 90.0), dpi=90)
            grid = fig.add_gridspec(2, 2, width_ratios=(1.55, 1.0))
            ax3d = fig.add_subplot(grid[:, 0], projection="3d")
            point = np.real(result.backend.trajectory.evaluate(float(time)))
            history_times = np.linspace(0.0, float(time), 120)
            history = np.asarray(
                [result.backend.trajectory.evaluate(float(value)) for value in history_times]
            ).real
            ax3d.plot(history[:, 0], history[:, 1], history[:, 2], color="#1368aa")
            ax3d.scatter(*point, color="#d62828", s=35)
            for gate in scenario.gates:
                boundary = gate.boundary_world(float(time))
                closed = np.vstack((boundary, boundary[0]))
                ax3d.plot(closed[:, 0], closed[:, 1], closed[:, 2], color="#e07a1f", lw=1.0)
            ax3d.set_xlim(-10, 10)
            ax3d.set_ylim(-10, 10)
            ax3d.set_zlim(0, config.scenario.world_size[2])
            current = min(
                len(scenario.gates) - 1,
                int(np.searchsorted(result.backend.traversal_times, time)),
            )
            gate = scenario.gates[current]
            ax_gate = fig.add_subplot(grid[0, 1])
            physical = gate.local_polygon(float(time))
            xy = np.asarray(physical.exterior.coords)
            ax_gate.plot(xy[:, 0], xy[:, 1], color="black")
            safe = gate.safe_polygon(float(time), config.safety.safety_radius)
            for component in getattr(safe, "geoms", (safe,)):
                if not component.is_empty:
                    safe_xy = np.asarray(component.exterior.coords)
                    ax_gate.fill(safe_xy[:, 0], safe_xy[:, 1], color="#8ecae6", alpha=0.4)
            for track in result.disc_tracks.get(current, []):
                if track.active_at(float(time)):
                    center, radius, _, _ = track.evaluate(float(time))
                    ax_gate.add_patch(Circle(center, radius, fill=False, color="#777777", lw=0.7))
            node = selected.get(current)
            if node is not None:
                ax_gate.add_patch(
                    Circle(node.center_local, node.radius, fill=False, color="#2a9d8f", lw=2)
                )
                ax_gate.scatter(
                    result.backend.local_points[current, 0],
                    result.backend.local_points[current, 1],
                    color="#d62828",
                    marker="x",
                )
            ax_gate.set_aspect("equal")
            ax_gate.set_title(f"current gate: G{current + 1}")
            ax_time = fig.add_subplot(grid[1, 1])
            ax_time.axhline(0.0, color="#666666")
            ax_time.scatter(
                result.backend.traversal_times,
                np.zeros(len(result.backend.traversal_times)),
                c=np.arange(len(result.backend.traversal_times)),
                cmap="viridis",
                s=35,
            )
            ax_time.axvline(time, color="#d62828")
            ax_time.set_xlim(0.0, result.backend.total_time)
            ax_time.set_yticks([])
            ax_time.set_xlabel("time [s]")
            fig.tight_layout()
            fig.canvas.draw()
            frame = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
            writer.append_data(frame)
            plt.close(fig)
    return target


__all__ = [
    "make_video",
    "plot_gate_diagnostics",
    "plot_overview",
    "plot_profiles",
]
