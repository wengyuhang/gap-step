"""Headless visualizations and trajectory exports for SC-DynaTOGT.

The functions in this module are deliberately file-oriented: every output
path is created on demand, and every Matplotlib figure is closed even if
rendering fails.  This makes the same API suitable for command-line
experiments, CI, and machines without a display server.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import imageio.v2 as imageio
import matplotlib

# SC-DynaTOGT visualizations are experiment artifacts, never interactive
# widgets.  Selecting Agg here keeps imports safe on headless compute nodes.
matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402
import numpy as np
from numpy.typing import ArrayLike, NDArray

from .environment import SCWindowTrack
from .minco import MincoSnap
from .optimizer import OptimizationResult
from .preprocessing import PreprocessedGate


FloatArray = NDArray[np.float64]
TrajectoryInput = MincoSnap | OptimizationResult


def _output_path(path: str | Path) -> Path:
    output = Path(path).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    return output


def _closed(vertices: ArrayLike) -> FloatArray:
    points = np.asarray(vertices, dtype=float)
    if points.ndim != 2 or points.shape[1] not in (2, 3) or len(points) < 2:
        raise ValueError("vertices must have shape (n, 2) or (n, 3)")
    return np.vstack((points, points[0]))


def _trajectory(value: TrajectoryInput) -> MincoSnap:
    if isinstance(value, MincoSnap):
        return value
    trajectory = getattr(value, "trajectory", None)
    if not isinstance(trajectory, MincoSnap):
        raise TypeError("trajectory_or_result must be MincoSnap or OptimizationResult")
    return trajectory


def _crossing_times(value: TrajectoryInput, trajectory: MincoSnap, count: int) -> FloatArray:
    supplied = getattr(value, "traversal_times", None)
    if supplied is None:
        times = np.cumsum(np.real(trajectory.durations))[:-1]
    else:
        times = np.asarray(supplied, dtype=float)
    if times.shape != (count,):
        raise ValueError(
            f"expected {count} traversal times for the track, got shape {times.shape}"
        )
    total = float(np.real(trajectory.total_time))
    if np.any(times < 0.0) or np.any(times > total):
        raise ValueError("traversal times must lie inside the trajectory interval")
    return times


def _mapped_grid(
    gate: PreprocessedGate,
    *,
    radial_count: int,
    angular_count: int,
    samples_per_line: int,
    maximum_radius: float,
) -> tuple[list[FloatArray], list[FloatArray], list[FloatArray], list[FloatArray]]:
    if radial_count < 1 or angular_count < 2 or samples_per_line < 8:
        raise ValueError("SC grid counts are too small")
    if not (0.0 < maximum_radius < 1.0):
        raise ValueError("maximum_radius must lie in (0, 1)")

    disk_lines: list[FloatArray] = []
    mapped_lines: list[FloatArray] = []
    theta = np.linspace(0.0, 2.0 * np.pi, samples_per_line, endpoint=True)
    for radius in np.linspace(maximum_radius / radial_count, maximum_radius, radial_count):
        disk = np.column_stack((radius * np.cos(theta), radius * np.sin(theta)))
        disk_lines.append(disk)
        mapped_lines.append(gate.sc_map.evaluate_many(disk))

    disk_rays: list[FloatArray] = []
    mapped_rays: list[FloatArray] = []
    radius = np.linspace(0.0, maximum_radius, max(8, samples_per_line // 2))
    for angle in np.linspace(0.0, 2.0 * np.pi, angular_count, endpoint=False):
        ray = np.column_stack((radius * np.cos(angle), radius * np.sin(angle)))
        disk_rays.append(ray)
        mapped_rays.append(gate.sc_map.evaluate_many(ray))
    return disk_lines, mapped_lines, disk_rays, mapped_rays


def plot_preprocessing(
    gate: PreprocessedGate,
    output_path: str | Path,
    *,
    radial_count: int = 5,
    angular_count: int = 12,
    samples_per_line: int = 80,
    maximum_radius: float = 0.96,
    dpi: int = 160,
) -> Path:
    """Plot dense/sample/corner/inset geometry and its disk-SC grid.

    The Chang output is shown only as a boundary sampling stage.  The third
    panel is evaluated exclusively through the persisted Schwarz--Christoffel
    map, making the distinction between the two roles visible in the output.
    """

    output = _output_path(output_path)
    disk_lines, mapped_lines, disk_rays, mapped_rays = _mapped_grid(
        gate,
        radial_count=radial_count,
        angular_count=angular_count,
        samples_per_line=samples_per_line,
        maximum_radius=maximum_radius,
    )
    figure, axes = plt.subplots(1, 3, figsize=(14.0, 4.6), constrained_layout=True)
    try:
        dense = _closed(gate.dense_boundary.vertices)
        sampled = _closed(gate.sampled_boundary.vertices)
        safe = _closed(gate.safe_polygon)

        axes[0].plot(dense[:, 0], dense[:, 1], color="0.72", linewidth=1.0, label="dense")
        axes[0].plot(
            sampled[:, 0], sampled[:, 1], "o-", color="#2878b5", markersize=2.5,
            linewidth=1.0, label=f"Chang sample (m={gate.selected_vertex_count})",
        )
        axes[0].plot(safe[:, 0], safe[:, 1], color="#d95319", linewidth=2.0, label="safe inset")
        corners = np.asarray(gate.dense_boundary.corners, dtype=float)
        if len(corners):
            axes[0].scatter(
                corners[:, 0], corners[:, 1], marker="*", s=70, color="#7e2f8e",
                edgecolor="white", linewidth=0.4, zorder=5, label="preserved corners",
            )
        axes[0].set_title("Boundary preprocessing")
        axes[0].legend(fontsize=7, loc="best")

        unit_theta = np.linspace(0.0, 2.0 * np.pi, 256)
        axes[1].plot(np.cos(unit_theta), np.sin(unit_theta), color="0.15", linewidth=1.2)
        for line in disk_lines:
            axes[1].plot(line[:, 0], line[:, 1], color="#2878b5", linewidth=0.7)
        for line in disk_rays:
            axes[1].plot(line[:, 0], line[:, 1], color="#2878b5", linewidth=0.7)
        axes[1].scatter([0.0], [0.0], color="#d95319", s=18, zorder=4)
        axes[1].set_title("Unit-disk grid")

        axes[2].plot(safe[:, 0], safe[:, 1], color="0.15", linewidth=1.5)
        for line in mapped_lines:
            axes[2].plot(line[:, 0], line[:, 1], color="#2878b5", linewidth=0.7)
        for line in mapped_rays:
            axes[2].plot(line[:, 0], line[:, 1], color="#2878b5", linewidth=0.7)
        axes[2].scatter(
            [gate.sc_map.q0[0]], [gate.sc_map.q0[1]], color="#d95319", s=18, zorder=4,
            label="polylabel normalization",
        )
        axes[2].set_title("Schwarz--Christoffel image")
        axes[2].legend(fontsize=7, loc="best")

        for axis in axes:
            axis.set_aspect("equal", adjustable="box")
            axis.set_xlabel("local x [m]")
            axis.set_ylabel("local y [m]")
            axis.grid(True, color="0.9", linewidth=0.5)
        axes[1].set_xlabel("Re(z)")
        axes[1].set_ylabel("Im(z)")
        figure.suptitle(f"SC-DynaTOGT preprocessing: {gate.name}")
        figure.savefig(output, dpi=dpi, bbox_inches="tight")
    finally:
        plt.close(figure)
    return output


def _add_window_polygon(axis: object, polygon: FloatArray, *, color: str, label: str | None) -> None:
    collection = Poly3DCollection(
        [polygon], facecolor=color, edgecolor=color, linewidth=1.1, alpha=0.20,
        label=label,
    )
    axis.add_collection3d(collection)  # type: ignore[attr-defined]
    closed = _closed(polygon)
    axis.plot(closed[:, 0], closed[:, 1], closed[:, 2], color=color, linewidth=1.2)  # type: ignore[attr-defined]


def _add_physical_boundary(
    axis: object,
    boundary: FloatArray,
    *,
    label: str | None,
) -> None:
    """Draw the original aperture boundary without obscuring the inset safe set."""

    closed = _closed(boundary)
    axis.plot(  # type: ignore[attr-defined]
        closed[:, 0],
        closed[:, 1],
        closed[:, 2],
        color="0.18",
        linewidth=1.6,
        linestyle="--",
        label=label,
    )


def _axis_limits(points: Sequence[FloatArray]) -> tuple[FloatArray, FloatArray]:
    nonempty = [np.asarray(point, dtype=float).reshape(-1, 3) for point in points if np.size(point)]
    if not nonempty:
        raise ValueError("at least one 3D point is required")
    combined = np.vstack(nonempty)
    lower = combined.min(axis=0)
    upper = combined.max(axis=0)
    span = np.maximum(upper - lower, 1.0e-3)
    padding = np.maximum(0.08 * span, 0.05)
    return lower - padding, upper + padding


def _set_3d_limits(axis: object, lower: FloatArray, upper: FloatArray) -> None:
    axis.set_xlim(float(lower[0]), float(upper[0]))  # type: ignore[attr-defined]
    axis.set_ylim(float(lower[1]), float(upper[1]))  # type: ignore[attr-defined]
    axis.set_zlim(float(lower[2]), float(upper[2]))  # type: ignore[attr-defined]
    axis.set_box_aspect(np.maximum(upper - lower, 1.0e-3))  # type: ignore[attr-defined]


def plot_trajectory(
    track: SCWindowTrack,
    trajectory_or_result: TrajectoryInput,
    output_path: str | Path,
    *,
    num_samples: int = 401,
    dpi: int = 160,
) -> Path:
    """Plot a 3D MINCO path and every gate at its traversal time."""

    if num_samples < 2:
        raise ValueError("num_samples must be at least two")
    output = _output_path(output_path)
    trajectory = _trajectory(trajectory_or_result)
    crossings = _crossing_times(trajectory_or_result, trajectory, len(track.order))
    samples = trajectory.sample(num_samples=num_samples)
    positions = np.asarray(np.real(samples.position), dtype=float)
    safe_polygons = [
        track.windows[window_index].polygon_at(float(crossings[crossing_index]))
        for crossing_index, window_index in enumerate(track.order)
    ]
    physical_boundaries = [
        track.windows[window_index].physical_boundary_at(float(crossings[crossing_index]))
        for crossing_index, window_index in enumerate(track.order)
    ]
    limit_geometry = [boundary for boundary in physical_boundaries if boundary is not None]
    lower, upper = _axis_limits([positions, *safe_polygons, *limit_geometry])

    many_gates = len(safe_polygons) > 3
    figure = plt.figure(figsize=(10.4 if many_gates else 8.0, 6.4), constrained_layout=True)
    try:
        axis = figure.add_subplot(111, projection="3d")
        axis.plot(positions[:, 0], positions[:, 1], positions[:, 2], color="#0072bd", linewidth=2.1, label="MINCO")
        axis.scatter(*track.start, color="#2ca02c", s=42, marker="o", label="start")
        axis.scatter(*track.goal, color="#d62728", s=48, marker="X", label="goal")
        colors = plt.cm.viridis(np.linspace(0.12, 0.88, max(1, len(safe_polygons))))
        for index, (safe_polygon, physical_boundary) in enumerate(
            zip(safe_polygons, physical_boundaries)
        ):
            if physical_boundary is not None:
                _add_physical_boundary(
                    axis,
                    physical_boundary,
                    label="physical boundary" if index == 0 else None,
                )
            window = track.windows[track.order[index]]
            _add_window_polygon(
                axis,
                safe_polygon,
                color=matplotlib.colors.to_hex(colors[index]),
                label=(
                    f"gate {index + 1} {window.name} safe at "
                    f"t={crossings[index]:.2f}s"
                ),
            )
        crossing_points = np.asarray(trajectory.evaluate(crossings), dtype=float)
        if len(crossing_points):
            axis.scatter(
                crossing_points[:, 0], crossing_points[:, 1], crossing_points[:, 2],
                color="#ff7f0e", edgecolors="0.15", linewidths=0.6,
                s=28, marker="D", label="crossing center",
            )
        _set_3d_limits(axis, lower, upper)
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.set_zlabel("z [m]")
        axis.set_title(f"{track.name}: ordered dynamic-window traversal")
        axis.view_init(elev=24.0, azim=-61.0)
        if many_gates:
            axis.legend(
                fontsize=7.2,
                loc="center left",
                bbox_to_anchor=(1.01, 0.5),
                framealpha=1.0,
                facecolor="white",
            )
        else:
            axis.legend(fontsize=8, loc="best")
        figure.savefig(output, dpi=dpi, bbox_inches="tight")
    finally:
        plt.close(figure)
    return output


def export_trajectory_csv(
    trajectory_or_result: TrajectoryInput,
    output_path: str | Path,
    *,
    num_samples: int = 501,
    times: ArrayLike | None = None,
) -> Path:
    """Export time, position, and derivatives through crackle as a CSV."""

    if times is not None and num_samples != 501:
        raise ValueError("specify custom times without also changing num_samples")
    trajectory = _trajectory(trajectory_or_result)
    samples = (
        trajectory.sample(times=np.asarray(times, dtype=float))
        if times is not None
        else trajectory.sample(num_samples=num_samples)
    )
    columns = [
        np.asarray(samples.time, dtype=float)[:, None],
        np.asarray(np.real(samples.position), dtype=float),
        np.asarray(np.real(samples.velocity), dtype=float),
        np.asarray(np.real(samples.acceleration), dtype=float),
        np.asarray(np.real(samples.jerk), dtype=float),
        np.asarray(np.real(samples.snap), dtype=float),
        np.asarray(np.real(samples.crackle), dtype=float),
    ]
    data = np.column_stack(columns)
    header = ",".join(
        ["time"]
        + [f"{prefix}{axis}" for prefix in ("p", "v", "a", "j", "s", "c") for axis in "xyz"]
    )
    output = _output_path(output_path)
    np.savetxt(output, data, delimiter=",", header=header, comments="", fmt="%.12g")
    return output


def export_dynamic_window_gif(
    track: SCWindowTrack,
    trajectory_or_result: TrajectoryInput,
    output_path: str | Path,
    *,
    num_frames: int = 72,
    trajectory_samples: int = 401,
    fps: float = 12.0,
    dpi: int = 90,
) -> Path:
    """Animate the trajectory together with every window's current pose."""

    if num_frames < 2 or trajectory_samples < 2:
        raise ValueError("num_frames and trajectory_samples must be at least two")
    if fps <= 0.0 or dpi < 30:
        raise ValueError("fps must be positive and dpi must be at least 30")
    trajectory = _trajectory(trajectory_or_result)
    total_time = float(np.real(trajectory.total_time))
    frame_times = np.linspace(0.0, total_time, num_frames)
    path_times = np.linspace(0.0, total_time, trajectory_samples)
    path_positions = np.asarray(np.real(trajectory.evaluate(path_times)), dtype=float)
    safe_polygons_by_frame = [
        [window.polygon_at(float(time)) for window in track.windows]
        for time in frame_times
    ]
    physical_boundaries_by_frame = [
        [window.physical_boundary_at(float(time)) for window in track.windows]
        for time in frame_times
    ]
    limit_points: list[FloatArray] = [path_positions]
    limit_points.extend(polygon for frame in safe_polygons_by_frame for polygon in frame)
    limit_points.extend(
        boundary
        for frame in physical_boundaries_by_frame
        for boundary in frame
        if boundary is not None
    )
    lower, upper = _axis_limits(limit_points)

    frames: list[NDArray[np.uint8]] = []
    colors = plt.cm.viridis(np.linspace(0.12, 0.88, max(1, len(track.windows))))
    for frame_index, time in enumerate(frame_times):
        figure = plt.figure(figsize=(6.4, 5.2), constrained_layout=True)
        try:
            axis = figure.add_subplot(111, projection="3d")
            elapsed = int(np.searchsorted(path_times, time, side="right"))
            elapsed = max(1, min(elapsed, len(path_times)))
            axis.plot(
                path_positions[:, 0], path_positions[:, 1], path_positions[:, 2],
                color="0.80", linewidth=1.0,
            )
            axis.plot(
                path_positions[:elapsed, 0], path_positions[:elapsed, 1], path_positions[:elapsed, 2],
                color="#0072bd", linewidth=2.2,
            )
            current = np.asarray(np.real(trajectory.evaluate(float(time))), dtype=float)
            axis.scatter(*current, color="#ff7f0e", s=38, marker="o")
            axis.scatter(*track.start, color="#2ca02c", s=28, marker="o")
            axis.scatter(*track.goal, color="#d62728", s=34, marker="X")
            for window_index, (safe_polygon, physical_boundary) in enumerate(
                zip(
                    safe_polygons_by_frame[frame_index],
                    physical_boundaries_by_frame[frame_index],
                )
            ):
                if physical_boundary is not None:
                    _add_physical_boundary(axis, physical_boundary, label=None)
                _add_window_polygon(
                    axis,
                    safe_polygon,
                    color=matplotlib.colors.to_hex(colors[window_index]),
                    label=None,
                )
            _set_3d_limits(axis, lower, upper)
            axis.set_xlabel("x [m]")
            axis.set_ylabel("y [m]")
            axis.set_zlabel("z [m]")
            axis.set_title(f"{track.name}  |  t = {time:.2f} s")
            axis.view_init(elev=24.0, azim=-61.0)
            figure.canvas.draw()
            frames.append(np.asarray(figure.canvas.buffer_rgba(), dtype=np.uint8)[..., :3].copy())
        finally:
            plt.close(figure)

    output = _output_path(output_path)
    imageio.mimsave(output, frames, duration=1.0 / fps, loop=0)
    return output


__all__ = [
    "export_dynamic_window_gif",
    "export_trajectory_csv",
    "plot_preprocessing",
    "plot_trajectory",
]
