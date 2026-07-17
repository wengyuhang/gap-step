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


_WINDOW_FRAME_COLOR = "#f28e2b"
_WINDOW_FRAME_SHADOW = "#343a40"
_TRAJECTORY_COLOR = "#0072bd"


def _display_boundary(window: object, time: float) -> FloatArray:
    """Return real aperture geometry, falling back only for legacy windows."""

    physical = window.physical_boundary_at(time)  # type: ignore[attr-defined]
    if physical is not None:
        return np.asarray(physical, dtype=float)
    return np.asarray(window.polygon_at(time), dtype=float)  # type: ignore[attr-defined]


def _add_window_frame(
    axis: object,
    boundary: FloatArray,
    *,
    label: str | None,
    annotation: str | None = None,
) -> None:
    """Draw one physical aperture as a common two-tone tubular frame."""

    closed = _closed(boundary)
    # A dark under-stroke and a narrower orange highlight read as a solid
    # racing-gate tube while keeping the aperture itself fully transparent.
    axis.plot(  # type: ignore[attr-defined]
        closed[:, 0],
        closed[:, 1],
        closed[:, 2],
        color=_WINDOW_FRAME_SHADOW,
        linewidth=5.6,
        solid_capstyle="round",
        solid_joinstyle="round",
        alpha=0.98,
    )
    axis.plot(  # type: ignore[attr-defined]
        closed[:, 0],
        closed[:, 1],
        closed[:, 2],
        color=_WINDOW_FRAME_COLOR,
        linewidth=3.2,
        solid_capstyle="round",
        solid_joinstyle="round",
        label=label,
    )
    if annotation:
        center = np.mean(boundary, axis=0)
        axis.text(  # type: ignore[attr-defined]
            float(center[0]),
            float(center[1]),
            float(center[2]),
            annotation,
            color="white",
            fontsize=7.2,
            fontweight="bold",
            horizontalalignment="center",
            verticalalignment="center",
            bbox={
                "boxstyle": "circle,pad=0.22",
                "facecolor": _WINDOW_FRAME_SHADOW,
                "edgecolor": _WINDOW_FRAME_COLOR,
                "linewidth": 0.8,
                "alpha": 0.92,
            },
        )


def _quadrotor_basis(velocity: ArrayLike, acceleration: ArrayLike) -> FloatArray:
    """Build a physically motivated body frame from flat-output derivatives."""

    velocity_array = np.asarray(velocity, dtype=float)
    acceleration_array = np.asarray(acceleration, dtype=float)
    if velocity_array.shape != (3,) or acceleration_array.shape != (3,):
        raise ValueError("velocity and acceleration must have shape (3,)")

    thrust = acceleration_array + np.asarray((0.0, 0.0, 9.81))
    thrust_norm = float(np.linalg.norm(thrust))
    body_z = thrust / thrust_norm if thrust_norm > 1.0e-9 else np.asarray((0.0, 0.0, 1.0))
    heading = np.asarray((velocity_array[0], velocity_array[1], 0.0))
    if float(np.linalg.norm(heading)) <= 1.0e-9:
        heading = np.asarray((1.0, 0.0, 0.0))
    else:
        heading = heading / np.linalg.norm(heading)
    body_y = np.cross(body_z, heading)
    if float(np.linalg.norm(body_y)) <= 1.0e-9:
        axes = np.eye(3)
        fallback = axes[int(np.argmin(np.abs(axes @ body_z)))]
        body_y = np.cross(body_z, fallback)
    body_y = body_y / np.linalg.norm(body_y)
    body_x = np.cross(body_y, body_z)
    body_x = body_x / np.linalg.norm(body_x)
    return np.column_stack((body_x, body_y, body_z))


def _drone_arm_length(boundaries: Sequence[FloatArray]) -> float:
    """Choose a visible but metre-scale drone size from the gate geometry."""

    diagonals = [
        float(np.linalg.norm(np.ptp(np.asarray(boundary, dtype=float), axis=0)))
        for boundary in boundaries
        if len(boundary)
    ]
    reference = float(np.median(diagonals)) if diagonals else 3.0
    return float(np.clip(0.13 * reference, 0.38, 0.72))


def _add_quadrotor(
    axis: object,
    position: ArrayLike,
    velocity: ArrayLike,
    acceleration: ArrayLike,
    *,
    arm_length: float,
    label: str | None = None,
) -> None:
    """Draw a compact X-frame quadrotor with four translucent rotor disks."""

    if arm_length <= 0.0:
        raise ValueError("arm_length must be positive")
    center = np.asarray(position, dtype=float)
    if center.shape != (3,):
        raise ValueError("position must have shape (3,)")
    basis = _quadrotor_basis(velocity, acceleration)
    body_x, body_y = basis[:, 0], basis[:, 1]
    diagonal = arm_length / np.sqrt(2.0)
    offsets = np.asarray(
        ((diagonal, diagonal), (diagonal, -diagonal), (-diagonal, -diagonal), (-diagonal, diagonal))
    )
    rotor_centers = center[None, :] + offsets[:, :1] * body_x + offsets[:, 1:] * body_y

    for rotor_index in ((0, 2), (1, 3)):
        endpoints = rotor_centers[list(rotor_index)]
        axis.plot(  # type: ignore[attr-defined]
            endpoints[:, 0], endpoints[:, 1], endpoints[:, 2],
            color="#20262e", linewidth=4.2, solid_capstyle="round",
            label=label if rotor_index == (0, 2) else None,
        )
        axis.plot(  # type: ignore[attr-defined]
            endpoints[:, 0], endpoints[:, 1], endpoints[:, 2],
            color="#adb5bd", linewidth=1.2, solid_capstyle="round",
        )

    rotor_radius = 0.29 * arm_length
    theta = np.linspace(0.0, 2.0 * np.pi, 20)
    circle = np.column_stack((np.cos(theta), np.sin(theta)))
    for index, rotor_center in enumerate(rotor_centers):
        disk = (
            rotor_center[None, :]
            + rotor_radius * circle[:, :1] * body_x
            + rotor_radius * circle[:, 1:] * body_y
        )
        rotor_color = "#ffb703" if index < 2 else "#8ecae6"
        axis.add_collection3d(  # type: ignore[attr-defined]
            Poly3DCollection(
                [disk],
                facecolor=rotor_color,
                edgecolor="#20262e",
                linewidth=0.7,
                alpha=0.48,
            )
        )

    body_radius = 0.27 * arm_length
    body = np.asarray(
        [
            center + body_radius * body_x,
            center + 0.72 * body_radius * body_y,
            center - body_radius * body_x,
            center - 0.72 * body_radius * body_y,
        ]
    )
    axis.add_collection3d(  # type: ignore[attr-defined]
        Poly3DCollection(
            [body], facecolor="#1f2933", edgecolor="#f8f9fa", linewidth=0.6, alpha=1.0
        )
    )
    nose = np.vstack((center, center + 0.62 * arm_length * body_x))
    axis.plot(  # type: ignore[attr-defined]
        nose[:, 0], nose[:, 1], nose[:, 2], color="#e63946", linewidth=2.2
    )


def _add_ground_plane(axis: object, lower: FloatArray, upper: FloatArray) -> None:
    """Add a subtle z=0 reference plane when the scene lies above ground."""

    if lower[2] > 1.0e-9 or upper[2] < -1.0e-9:
        return
    ground = np.asarray(
        [
            (lower[0], lower[1], 0.0),
            (upper[0], lower[1], 0.0),
            (upper[0], upper[1], 0.0),
            (lower[0], upper[1], 0.0),
        ]
    )
    axis.add_collection3d(  # type: ignore[attr-defined]
        Poly3DCollection([ground], facecolor="#dfe7ec", edgecolor="none", alpha=0.20)
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
    axis.set_box_aspect(  # type: ignore[attr-defined]
        np.maximum(upper - lower, 1.0e-3), zoom=1.12
    )


def plot_trajectory(
    track: SCWindowTrack,
    trajectory_or_result: TrajectoryInput,
    output_path: str | Path,
    *,
    num_samples: int = 401,
    dpi: int = 160,
) -> Path:
    """Plot a 3D MINCO path, physical gate frames, and crossing drones.

    The inset polygons remain part of optimization and validation but are
    intentionally absent from this scene-level visualization.
    """

    if num_samples < 2:
        raise ValueError("num_samples must be at least two")
    output = _output_path(output_path)
    trajectory = _trajectory(trajectory_or_result)
    crossings = _crossing_times(trajectory_or_result, trajectory, len(track.order))
    samples = trajectory.sample(num_samples=num_samples)
    positions = np.asarray(np.real(samples.position), dtype=float)
    display_boundaries = [
        _display_boundary(
            track.windows[window_index], float(crossings[crossing_index])
        )
        for crossing_index, window_index in enumerate(track.order)
    ]
    lower, upper = _axis_limits([positions, *display_boundaries])
    if lower[2] > 0.0:
        lower[2] = 0.0

    many_gates = len(display_boundaries) > 3
    figure = plt.figure(figsize=(10.4 if many_gates else 8.0, 6.4), constrained_layout=True)
    try:
        axis = figure.add_subplot(111, projection="3d")
        axis.plot(
            positions[:, 0], positions[:, 1], positions[:, 2],
            color=_TRAJECTORY_COLOR, linewidth=2.1, label="MINCO trajectory",
        )
        if np.allclose(track.start, track.goal):
            axis.scatter(
                *track.start,
                facecolor="white",
                edgecolor="#2a9d8f",
                linewidth=1.8,
                s=58,
                marker="o",
                label="start / finish",
            )
            axis.text(*track.start, "  S/F", color="#1f6f65", fontsize=8, fontweight="bold")
        else:
            axis.scatter(*track.start, color="#2ca02c", s=42, marker="o", label="start")
            axis.scatter(*track.goal, color="#d62728", s=48, marker="X", label="goal")
        for index, boundary in enumerate(display_boundaries):
            _add_window_frame(
                axis,
                boundary,
                label="physical window frame" if index == 0 else None,
                annotation=str(index + 1),
            )
        crossing_samples = trajectory.sample(times=crossings)
        crossing_points = np.asarray(np.real(crossing_samples.position), dtype=float)
        arm_length = _drone_arm_length(display_boundaries)
        if len(crossing_points):
            crossing_velocities = np.asarray(np.real(crossing_samples.velocity), dtype=float)
            crossing_accelerations = np.asarray(
                np.real(crossing_samples.acceleration), dtype=float
            )
            for index, (point, velocity, acceleration) in enumerate(
                zip(crossing_points, crossing_velocities, crossing_accelerations)
            ):
                _add_quadrotor(
                    axis,
                    point,
                    velocity,
                    acceleration,
                    arm_length=arm_length,
                    label="quadrotor at crossing" if index == 0 else None,
                )
        _add_ground_plane(axis, lower, upper)
        _set_3d_limits(axis, lower, upper)
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.set_zlabel("z [m]")
        axis.set_title(
            f"Physical dynamic-window traversal ({len(track.order)} ordered windows)"
        )
        axis.set_facecolor("#f7f9fb")
        axis.grid(True, alpha=0.28, linewidth=0.5)
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


def crossing_scale_data(
    track: SCWindowTrack,
    trajectory_or_result: TrajectoryInput,
) -> tuple[FloatArray, FloatArray]:
    """Return designated crossing times and the corresponding gate scales."""

    trajectory = _trajectory(trajectory_or_result)
    times = _crossing_times(trajectory_or_result, trajectory, len(track.order))
    scales = np.asarray(
        [
            track.windows[window_index].motion.scale(float(times[index]))[0]
            for index, window_index in enumerate(track.order)
        ],
        dtype=float,
    )
    return times, scales


def scale_profile_data(
    track: SCWindowTrack,
    trajectory_or_result: TrajectoryInput,
    *,
    num_samples: int = 501,
) -> tuple[FloatArray, FloatArray]:
    """Return common time samples and one scale row per designated gate."""

    if num_samples < 2:
        raise ValueError("num_samples must be at least two")
    trajectory = _trajectory(trajectory_or_result)
    total_time = float(np.real(trajectory.total_time))
    times = np.linspace(0.0, total_time, num_samples)
    scales = np.vstack(
        [
            [track.windows[window_index].motion.scale(float(time))[0] for time in times]
            for window_index in track.order
        ]
    )
    return times, np.asarray(scales, dtype=float)


def plot_route_overview(
    track: SCWindowTrack,
    trajectory_or_result: TrajectoryInput,
    output_path: str | Path,
    *,
    num_samples: int = 401,
    representative_fraction: float = 0.48,
    dpi: int = 160,
) -> Path:
    """Create a low-clutter route figure with one representative quadrotor."""

    if num_samples < 2:
        raise ValueError("num_samples must be at least two")
    if not (0.0 <= representative_fraction <= 1.0):
        raise ValueError("representative_fraction must lie in [0, 1]")
    output = _output_path(output_path)
    trajectory = _trajectory(trajectory_or_result)
    crossings = _crossing_times(trajectory_or_result, trajectory, len(track.order))
    positions = np.asarray(
        np.real(trajectory.sample(num_samples=num_samples).position), dtype=float
    )
    boundaries = [
        _display_boundary(track.windows[window_index], float(crossings[index]))
        for index, window_index in enumerate(track.order)
    ]
    lower, upper = _axis_limits([positions, *boundaries])
    if lower[2] > 0.0:
        lower[2] = 0.0

    figure = plt.figure(figsize=(10.2, 6.2), constrained_layout=True)
    try:
        axis = figure.add_subplot(111, projection="3d")
        axis.plot(
            positions[:, 0], positions[:, 1], positions[:, 2],
            color=_TRAJECTORY_COLOR, linewidth=2.4,
        )
        if np.allclose(track.start, track.goal):
            axis.scatter(
                *track.start, facecolor="white", edgecolor="#2a9d8f",
                linewidth=1.8, s=64, marker="o",
            )
            axis.text(*track.start, "  START / FINISH", color="#1f6f65", fontsize=7.5)
        else:
            axis.scatter(*track.start, color="#2ca02c", s=42, marker="o")
            axis.scatter(*track.goal, color="#d62728", s=48, marker="X")
        for index, boundary in enumerate(boundaries):
            _add_window_frame(axis, boundary, label=None, annotation=str(index + 1))

        representative_time = representative_fraction * float(np.real(trajectory.total_time))
        representative = trajectory.sample(times=np.asarray((representative_time,)))
        _add_quadrotor(
            axis,
            np.asarray(np.real(representative.position[0]), dtype=float),
            np.asarray(np.real(representative.velocity[0]), dtype=float),
            np.asarray(np.real(representative.acceleration[0]), dtype=float),
            arm_length=_drone_arm_length(boundaries),
        )
        _add_ground_plane(axis, lower, upper)
        _set_3d_limits(axis, lower, upper)
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.set_zlabel("z [m]")
        axis.set_title(f"SC-DynaTOGT closed-loop route · {len(track.order)} ordered gates")
        axis.set_facecolor("#fbfcfd")
        axis.grid(True, color="#cbd5df", alpha=0.28, linewidth=0.45)
        axis.view_init(elev=24.0, azim=-61.0)
        figure.savefig(output, dpi=dpi, bbox_inches="tight")
    finally:
        plt.close(figure)
    return output


def _project_to_gate_plane(points: FloatArray, center: FloatArray, basis: FloatArray) -> FloatArray:
    return (np.asarray(points, dtype=float) - center[None, :]) @ basis


def plot_crossing_grid(
    track: SCWindowTrack,
    trajectory_or_result: TrajectoryInput,
    output_path: str | Path,
    *,
    columns: int = 3,
    dpi: int = 160,
) -> Path:
    """Plot fixed-world-scale front views of all designated crossings."""

    if columns < 1:
        raise ValueError("columns must be positive")
    output = _output_path(output_path)
    trajectory = _trajectory(trajectory_or_result)
    crossing_times, crossing_scales = crossing_scale_data(track, trajectory_or_result)
    samples = trajectory.sample(times=crossing_times)
    positions = np.asarray(np.real(samples.position), dtype=float)
    velocities = np.asarray(np.real(samples.velocity), dtype=float)
    accelerations = np.asarray(np.real(samples.acceleration), dtype=float)

    projections: list[tuple[FloatArray, FloatArray, FloatArray, FloatArray]] = []
    arm_length = _drone_arm_length(
        [
            _display_boundary(track.windows[window_index], float(crossing_times[index]))
            for index, window_index in enumerate(track.order)
        ]
    )
    maximum_extent = 0.0
    for index, window_index in enumerate(track.order):
        window = track.windows[window_index]
        center, basis, *_ = window.state_at(float(crossing_times[index]))
        boundary = _display_boundary(window, float(crossing_times[index]))
        boundary_local = _project_to_gate_plane(boundary, center, basis)
        body_basis = _quadrotor_basis(velocities[index], accelerations[index])
        diagonal = arm_length / np.sqrt(2.0)
        offsets = np.asarray(
            ((diagonal, diagonal), (diagonal, -diagonal), (-diagonal, -diagonal), (-diagonal, diagonal))
        )
        rotor_global = (
            positions[index][None, :]
            + offsets[:, :1] * body_basis[:, 0]
            + offsets[:, 1:] * body_basis[:, 1]
        )
        rotor_local = _project_to_gate_plane(rotor_global, center, basis)
        center_local = _project_to_gate_plane(positions[index][None, :], center, basis)[0]
        projections.append((boundary_local, rotor_local, center_local, center))
        maximum_extent = max(
            maximum_extent,
            float(np.max(np.abs(boundary_local))),
            float(np.max(np.abs(rotor_local))),
        )
    limit = max(1.0, 1.15 * maximum_extent)
    rows = int(np.ceil(len(track.order) / columns))
    figure, axes = plt.subplots(
        rows, columns, figsize=(4.1 * columns, 3.75 * rows), squeeze=False,
        constrained_layout=True,
    )
    try:
        for index, axis in enumerate(axes.flat):
            if index >= len(track.order):
                axis.set_visible(False)
                continue
            window_index = track.order[index]
            window = track.windows[window_index]
            boundary, rotors, center_local, _ = projections[index]
            closed = _closed(boundary)
            axis.plot(
                closed[:, 0], closed[:, 1], color=_WINDOW_FRAME_SHADOW,
                linewidth=7.0, solid_capstyle="round", solid_joinstyle="round",
            )
            axis.plot(
                closed[:, 0], closed[:, 1], color=_WINDOW_FRAME_COLOR,
                linewidth=4.1, solid_capstyle="round", solid_joinstyle="round",
            )
            for first, second in ((0, 2), (1, 3)):
                axis.plot(
                    rotors[[first, second], 0], rotors[[first, second], 1],
                    color="#20262e", linewidth=4.0, solid_capstyle="round",
                )
            axis.scatter(
                rotors[:, 0], rotors[:, 1], s=58,
                facecolors=("#ffb703", "#ffb703", "#8ecae6", "#8ecae6"),
                edgecolors="#20262e", linewidths=0.8, zorder=4,
            )
            axis.scatter(*center_local, s=20, color="#e63946", zorder=5)
            axis.set_xlim(-limit, limit)
            axis.set_ylim(-limit, limit)
            axis.set_aspect("equal", adjustable="box")
            axis.grid(True, color="#dce3e8", linewidth=0.5)
            axis.set_xlabel("gate-local x [m]")
            axis.set_ylabel("gate-local y [m]")
            axis.set_title(
                f"Gate {index + 1} · {window.name}\n"
                f"t = {crossing_times[index]:.2f} s   scale = {crossing_scales[index]:.3f}",
                fontsize=10,
            )
        figure.suptitle("Fixed-scale views at designated crossings", fontsize=14)
        figure.savefig(output, dpi=dpi, bbox_inches="tight")
    finally:
        plt.close(figure)
    return output


def plot_scale_profile(
    track: SCWindowTrack,
    trajectory_or_result: TrajectoryInput,
    output_path: str | Path,
    *,
    num_samples: int = 501,
    dpi: int = 160,
) -> Path:
    """Plot every gate scale and highlight its designated crossing."""

    output = _output_path(output_path)
    trajectory = _trajectory(trajectory_or_result)
    crossing_times, crossing_scales = crossing_scale_data(track, trajectory_or_result)
    times, profile_scales = scale_profile_data(
        track, trajectory_or_result, num_samples=num_samples
    )
    total_time = float(np.real(trajectory.total_time))
    colors = plt.get_cmap("tab10")(np.linspace(0.0, 0.8, len(track.order)))
    figure, axis = plt.subplots(figsize=(10.4, 5.0), constrained_layout=True)
    try:
        for index, window_index in enumerate(track.order):
            window = track.windows[window_index]
            scales = profile_scales[index]
            axis.plot(times, scales, color=colors[index], linewidth=1.8, label=f"{index + 1} {window.name}")
            axis.scatter(
                [crossing_times[index]], [crossing_scales[index]],
                color=colors[index], edgecolor="white", linewidth=0.8, s=46, zorder=4,
            )
            axis.axvline(crossing_times[index], color=colors[index], alpha=0.14, linewidth=0.8)
        lower, upper = float(profile_scales.min()), float(profile_scales.max())
        padding = max(0.04, 0.08 * (upper - lower))
        axis.set_ylim(lower - padding, upper + padding)
        axis.set_xlim(0.0, total_time)
        axis.axhline(1.0, color="#495057", linewidth=0.9, linestyle="--", alpha=0.65)
        axis.set_xlabel("global trajectory time [s]")
        axis.set_ylabel("uniform gate scale s(t)")
        axis.set_title("Dynamic-window scale profiles and designated crossings")
        axis.grid(True, color="#dce3e8", linewidth=0.5)
        axis.legend(ncol=3, fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.16))
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
    """Animate a quadrotor and every physical window's current pose.

    Safe inset polygons are not rendered; they remain numerical planning
    geometry rather than physical objects in the scene.
    """

    if num_frames < 2 or trajectory_samples < 2:
        raise ValueError("num_frames and trajectory_samples must be at least two")
    if fps <= 0.0 or dpi < 30:
        raise ValueError("fps must be positive and dpi must be at least 30")
    trajectory = _trajectory(trajectory_or_result)
    total_time = float(np.real(trajectory.total_time))
    frame_times = np.linspace(0.0, total_time, num_frames)
    path_times = np.linspace(0.0, total_time, trajectory_samples)
    path_samples = trajectory.sample(times=path_times)
    path_positions = np.asarray(np.real(path_samples.position), dtype=float)
    display_boundaries_by_frame = [
        [_display_boundary(window, float(time)) for window in track.windows]
        for time in frame_times
    ]
    limit_points: list[FloatArray] = [path_positions]
    limit_points.extend(
        boundary for frame in display_boundaries_by_frame for boundary in frame
    )
    lower, upper = _axis_limits(limit_points)
    if lower[2] > 0.0:
        lower[2] = 0.0
    arm_length = _drone_arm_length(display_boundaries_by_frame[0])

    frames: list[NDArray[np.uint8]] = []
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
                color=_TRAJECTORY_COLOR, linewidth=2.2,
            )
            current = np.asarray(np.real(trajectory.evaluate(float(time))), dtype=float)
            velocity = np.asarray(
                np.real(trajectory.evaluate(float(time), derivative=1)), dtype=float
            )
            acceleration = np.asarray(
                np.real(trajectory.evaluate(float(time), derivative=2)), dtype=float
            )
            _add_quadrotor(
                axis,
                current,
                velocity,
                acceleration,
                arm_length=arm_length,
            )
            if np.allclose(track.start, track.goal):
                axis.scatter(
                    *track.start,
                    facecolor="white",
                    edgecolor="#2a9d8f",
                    linewidth=1.5,
                    s=38,
                    marker="o",
                )
                axis.text(*track.start, "  S/F", color="#1f6f65", fontsize=7, fontweight="bold")
            else:
                axis.scatter(*track.start, color="#2ca02c", s=28, marker="o")
                axis.scatter(*track.goal, color="#d62728", s=34, marker="X")
            for window_index, boundary in enumerate(
                display_boundaries_by_frame[frame_index]
            ):
                _add_window_frame(
                    axis,
                    boundary,
                    label=None,
                    annotation=str(window_index + 1),
                )
            _add_ground_plane(axis, lower, upper)
            _set_3d_limits(axis, lower, upper)
            axis.set_xlabel("x [m]")
            axis.set_ylabel("y [m]")
            axis.set_zlabel("z [m]")
            axis.set_title(
                f"Physical dynamic-window scene  |  t = {time:.2f} s"
            )
            axis.set_facecolor("#f7f9fb")
            axis.grid(True, alpha=0.28, linewidth=0.5)
            axis.view_init(elev=24.0, azim=-61.0)
            figure.canvas.draw()
            frames.append(np.asarray(figure.canvas.buffer_rgba(), dtype=np.uint8)[..., :3].copy())
        finally:
            plt.close(figure)

    output = _output_path(output_path)
    imageio.mimsave(output, frames, duration=1.0 / fps, loop=0)
    return output


__all__ = [
    "crossing_scale_data",
    "export_dynamic_window_gif",
    "export_trajectory_csv",
    "plot_crossing_grid",
    "plot_preprocessing",
    "plot_route_overview",
    "plot_scale_profile",
    "plot_trajectory",
    "scale_profile_data",
]
