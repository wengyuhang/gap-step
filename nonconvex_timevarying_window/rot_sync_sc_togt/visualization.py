"""CSV, 3-D figure and GIF export for composite rotation-Sync trajectories."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

from .collision import (
    CollisionReport,
    body_rotations,
    cuboid_vertices,
    cuboid_window_collision,
)
from .optimizer import RotSyncOptimizationResult
from .scenarios import RotSyncScenario


_CUBOID_FACES = (
    (0, 1, 3, 2), (4, 5, 7, 6),
    (0, 1, 5, 4), (2, 3, 7, 6),
    (0, 2, 6, 4), (1, 3, 7, 5),
)
_BOX_SIGNS = np.asarray(
    [
        (-1.0, -1.0, -1.0),
        (-1.0, -1.0, 1.0),
        (-1.0, 1.0, -1.0),
        (-1.0, 1.0, 1.0),
        (1.0, -1.0, -1.0),
        (1.0, -1.0, 1.0),
        (1.0, 1.0, -1.0),
        (1.0, 1.0, 1.0),
    ],
    dtype=float,
)


def _closed(vertices: np.ndarray) -> np.ndarray:
    return np.vstack((vertices, vertices[0]))


def _segment_index(trajectory, instant: float) -> int:
    cumulative = np.cumsum(trajectory.durations)
    return min(int(np.searchsorted(cumulative, instant, side="right")), trajectory.num_segments - 1)


def export_trajectory_csv(
    scenario: RotSyncScenario,
    result: RotSyncOptimizationResult,
    output_path: str | Path,
    *,
    samples: int = 601,
) -> Path:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    sample = result.forward.trajectory.sample(num_samples=samples)
    fields = [
        "time", "segment_type", "x", "y", "z", "vx", "vy", "vz",
        "ax", "ay", "az", "jx", "jy", "jz", "snap_x", "snap_y", "snap_z",
        "cuboid_collision",
    ]
    rotations = body_rotations(result.forward.trajectory, sample.time)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, instant in enumerate(sample.time):
            segment = _segment_index(result.forward.trajectory, float(instant))
            values = np.r_[
                sample.position[index], sample.velocity[index], sample.acceleration[index],
                sample.jerk[index], sample.snap[index],
            ]
            collision = any(
                cuboid_window_collision(
                    window,
                    float(instant),
                    sample.position[index],
                    rotations[index],
                    scenario.body,
                )[0]
                for window in scenario.windows
            )
            writer.writerow(
                {
                    "time": float(instant),
                    "segment_type": result.forward.trajectory.segment_kinds[segment],
                    **dict(zip(fields[2:-1], (float(value) for value in values))),
                    "cuboid_collision": collision,
                }
            )
    return path


def _limits(scenario: RotSyncScenario, result: RotSyncOptimizationResult) -> tuple[np.ndarray, np.ndarray]:
    points = result.forward.trajectory.sample(num_samples=301).position
    boundary_points = []
    for index, window in enumerate(scenario.windows):
        boundary_points.append(window.boundary_at(result.forward.crossing_times[index]))
    all_points = np.vstack((points, *boundary_points))
    low, high = np.min(all_points, axis=0), np.max(all_points, axis=0)
    padding = np.maximum(0.8, 0.08 * (high - low + 1.0))
    return low - padding, high + padding


def _draw_window(ax, window, instant: float, *, alpha: float = 0.75) -> None:
    front = _closed(window.boundary_at(instant, z=-0.5 * window.thickness))
    back = _closed(window.boundary_at(instant, z=0.5 * window.thickness))
    ax.plot(front[:, 0], front[:, 1], front[:, 2], color="#d95f02", lw=1.4, alpha=alpha)
    ax.plot(back[:, 0], back[:, 1], back[:, 2], color="#d95f02", lw=1.4, alpha=alpha)
    stride = max(1, len(front) // 12)
    for index in range(0, len(front) - 1, stride):
        ax.plot(
            (front[index, 0], back[index, 0]),
            (front[index, 1], back[index, 1]),
            (front[index, 2], back[index, 2]),
            color="#d95f02", lw=0.6, alpha=0.45,
        )


def _draw_cuboid(
    ax,
    position: np.ndarray,
    rotation: np.ndarray,
    scenario: RotSyncScenario,
    *,
    collision: bool = False,
    alpha: float = 0.72,
) -> None:
    """Draw the collision cuboid plus an attitude-aligned X quadrotor."""

    vertices = cuboid_vertices(position, rotation, scenario.body)
    faces = [[vertices[index] for index in face] for face in _CUBOID_FACES]
    color = "#d73027" if collision else "#2a9d8f"
    ax.add_collection3d(
        Poly3DCollection(
            faces,
            facecolor=color,
            edgecolor=color,
            linewidth=0.9,
            alpha=0.34 if collision else 0.13,
        )
    )

    half = np.asarray(scenario.body.half_extents, dtype=float)
    body_half = np.asarray((0.32 * half[0], 0.32 * half[1], 0.55 * half[2]))
    body_vertices = position + (rotation @ (_BOX_SIGNS * body_half).T).T
    body_faces = [[body_vertices[index] for index in face] for face in _CUBOID_FACES]
    ax.add_collection3d(
        Poly3DCollection(
            body_faces,
            facecolor="#b2182b" if collision else "#263238",
            edgecolor="#111111",
            linewidth=0.55,
            alpha=alpha,
        )
    )

    motor_offset = 0.56 * min(half[0], half[1])
    propeller_radius = 0.38 * min(half[0], half[1])
    motors_local = np.asarray(
        [
            (motor_offset, motor_offset, 0.0),
            (motor_offset, -motor_offset, 0.0),
            (-motor_offset, motor_offset, 0.0),
            (-motor_offset, -motor_offset, 0.0),
        ]
    )
    motors = position + (rotation @ motors_local.T).T
    arm_color = "#b2182b" if collision else "#455a64"
    rotor_color = "#ef8a62" if collision else "#00acc1"
    for motor_local, motor in zip(motors_local, motors):
        ax.plot(
            (position[0], motor[0]),
            (position[1], motor[1]),
            (position[2], motor[2]),
            color=arm_color,
            lw=2.4,
            solid_capstyle="round",
        )
        angles = np.linspace(0.0, 2.0 * np.pi, 37)
        rotor_local = motor_local + np.column_stack(
            (
                propeller_radius * np.cos(angles),
                propeller_radius * np.sin(angles),
                np.zeros_like(angles),
            )
        )
        rotor = position + (rotation @ rotor_local.T).T
        ax.plot(*rotor.T, color=rotor_color, lw=1.15, alpha=0.92)
        ax.scatter(*motor, color="#111111", s=8)

    nose = position + rotation @ np.asarray((0.82 * half[0], 0.0, 0.20 * half[2]))
    ax.plot(
        (position[0], nose[0]),
        (position[1], nose[1]),
        (position[2], nose[2]),
        color="#fdd835",
        lw=2.2,
    )


def plot_trajectory(
    scenario: RotSyncScenario,
    result: RotSyncOptimizationResult,
    output_path: str | Path,
    *,
    collision_report: CollisionReport | None = None,
) -> Path:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure = plt.figure(figsize=(10.5, 8.0), constrained_layout=True)
    ax = figure.add_subplot(111, projection="3d")
    trajectory = result.forward.trajectory
    sampled = trajectory.sample(num_samples=801)
    ax.plot(*sampled.position.T, color="#2166ac", lw=2.0, label="complete trajectory")
    cumulative = np.r_[0.0, np.cumsum(trajectory.durations)]
    for segment, kind in enumerate(trajectory.segment_kinds):
        if kind != "sync":
            continue
        times = np.linspace(cumulative[segment], cumulative[segment + 1], 80)
        positions = trajectory.evaluate(times)
        ax.plot(*positions.T, color="#fdae61", lw=4.0, label="analytic Sync" if segment == 1 else None)
        ax.scatter(*positions[0], color="#fdae61", s=28, marker=">")
        ax.scatter(*positions[-1], color="#fdae61", s=28, marker=">")
    crossing_rotations = body_rotations(trajectory, result.forward.crossing_times)
    for index, window in enumerate(scenario.windows):
        crossing = float(result.forward.crossing_times[index])
        _draw_window(ax, window, crossing)
        point = trajectory.evaluate(crossing)
        collides = any(
            cuboid_window_collision(
                candidate,
                crossing,
                point,
                crossing_rotations[index],
                scenario.body,
            )[0]
            for candidate in scenario.windows
        )
        _draw_cuboid(
            ax,
            point,
            crossing_rotations[index],
            scenario,
            collision=collides,
        )
        ax.text(*window.center, f" {index + 1}:{window.name}", fontsize=9)
    start = scenario.start_state.position
    ax.scatter(*start, color="#1a9850", s=58, marker="o", label="start / finish" if scenario.closed else "start")
    if not scenario.closed:
        ax.scatter(*scenario.goal_state.position, color="#762a83", s=58, marker="X", label="finish")
    low, high = _limits(scenario, result)
    ax.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]), zlim=(low[2], high[2]))
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    collision_text = (
        ""
        if collision_report is None
        else f" | collision={collision_report.sampled_collision_rate:.3%}"
    )
    full_size = 2.0 * np.asarray(scenario.body.half_extents)
    ax.set_title(
        f"{scenario.name}\n"
        f"MINCO / analytic Sync | T={result.total_time:.3f}s | "
        f"vmax={result.extrema['max_velocity']:.3f}m/s | "
        f"amax={result.max_acceleration:.3f}m/s²\n"
        f"cuboid={full_size[0]:.3f}×{full_size[1]:.3f}×{full_size[2]:.3f} m"
        f"{collision_text}",
        fontsize=11,
    )
    handles, labels = ax.get_legend_handles_labels()
    handles.append(
        Patch(
            facecolor="#2a9d8f",
            edgecolor="#263238",
            alpha=0.35,
            label="quadrotor + collision cuboid",
        )
    )
    labels.append("quadrotor + collision cuboid")
    ax.legend(handles, labels, loc="upper left")
    ax.view_init(elev=24, azim=-58)
    figure.savefig(path, dpi=180)
    plt.close(figure)
    return path


def plot_sync_closeups(
    scenario: RotSyncScenario,
    result: RotSyncOptimizationResult,
    output_path: str | Path,
) -> Path:
    """Export one true-scale close view for every analytic Sync crossing."""

    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure = plt.figure(figsize=(14.0, 10.0), constrained_layout=True)
    trajectory = result.forward.trajectory
    rotations = body_rotations(trajectory, result.forward.crossing_times)
    for index, (window, sync) in enumerate(zip(scenario.windows, trajectory.sync_segments)):
        ax = figure.add_subplot(2, 3, index + 1, projection="3d")
        crossing = float(result.forward.crossing_times[index])
        context_times = np.linspace(
            max(0.0, sync.entry_time - 0.75),
            min(trajectory.total_time, sync.entry_time + sync.duration + 0.75),
            120,
        )
        context = trajectory.evaluate(context_times)
        sync_times = np.linspace(sync.entry_time, sync.entry_time + sync.duration, 60)
        sync_positions = trajectory.evaluate(sync_times)
        ax.plot(*context.T, color="#2166ac", lw=2.0)
        ax.plot(*sync_positions.T, color="#fdae61", lw=4.2)
        _draw_window(ax, window, crossing, alpha=0.90)
        point = trajectory.evaluate(crossing)
        _draw_cuboid(ax, point, rotations[index], scenario, alpha=0.92)
        span = float(np.max(np.ptp(window.physical_polygon, axis=0)))
        radius = max(2.15, 0.5 * span + 0.70)
        center = window.center
        ax.set(
            xlim=(center[0] - radius, center[0] + radius),
            ylim=(center[1] - radius, center[1] + radius),
            zlim=(center[2] - radius, center[2] + radius),
        )
        ax.set_box_aspect((1.0, 1.0, 1.0))
        if hasattr(sync, "local_entry_point"):
            local_label = (
                f"q_in=({sync.local_entry_point[0]:.3f}, {sync.local_entry_point[1]:.3f}), "
                f"q_out=({sync.local_exit_point[0]:.3f}, {sync.local_exit_point[1]:.3f}) m"
            )
        else:
            local_label = f"q=({sync.local_point[0]:.3f}, {sync.local_point[1]:.3f}) m"
        ax.set_title(f"{index + 1}: {window.name}  Δ={sync.duration:.3f}s\n{local_label}")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        ax.view_init(elev=22, azim=-55)
    figure.suptitle(
        "Analytic rotation-synchronised crossings (true physical scale)\n"
        "orange: Sync, cyan rotors + translucent box: collision body",
        fontsize=14,
    )
    figure.savefig(path, dpi=180)
    plt.close(figure)
    return path


def export_animation(
    scenario: RotSyncScenario,
    result: RotSyncOptimizationResult,
    output_path: str | Path,
    *,
    frames: int = 100,
    fps: int = 16,
    collision_report: CollisionReport | None = None,
) -> Path:
    if frames < 12 or fps < 1:
        raise ValueError("animation requires at least 12 frames and positive fps")
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    trajectory = result.forward.trajectory
    # A Sync interval can be much shorter than the whole lap.  Reserve frames
    # inside every Sync so the animation visibly demonstrates frame locking.
    uniform_count = max(12, int(round(0.65 * frames)))
    reserved = max(8, (frames - uniform_count) // max(1, len(trajectory.sync_segments)))
    frame_chunks = [np.linspace(0.0, trajectory.total_time, uniform_count)]
    for sync in trajectory.sync_segments:
        frame_chunks.append(
            np.linspace(sync.entry_time, sync.entry_time + sync.duration, reserved)
        )
    frame_times = np.unique(np.concatenate(frame_chunks))
    cumulative = np.r_[0.0, np.cumsum(trajectory.durations)]
    trace_times = np.linspace(0.0, trajectory.total_time, 700)
    trace = trajectory.evaluate(trace_times)
    frame_positions = trajectory.evaluate(frame_times)
    frame_rotations = body_rotations(trajectory, frame_times)
    frame_collisions = np.asarray(
        [
            any(
                cuboid_window_collision(
                    window,
                    float(instant),
                    frame_positions[index],
                    frame_rotations[index],
                    scenario.body,
                )[0]
                for window in scenario.windows
            )
            for index, instant in enumerate(frame_times)
        ],
        dtype=bool,
    )
    figure = plt.figure(figsize=(9.6, 7.2), constrained_layout=True)

    def update(frame: int):
        figure.clear()
        ax = figure.add_subplot(111, projection="3d")
        instant = float(frame_times[frame])
        past = trace_times <= instant
        ax.plot(*trace.T, color="#9ecae1", lw=1.0, alpha=0.55)
        ax.plot(*trace[past].T, color="#2166ac", lw=2.4)
        for window in scenario.windows:
            _draw_window(ax, window, instant)
        position = frame_positions[frame]
        _draw_cuboid(
            ax,
            position,
            frame_rotations[frame],
            scenario,
            collision=bool(frame_collisions[frame]),
            alpha=0.88,
        )
        # Follow the vehicle locally: an overview spanning the 90+ m lap makes
        # a true-scale 0.60 m quadrotor unreadably small during a short Sync.
        local_radius = 3.6
        vertical_radius = 3.0
        ax.set(
            xlim=(position[0] - local_radius, position[0] + local_radius),
            ylim=(position[1] - local_radius, position[1] + local_radius),
            zlim=(position[2] - vertical_radius, position[2] + vertical_radius),
        )
        ax.set_box_aspect((1.0, 1.0, 0.82))
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        segment = _segment_index(trajectory, instant)
        if trajectory.segment_kinds[segment] == "sync":
            sync_times = np.linspace(cumulative[segment], instant, 32)
            sync_trace = trajectory.evaluate(sync_times)
            ax.plot(*sync_trace.T, color="#fdae61", lw=4.4)
            ax.scatter(*sync_trace[0], color="#fdae61", s=25, marker=">")
        state = "COLLISION" if frame_collisions[frame] else "clear"
        rate = "" if collision_report is None else f"  sampled rate={collision_report.sampled_collision_rate:.3%}"
        ax.set_title(
            f"{scenario.name}  t={instant:.2f}s / {trajectory.total_time:.2f}s  "
            f"segment={trajectory.segment_kinds[segment]}  cuboid={state}{rate}"
        )
        ax.view_init(elev=24, azim=-58)
        return ()

    animation = FuncAnimation(figure, update, frames=len(frame_times), interval=1000 / fps, blit=False)
    animation.save(path, writer=PillowWriter(fps=fps))
    plt.close(figure)
    return path


__all__ = [
    "export_animation",
    "export_trajectory_csv",
    "plot_sync_closeups",
    "plot_trajectory",
]
