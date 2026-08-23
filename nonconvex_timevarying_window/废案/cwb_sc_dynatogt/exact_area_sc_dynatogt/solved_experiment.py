"""Validation, export, and static figures for genuinely optimized Experiment B."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv

import matplotlib

matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np
from matplotlib.patches import Polygon as PolygonPatch  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402
from shapely.geometry import MultiPoint, Point, Polygon

from nonconvex_timevarying_window.cwb_sc_dynatogt.body_model import CuboidBody
from nonconvex_timevarying_window.cwb_sc_dynatogt.config import WholeBodySafetyConfig
from nonconvex_timevarying_window.cwb_sc_dynatogt.gate_frame import frame_at
from nonconvex_timevarying_window.cwb_sc_dynatogt.plane_section import (
    cuboid_world_vertices,
    find_planned_crossing_interval,
    gate_local_vertex_coordinates,
    plane_section_at,
)
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.optimizer import ForwardPass, OptimizationConfig

from .stress_case import WORLD_CLEARANCE


_OLD = "#d55e00"
_OURS = "#0072b2"
_FRAME = "#777777"
_COLLISION_EPSILON = 1.0e-6

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


@dataclass(frozen=True)
class ValidationSample:
    time: float
    center_clearance: float
    section_area: float
    intersection_area: float
    outside_area: float
    collision: bool


@dataclass(frozen=True)
class ValidationReport:
    samples: tuple[ValidationSample, ...]
    contact_start: float
    contact_end: float
    minimum_center_clearance: float
    maximum_outside_area: float
    collision: bool
    first_collision_time: float | None


def validate_forward(
    forward: ForwardPass,
    track: SCWindowTrack,
    body: CuboidBody,
    optimization: OptimizationConfig,
    *,
    sample_count: int = 15001,
) -> ValidationReport:
    """Apply one independent dense physical-aperture validator."""

    section_config = WholeBodySafetyConfig(
        half_extents=tuple(float(value) for value in body.half_extents),
        time_tolerance=2.0e-5,
        lambda_tolerance=2.0e-5,
        max_interval_depth=22,
    )
    window_index = track.order[0]
    window = track.windows[window_index]
    contact = find_planned_crossing_interval(
        window_index=window_index,
        traversal_time=float(forward.traversal_times[0]),
        trajectory=forward.trajectory,
        window=window,
        body=body,
        config=section_config,
        parameters=optimization.quadrotor,
    )
    values: list[ValidationSample] = []
    for instant in np.linspace(contact.start, contact.end, sample_count):
        time = float(instant)
        section = plane_section_at(
            forward.trajectory,
            time,
            window,
            body,
            section_config,
            parameters=optimization.quadrotor,
        )
        if len(section.vertices) < 3:
            continue
        gate = frame_at(window, time)
        body_polygon = Polygon(section.local_polygon * gate.scale)
        gate_polygon = Polygon(
            np.asarray(window.physical_boundary, dtype=float) * gate.scale
        )
        intersection = float(body_polygon.intersection(gate_polygon).area)
        outside = max(0.0, float(body_polygon.area) - intersection)
        position = np.asarray(forward.trajectory.evaluate(time), dtype=float)
        center_plane = (position - gate.center) @ gate.basis
        center_point = Point(float(center_plane[0]), float(center_plane[1]))
        unsigned = float(gate_polygon.boundary.distance(center_point))
        clearance = unsigned if gate_polygon.covers(center_point) else -unsigned
        collision = bool(
            intersection > _COLLISION_EPSILON
            and outside > _COLLISION_EPSILON
        )
        values.append(
            ValidationSample(
                time,
                clearance,
                float(body_polygon.area),
                intersection,
                outside,
                collision,
            )
        )
    if not values:
        raise RuntimeError("dense validator found no non-degenerate contact samples")
    collisions = [sample for sample in values if sample.collision]
    return ValidationReport(
        tuple(values),
        float(contact.start),
        float(contact.end),
        min(sample.center_clearance for sample in values),
        max(sample.outside_area for sample in values),
        bool(collisions),
        collisions[0].time if collisions else None,
    )


def export_trajectory_csv(
    path: Path,
    forwards: dict[str, ForwardPass],
    track: SCWindowTrack,
    body: CuboidBody,
    optimization: OptimizationConfig,
    *,
    samples: int = 1201,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []
    window = track.windows[track.order[0]]
    section_config = WholeBodySafetyConfig(
        half_extents=tuple(float(value) for value in body.half_extents)
    )
    for method, forward in forwards.items():
        total = float(np.sum(forward.durations))
        for instant in np.linspace(0.0, total, samples):
            time = float(instant)
            position = np.asarray(forward.trajectory.evaluate(time), dtype=float)
            velocity = np.asarray(forward.trajectory.evaluate(time, 1), dtype=float)
            gate = frame_at(window, time)
            section = plane_section_at(
                forward.trajectory, time, window, body, section_config,
                parameters=optimization.quadrotor,
            )
            outside = 0.0
            clearance = float("nan")
            if len(section.vertices) >= 3:
                body_polygon = Polygon(section.local_polygon * gate.scale)
                gate_polygon = Polygon(
                    np.asarray(window.physical_boundary) * gate.scale
                )
                outside = max(
                    0.0,
                    float(body_polygon.area)
                    - float(body_polygon.intersection(gate_polygon).area),
                )
                local = (position - gate.center) @ gate.basis
                point = Point(float(local[0]), float(local[1]))
                unsigned = float(gate_polygon.boundary.distance(point))
                clearance = unsigned if gate_polygon.covers(point) else -unsigned
            rows.append({
                "method": method,
                "time": time,
                "x": position[0], "y": position[1], "z": position[2],
                "vx": velocity[0], "vy": velocity[1], "vz": velocity[2],
                "window_x": gate.center[0], "window_y": gate.center[1],
                "window_z": gate.center[2], "window_scale": gate.scale,
                "center_clearance": clearance,
                "outside_area": outside,
            })
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _draw_cuboid(axis, vertices: np.ndarray, color: str) -> None:
    faces = (
        (0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4),
        (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5),
    )
    axis.add_collection3d(Poly3DCollection(
        [[vertices[index] for index in face] for face in faces],
        facecolor=color, edgecolor=color, linewidth=0.8, alpha=0.38,
    ))


def _draw_3d(
    axis,
    forward: ForwardPass,
    track: SCWindowTrack,
    body: CuboidBody,
    optimization: OptimizationConfig,
    instant: float,
    color: str,
) -> None:
    window = track.windows[track.order[0]]
    gate = frame_at(window, instant)
    boundary = np.asarray(window.physical_boundary) * gate.scale
    boundary_world = gate.center + boundary @ gate.basis.T
    boundary_world = np.vstack((boundary_world, boundary_world[0]))
    axis.plot(*boundary_world.T, color="#222222", linewidth=5)
    axis.plot(*boundary_world.T, color=_FRAME, linewidth=2.7)
    times = np.linspace(0.0, float(np.sum(forward.durations)), 301)
    path = np.asarray(forward.trajectory.evaluate(times), dtype=float)
    axis.plot(path[:, 0], path[:, 1], path[:, 2], color=color, linewidth=2.4)
    vertices, _ = cuboid_world_vertices(
        forward.trajectory, instant, body, parameters=optimization.quadrotor
    )
    _draw_cuboid(axis, vertices, color)
    center = np.asarray(forward.trajectory.evaluate(instant), dtype=float)
    axis.scatter(*center, color="black", s=20)
    axis.scatter(*track.start, marker="*", s=70, color="#222222")
    combined = np.vstack((path, boundary_world))
    lower, upper = combined.min(axis=0), combined.max(axis=0)
    span = np.maximum(upper - lower, np.array([1.0, 1.0, 1.0]))
    middle = 0.5 * (lower + upper)
    radius = 0.56 * float(np.max(span))
    axis.set_xlim(middle[0] - radius, middle[0] + radius)
    axis.set_ylim(middle[1] - radius, middle[1] + radius)
    axis.set_zlim(max(-0.2, middle[2] - 0.55 * radius), middle[2] + 0.55 * radius)
    axis.set_box_aspect((1.2, 1.2, 0.7))
    axis.view_init(elev=24, azim=-58)
    axis.tick_params(labelsize=5, pad=0)


def _draw_side(
    axis,
    forward: ForwardPass,
    track: SCWindowTrack,
    body: CuboidBody,
    optimization: OptimizationConfig,
    instant: float,
    color: str,
) -> tuple[bool, float]:
    window = track.windows[track.order[0]]
    gate = frame_at(window, instant)
    vertices, _ = cuboid_world_vertices(
        forward.trajectory, instant, body, parameters=optimization.quadrotor
    )
    local3 = gate_local_vertex_coordinates(vertices, gate)
    # Convert normalized in-plane coordinate back to metric u; n is metric.
    projection = np.column_stack((local3[:, 0] * gate.scale, local3[:, 2]))
    hull = np.asarray(MultiPoint(projection).convex_hull.exterior.coords)
    axis.fill(hull[:, 0], hull[:, 1], color=color, alpha=0.30, edgecolor=color, linewidth=2)
    center = np.asarray(forward.trajectory.evaluate(instant), dtype=float)
    center_local = np.array([
        float((center - gate.center) @ gate.basis[:, 0]),
        float((center - gate.center) @ gate.normal),
    ])
    axis.scatter(*center_local, color="black", s=25, zorder=8)
    axis.axhline(0.0, color="#6baed6", linewidth=2)
    boundary_u = np.asarray(window.physical_boundary)[:, 0] * gate.scale
    axis.plot(
        [float(boundary_u.min()), float(boundary_u.max())], [0.0, 0.0],
        color=_FRAME, linewidth=7, alpha=0.75,
    )
    axis.plot(hull[:, 0], hull[:, 1], color=color, linewidth=1.5)
    section_config = WholeBodySafetyConfig(
        half_extents=tuple(float(value) for value in body.half_extents)
    )
    section = plane_section_at(
        forward.trajectory, instant, window, body, section_config,
        parameters=optimization.quadrotor,
    )
    outside = 0.0
    collision = False
    if len(section.vertices) >= 3:
        body_polygon = Polygon(section.local_polygon * gate.scale)
        physical = Polygon(np.asarray(window.physical_boundary) * gate.scale)
        intersection = float(body_polygon.intersection(physical).area)
        outside = max(0.0, float(body_polygon.area) - intersection)
        collision = bool(
            intersection > _COLLISION_EPSILON
            and outside > _COLLISION_EPSILON
        )
    if collision:
        axis.scatter(
            center_local[0], 0.0, marker="X", s=170, color="#cb181d",
            edgecolor="white", linewidth=1.0, zorder=12,
        )
        axis.text(
            0.03, 0.94, f"整机碰撞\nE={outside:.2e} m²",
            transform=axis.transAxes, va="top", color="#b30000",
            fontsize=9, fontweight="bold",
        )
    margin = 0.12
    axis.set_xlim(float(hull[:, 0].min()) - margin, float(hull[:, 0].max()) + margin)
    axis.set_ylim(float(hull[:, 1].min()) - margin, float(hull[:, 1].max()) + margin)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("u [m]", fontsize=7)
    axis.set_ylabel("n [m]", fontsize=7)
    axis.tick_params(labelsize=6)
    axis.grid(alpha=0.2)
    return collision, outside


def render_timeline(
    path: Path,
    forwards: dict[str, ForwardPass],
    reports: dict[str, ValidationReport],
    track: SCWindowTrack,
    body: CuboidBody,
    optimization: OptimizationConfig,
    *,
    dpi: int = 145,
) -> None:
    """Render method-specific optimizer times in synchronized 3-D/side rows."""

    path.parent.mkdir(parents=True, exist_ok=True)
    figure = plt.figure(figsize=(20.0, 13.6))
    for method_index, method in enumerate(("Old-0.315", "Ours")):
        forward, report = forwards[method], reports[method]
        total = float(np.sum(forward.durations))
        event = (
            report.first_collision_time
            if report.first_collision_time is not None
            else report.contact_start + 0.35 * (report.contact_end - report.contact_start)
        )
        times = (
            max(0.0, report.contact_start - 0.18),
            float(event),
            float(forward.traversal_times[0]),
            min(total, report.contact_end + 0.12),
            total,
        )
        stages = ("接近窗口", "首次碰撞" if method_index == 0 else "安全进入", "中心穿越", "离开窗口", "返回终点")
        color = _OLD if method_index == 0 else _OURS
        for column, (stage, instant) in enumerate(zip(stages, times)):
            scene = figure.add_subplot(4, 5, 2 * method_index * 5 + column + 1, projection="3d")
            side = figure.add_subplot(4, 5, (2 * method_index + 1) * 5 + column + 1)
            _draw_3d(scene, forward, track, body, optimization, instant, color)
            collision, outside = _draw_side(
                side, forward, track, body, optimization, instant, color
            )
            if collision:
                scene.text2D(
                    0.04, 0.90, f"当前整机碰撞  E={outside:.2e} m²",
                    transform=scene.transAxes, color="#b30000",
                    fontsize=9, fontweight="bold",
                )
            gate = frame_at(track.windows[track.order[0]], instant)
            shift = gate.center - track.windows[track.order[0]].center0
            yaw = np.degrees(np.arctan2(gate.basis[1, 0], gate.basis[0, 0]))
            scene.text2D(
                0.02, 0.02,
                f"门框 Δp=({shift[0]:+.2f},{shift[1]:+.2f},{shift[2]:+.2f}) m\n"
                f"scale={gate.scale:.3f}, angle={yaw:+.1f}°",
                transform=scene.transAxes, fontsize=7,
                bbox=dict(boxstyle="round", facecolor="white", alpha=0.88),
            )
            scene.set_title(f"{stage}\nt={instant:.3f} s", color=color, fontsize=10, fontweight="bold")
    labels = ((0.785, "旧方法\n3D", _OLD), (0.585, "旧方法\n完整机体侧视", _OLD),
              (0.370, "新方法\n3D", _OURS), (0.165, "新方法\n完整机体侧视", _OURS))
    for y, label, color in labels:
        figure.text(0.008, y, label, color=color, fontsize=12, fontweight="bold", va="center")
    figure.suptitle(
        "实际 MINCO/L-BFGS 求解结果：3D 与完整机体侧视\n"
        "Old 碰撞后的面板仅回放求解器输出轨迹；真实执行应在首碰时停止",
        fontsize=16, fontweight="bold", color="#6b1d1d", y=0.98,
    )
    figure.subplots_adjust(left=0.065, right=0.995, bottom=0.045, top=0.89, wspace=0.16, hspace=0.30)
    figure.savefig(path, dpi=dpi, facecolor="white")
    plt.close(figure)


__all__ = [
    "ValidationReport",
    "export_trajectory_csv",
    "render_timeline",
    "validate_forward",
]
