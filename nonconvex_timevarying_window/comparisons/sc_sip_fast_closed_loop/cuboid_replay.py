"""Replay the certified hardest comparison track with the full cuboid body."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import imageio.v2 as imageio
import matplotlib

matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon as PolygonPatch  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402
import numpy as np
from scipy.optimize import differential_evolution

from nonconvex_timevarying_window.sc_dynatogt.collision import (
    point_to_oriented_cuboid_distance_squared,
)
from nonconvex_timevarying_window.sip_dynatogt.constraints import point_flatness
from nonconvex_timevarying_window.sip_dynatogt.io import load_run

from .experiment import _sampled_clearance_profile


EDGES = (
    (0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
    (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7),
)
FACES = (
    (0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4),
    (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5),
)


def _locate(trajectory, time: float) -> tuple[int, float]:
    cumulative = np.concatenate(([0.0], np.cumsum(trajectory.durations)))
    segment = min(int(np.searchsorted(cumulative[1:], time, side="right")), len(trajectory.durations) - 1)
    tau = (time - cumulative[segment]) / trajectory.durations[segment]
    return segment, float(np.clip(tau, 0.0, 1.0))


def _flat(trajectory, config, time: float):
    return point_flatness(trajectory, *_locate(trajectory, time), config)


def cuboid_vertices(center: np.ndarray, rotation: np.ndarray, half_extents) -> np.ndarray:
    signs = np.asarray([
        [-1, -1, -1], [-1, -1, 1], [-1, 1, -1], [-1, 1, 1],
        [1, -1, -1], [1, -1, 1], [1, 1, -1], [1, 1, 1],
    ], dtype=float)
    local = signs * np.asarray(half_extents, dtype=float)
    return np.asarray(center) + local @ np.asarray(rotation).T


def cuboid_plane_section(vertices: np.ndarray, plane_center: np.ndarray, normal: np.ndarray) -> np.ndarray:
    signed = (vertices - plane_center) @ normal
    points: list[np.ndarray] = []
    for left, right in EDGES:
        a, b = vertices[left], vertices[right]
        da, db = float(signed[left]), float(signed[right])
        if abs(da) <= 1.0e-10:
            points.append(a)
        if da * db < 0.0:
            points.append(a + da / (da - db) * (b - a))
    if not points:
        return np.empty((0, 3))
    unique: list[np.ndarray] = []
    for point in points:
        if not any(np.linalg.norm(point - previous) <= 1.0e-9 for previous in unique):
            unique.append(point)
    return np.asarray(unique)


def _boundary_local(window, samples_per_segment: int = 100) -> np.ndarray:
    parts = []
    for index, segment in enumerate(window.boundary):
        parameter = np.linspace(0.0, 1.0, samples_per_segment, endpoint=index == len(window.boundary) - 1)
        parts.append(np.asarray([segment.evaluate(float(value)) for value in parameter]))
    return np.vstack(parts)


def _boundary_world(window, time: float, samples_per_segment: int = 80) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    center, basis, scale = window.state_at(time)
    local = _boundary_local(window, samples_per_segment)
    world = center + (basis @ np.column_stack((scale * local, np.zeros(len(local)))).T).T
    return world, center, basis, scale


def _draw_cuboid_3d(axis, vertices: np.ndarray, *, alpha: float = 0.30) -> None:
    faces = [[vertices[index] for index in face] for face in FACES]
    axis.add_collection3d(Poly3DCollection(faces, facecolor="#2878b5", edgecolor="#123f62", linewidth=0.65, alpha=alpha))


def _closest_point_on_cuboid(point: np.ndarray, center: np.ndarray, rotation: np.ndarray, half_extents) -> np.ndarray:
    local = rotation.T @ (np.asarray(point) - center)
    local = np.clip(local, -np.asarray(half_extents), np.asarray(half_extents))
    return center + rotation @ local


def _crossing_records(problem, trajectory, config, traversal_times: np.ndarray) -> list[dict]:
    records = []
    for route_index, (window_index, time) in enumerate(zip(problem.order, traversal_times)):
        flat = _flat(trajectory, config, float(time))
        boundary, center, basis, scale = _boundary_world(problem.windows[window_index], float(time), 240)
        distance = np.sqrt(point_to_oriented_cuboid_distance_squared(boundary, flat.position, flat.rotation, config.body))
        section = cuboid_plane_section(cuboid_vertices(flat.position, flat.rotation, config.body.half_extents), center, basis[:, 2])
        records.append({
            "route_index": route_index,
            "window_index": int(window_index),
            "window_name": problem.windows[window_index].name,
            "time": float(time),
            "position": flat.position,
            "rotation": flat.rotation,
            "minimum_sampled_boundary_distance": float(distance.min()),
            "section": section,
            "center": center,
            "basis": basis,
            "scale": float(scale),
        })
    return records


def plot_route(problem, trajectory, config, crossings: list[dict], output: Path) -> None:
    times = np.linspace(0.0, trajectory.total_time, 1001)
    positions = np.asarray([_flat(trajectory, config, float(time)).position for time in times])
    figure = plt.figure(figsize=(12.0, 8.0), constrained_layout=True)
    axis = figure.add_subplot(111, projection="3d")
    axis.plot(*positions.T, color="#1f77b4", linewidth=2.0, label="certified SIP trajectory")
    for record in crossings:
        window = problem.windows[record["window_index"]]
        boundary, _, _, _ = _boundary_world(window, record["time"], 120)
        closed = np.vstack((boundary, boundary[0]))
        axis.plot(*closed.T, color="#d95319", linewidth=1.4)
        vertices = cuboid_vertices(record["position"], record["rotation"], config.body.half_extents)
        _draw_cuboid_3d(axis, vertices, alpha=0.38)
        axis.text(*record["position"], f" {record['route_index'] + 1}:{record['window_name']}", fontsize=7)
    axis.scatter(*positions[0], s=50, color="#2ca02c", label="start / finish")
    axis.set_xlabel("x [m]"); axis.set_ylabel("y [m]"); axis.set_zlabel("z [m]")
    axis.set_title("Hardest comparison track: certified trajectory with oriented cuboid at every crossing")
    axis.view_init(elev=23, azim=-58)
    axis.legend(loc="upper left")
    figure.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(figure)


def plot_crossings(problem, config, crossings: list[dict], output: Path) -> None:
    figure, axes = plt.subplots(2, 3, figsize=(14.2, 8.8), constrained_layout=True)
    for axis, record in zip(axes.flat, crossings):
        window = problem.windows[record["window_index"]]
        local_boundary = record["scale"] * _boundary_local(window, 180)
        axis.plot(*np.vstack((local_boundary, local_boundary[0])).T, color="#c62828", linewidth=2.0)
        axis.fill(local_boundary[:, 0], local_boundary[:, 1], color="#ffccbc", alpha=0.42)
        section = record["section"]
        if len(section) >= 3:
            projected = (section - record["center"]) @ record["basis"][:, :2]
            centroid = projected.mean(axis=0)
            angle = np.arctan2(projected[:, 1] - centroid[1], projected[:, 0] - centroid[0])
            projected = projected[np.argsort(angle)]
            axis.add_patch(PolygonPatch(projected, closed=True, facecolor="#2878b5", edgecolor="#123f62", alpha=0.55, label="cuboid plane section"))
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, color="0.88", linewidth=0.55)
        axis.set_xlabel("gate local u [m]"); axis.set_ylabel("gate local v [m]")
        axis.set_title(
            f"{record['route_index'] + 1}. {record['window_name']}  t={record['time']:.3f}s\n"
            f"sampled boundary distance={record['minimum_sampled_boundary_distance']:.4f}m"
        )
    figure.suptitle(
        "Actual cuboid/plane sections inside the six moving non-convex windows\n"
        f"cuboid half-extents={config.body.half_extents} m; required net clearance={config.clearance:.3f} m",
        fontsize=14,
    )
    figure.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(figure)


def _refine_critical_clearance(problem, trajectory, config, profile: dict) -> dict:
    sampled_time = float(profile["minimum_time"])
    window_index = int(profile["minimum_window"])
    boundary_index = int(profile["minimum_boundary"])
    window = problem.windows[window_index]
    boundary = window.boundary[boundary_index]

    def objective(values: np.ndarray) -> float:
        time, parameter = map(float, values)
        flat = _flat(trajectory, config, time)
        center, basis, scale = window.state_at(time)
        point = center + basis @ np.r_[scale * boundary.evaluate(parameter), 0.0]
        return float(
            point_to_oriented_cuboid_distance_squared(
                point[None, :], flat.position, flat.rotation, config.body
            )[0]
        )

    result = differential_evolution(
        objective,
        [
            (max(0.0, sampled_time - 0.03), min(trajectory.total_time, sampled_time + 0.03)),
            (0.0, 1.0),
        ],
        seed=7,
        tol=1.0e-11,
        polish=True,
        workers=1,
        updating="immediate",
    )
    return {
        "time": float(result.x[0]),
        "window_index": window_index,
        "window_name": window.name,
        "boundary_index": boundary_index,
        "boundary_parameter": float(result.x[1]),
        "distance": float(np.sqrt(max(0.0, result.fun))),
        "sampled_seed_time": sampled_time,
        "sampled_seed_distance": float(profile["minimum"]),
        "num_times": int(profile["num_times"]),
        "boundary_samples": int(profile["boundary_samples"]),
    }


def plot_critical_clearance(problem, trajectory, config, critical: dict, output: Path) -> dict:
    time = float(critical["time"])
    window_index = int(critical["window_index"])
    boundary_index = int(critical["boundary_index"])
    flat = _flat(trajectory, config, time)
    window = problem.windows[window_index]
    boundary, center, basis, _ = _boundary_world(window, time, 700)
    distances = np.sqrt(
        point_to_oriented_cuboid_distance_squared(
            boundary, flat.position, flat.rotation, config.body
        )
    )
    nearest_index = int(np.argmin(distances))
    nearest_boundary = boundary[nearest_index]
    nearest_body = _closest_point_on_cuboid(
        nearest_boundary, flat.position, flat.rotation, config.body.half_extents
    )
    sampled_distance = float(distances[nearest_index])
    vertices = cuboid_vertices(flat.position, flat.rotation, config.body.half_extents)

    figure = plt.figure(figsize=(10.4, 7.2), constrained_layout=True)
    axis = figure.add_subplot(111, projection="3d")
    closed = np.vstack((boundary, boundary[0]))
    axis.plot(*closed.T, color="#d95319", linewidth=2.0, label=f"{window.name} boundary")
    _draw_cuboid_3d(axis, vertices, alpha=0.58)
    axis.plot(
        [nearest_body[0], nearest_boundary[0]],
        [nearest_body[1], nearest_boundary[1]],
        [nearest_body[2], nearest_boundary[2]],
        color="#c62828",
        linewidth=3.0,
        marker="o",
        markersize=4,
        label=f"nearest sampled gap = {sampled_distance:.5f} m",
    )
    extent = np.vstack((boundary, vertices))
    lower = extent.min(axis=0)
    upper = extent.max(axis=0)
    midpoint = 0.5 * (lower + upper)
    radius = max(0.85, 0.62 * float(np.max(upper - lower)))
    axis.set_xlim(midpoint[0] - radius, midpoint[0] + radius)
    axis.set_ylim(midpoint[1] - radius, midpoint[1] + radius)
    axis.set_zlim(midpoint[2] - radius, midpoint[2] + radius)
    axis.set_xlabel("x [m]"); axis.set_ylabel("y [m]"); axis.set_zlabel("z [m]")
    axis.set_title(
        f"Locally refined tightest event: {window.name}, t={time:.4f} s\n"
        f"required clearance={config.clearance:.3f} m; local refined estimate={critical['distance']:.5f} m"
    )
    axis.view_init(elev=24, azim=-52)
    axis.legend(loc="upper left")
    figure.savefig(output, dpi=190, bbox_inches="tight")
    plt.close(figure)
    return {
        "time": time,
        "window_index": window_index,
        "window_name": window.name,
        "boundary_index": boundary_index,
        "boundary_parameter": float(critical["boundary_parameter"]),
        "locally_refined_distance": float(critical["distance"]),
        "dense_render_distance": sampled_distance,
        "extra_over_required_clearance": float(critical["distance"] - config.clearance),
        "sampled_seed_time": float(critical["sampled_seed_time"]),
        "sampled_seed_distance": float(critical["sampled_seed_distance"]),
        "num_times": int(critical["num_times"]),
        "boundary_samples": int(critical["boundary_samples"]),
    }


def export_gif(problem, trajectory, config, output: Path, *, frames: int = 84) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.parent / "_cuboid_frames"
    temporary.mkdir(parents=True, exist_ok=True)
    paths = []
    try:
        for frame, time in enumerate(np.linspace(0.0, trajectory.total_time, frames)):
            flat = _flat(trajectory, config, float(time))
            history_times = np.linspace(0.0, float(time), max(2, frame + 1))
            history = np.asarray([_flat(trajectory, config, float(value)).position for value in history_times])
            figure = plt.figure(figsize=(9.2, 6.4), constrained_layout=True)
            axis = figure.add_subplot(111, projection="3d")
            for window in problem.windows:
                boundary, _, _, _ = _boundary_world(window, float(time), 70)
                closed = np.vstack((boundary, boundary[0]))
                axis.plot(*closed.T, color="#d95319", linewidth=1.0, alpha=0.82)
            axis.plot(*history.T, color="#1f77b4", linewidth=1.8)
            _draw_cuboid_3d(axis, cuboid_vertices(flat.position, flat.rotation, config.body.half_extents), alpha=0.72)
            axis.set_xlim(-16, 17); axis.set_ylim(-20, 17); axis.set_zlim(0, 16)
            axis.set_xlabel("x [m]"); axis.set_ylabel("y [m]"); axis.set_zlabel("z [m]")
            axis.set_title(f"Certified cuboid replay — t={time:.2f} / {trajectory.total_time:.2f} s")
            axis.view_init(elev=24, azim=-58)
            frame_path = temporary / f"frame_{frame:04d}.png"
            figure.savefig(frame_path, dpi=105, bbox_inches="tight")
            plt.close(figure)
            paths.append(frame_path)
        with imageio.get_writer(output, mode="I", duration=0.10, loop=0) as writer:
            for path in paths:
                writer.append_data(imageio.imread(path))
    finally:
        for path in paths:
            path.unlink(missing_ok=True)
        temporary.rmdir()


def run(run_directory: str | Path, output_directory: str | Path, *, make_gif: bool = True) -> dict:
    problem, config, trajectory, stored = load_run(run_directory)
    traversal_times = np.asarray(stored["traversal_times"], dtype=float)
    crossings = _crossing_records(problem, trajectory, config, traversal_times)
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    plot_route(problem, trajectory, config, crossings, output / "cuboid_route_overview.png")
    plot_crossings(problem, config, crossings, output / "cuboid_crossings.png")
    clearance_profile = _sampled_clearance_profile(
        problem, trajectory, config, num_times=5001, boundary_samples=101
    )
    refined_clearance = _refine_critical_clearance(
        problem, trajectory, config, clearance_profile
    )
    critical_clearance = plot_critical_clearance(
        problem, trajectory, config, refined_clearance, output / "cuboid_critical_clearance.png"
    )
    if make_gif:
        export_gif(problem, trajectory, config, output / "cuboid_dynamic_replay.gif")
    summary = {
        "scenario": problem.name,
        "method": "SIP-DynaTOGT",
        "certificate_status": stored["status"],
        "total_time": float(trajectory.total_time),
        "body_half_extents": list(config.body.half_extents),
        "net_clearance": float(config.clearance),
        "traversal_order": [problem.windows[index].name for index in problem.order],
        "crossings": [
            {
                key: value for key, value in record.items()
                if key in {"route_index", "window_index", "window_name", "time", "minimum_sampled_boundary_distance"}
            }
            for record in crossings
        ],
        "minimum_crossing_sampled_distance": min(record["minimum_sampled_boundary_distance"] for record in crossings),
        "locally_refined_critical_clearance": critical_clearance,
    }
    (output / "cuboid_replay_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", required=True)
    parser.add_argument("--outdir", required=True)
    parser.add_argument("--no-gif", action="store_true")
    args = parser.parse_args()
    print(json.dumps(run(args.run, args.outdir, make_gif=not args.no_gif), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
