"""Diagnostic plots for crossing intervals and worst cuboid sections."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.dynamics import QuadrotorParameters, YawProfile
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.optimizer import ForwardPass

from .body_model import CuboidBody
from .config import WholeBodySafetyConfig
from .gate_frame import frame_at
from .plane_section import cuboid_world_vertices, gate_local_vertex_coordinates, plane_section_at
from .whole_body_safety import TrajectorySafetyReport


def plot_crossing_diagnostics(
    *,
    forward: ForwardPass,
    track: SCWindowTrack,
    body: CuboidBody,
    config: WholeBodySafetyConfig,
    report: TrajectorySafetyReport,
    output: str | Path,
    yaw_profile: YawProfile | None = None,
    parameters: QuadrotorParameters | None = None,
) -> Path:
    """Plot vertex normal extrema and the most dangerous local section."""

    import matplotlib.pyplot as plt

    if len(report.windows) != len(track.order):
        raise ValueError("report and track must have the same crossing count")
    destination = Path(output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    rows = max(1, len(report.windows))
    figure, axes = plt.subplots(rows, 2, figsize=(11.0, 3.8 * rows), squeeze=False)
    for crossing_index, window_report in enumerate(report.windows):
        window_index = track.order[crossing_index]
        window = track.windows[window_index]
        crossing = window_report.crossing_interval
        times = np.linspace(crossing.start, crossing.end, 161)
        minima, maxima = [], []
        for time in times:
            vertices, _ = cuboid_world_vertices(
                forward.trajectory, float(time), body,
                yaw_profile=yaw_profile, parameters=parameters,
            )
            xi = gate_local_vertex_coordinates(vertices, frame_at(window, float(time)))[:, 2]
            minima.append(float(np.min(xi)))
            maxima.append(float(np.max(xi)))
        axis = axes[crossing_index, 0]
        axis.plot(times, minima, label=r"$\xi_{min}^3$")
        axis.plot(times, maxima, label=r"$\xi_{max}^3$")
        axis.axhline(0.0, color="black", linewidth=0.8)
        axis.axvline(crossing.traversal_time, color="tab:red", linestyle="--", label="$t_i$")
        axis.axvspan(crossing.start, crossing.end, color="tab:orange", alpha=0.12)
        axis.set(title=f"{window.name}: plane coordinates", xlabel="time [s]", ylabel="metres")
        axis.legend()

        witness = window_report.witnesses[0] if window_report.witnesses else None
        if witness is None:
            time = crossing.traversal_time
        else:
            starts = np.concatenate(([0.0], np.cumsum(forward.durations[:-1])))
            time = float(starts[witness.minco_segment_index] + witness.normalized_time * forward.durations[witness.minco_segment_index])
        section = plane_section_at(
            forward.trajectory, time, window, body, config,
            yaw_profile=yaw_profile, parameters=parameters,
        )
        local_axis = axes[crossing_index, 1]
        safe = np.asarray(window.sc_map.evaluate_many(
            config.sc_safe_radius * np.column_stack((
                np.cos(np.linspace(0.0, 2.0 * np.pi, 361)),
                np.sin(np.linspace(0.0, 2.0 * np.pi, 361)),
            ))
        ))
        boundary = np.vstack((window.safe_polygon, window.safe_polygon[0]))
        local_axis.plot(boundary[:, 0], boundary[:, 1], color="black", label="SC domain")
        local_axis.plot(safe[:, 0], safe[:, 1], color="tab:green", label="safe radius image")
        polygon = section.local_polygon
        if len(polygon):
            polygon = np.vstack((polygon, polygon[0]))
            local_axis.plot(polygon[:, 0], polygon[:, 1], color="tab:blue", marker="o", label="cuboid section")
        if witness is not None:
            local_axis.scatter(*witness.local_point, color="tab:red", marker="x", s=70, label="witness")
        local_axis.set_aspect("equal", adjustable="box")
        local_axis.set(title=f"section at t={time:.4f}s", xlabel="gate q1", ylabel="gate q2")
        local_axis.legend()
    figure.tight_layout()
    figure.savefig(destination, dpi=180)
    plt.close(figure)
    return destination


__all__ = ["plot_crossing_diagnostics"]
