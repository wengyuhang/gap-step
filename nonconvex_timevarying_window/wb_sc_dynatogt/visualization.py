"""Figures for candidate recovery, attitude and method comparisons."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.dynamics import flatness_from_trajectory


def _pyplot():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def plot_candidate_recovery(
    polygon: np.ndarray,
    centers: np.ndarray,
    sphere_feasible: np.ndarray,
    cuboid_feasible: np.ndarray,
    path: str | Path,
) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    plt = _pyplot()
    figure, axis = plt.subplots(figsize=(6.2, 5.4))
    boundary = np.vstack((polygon, polygon[0]))
    axis.plot(boundary[:, 0], boundary[:, 1], color="black", linewidth=1.5, label="physical aperture")
    recovered = cuboid_feasible & ~sphere_feasible
    optimistic = sphere_feasible
    limit = min(8_000, len(centers))
    indices = np.linspace(0, len(centers) - 1, limit, dtype=int)
    axis.scatter(
        centers[indices][~cuboid_feasible[indices], 0],
        centers[indices][~cuboid_feasible[indices], 1],
        s=2,
        color="#c7c7c7",
        alpha=0.25,
        label="infeasible sample",
    )
    axis.scatter(
        centers[indices][optimistic[indices], 0],
        centers[indices][optimistic[indices], 1],
        s=4,
        color="#377eb8",
        alpha=0.45,
        label="legacy sphere feasible",
    )
    axis.scatter(
        centers[indices][recovered[indices], 0],
        centers[indices][recovered[indices], 1],
        s=5,
        color="#e41a1c",
        alpha=0.55,
        label="recovered by oriented cuboid",
    )
    axis.set_aspect("equal")
    axis.set_xlabel("gate-local u [m]")
    axis.set_ylabel("gate-local v [m]")
    axis.legend(fontsize=8, loc="best")
    axis.set_title("Candidate-set recovery")
    figure.tight_layout()
    figure.savefig(output, dpi=180)
    plt.close(figure)
    return output


def _rpy(rotation: np.ndarray) -> np.ndarray:
    pitch = np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))
    if abs(np.cos(pitch)) > 1.0e-8:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = 0.0
        yaw = np.arctan2(-rotation[0, 1], rotation[1, 1])
    return np.array([roll, pitch, yaw])


def plot_attitude_and_clearance(
    result: Any,
    path: str | Path,
    *,
    track: Any | None = None,
    samples: int = 241,
) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    times = np.linspace(0.0, result.total_time, samples)
    attitudes = np.empty((samples, 3), dtype=float)
    for index, time in enumerate(times):
        yaw = float(result.yaw_trajectory.evaluate(time))
        yaw_rate = float(result.yaw_trajectory.evaluate(time, 1))
        yaw_acceleration = float(result.yaw_trajectory.evaluate(time, 2))
        state = flatness_from_trajectory(
            result.trajectory,
            float(time),
            yaw=yaw,
            yaw_rate=yaw_rate,
            yaw_acceleration=yaw_acceleration,
            parameters=result.config.quadrotor,
        )
        attitudes[index] = _rpy(np.asarray(np.real(state.rotation), dtype=float))
    crossing_times = np.array([], dtype=float)
    crossing_clearances = np.array([], dtype=float)
    if result.safety_report is not None:
        crossing_times = np.asarray([check.time for check in result.safety_report.checks])
        crossing_clearances = np.asarray([check.clearance for check in result.safety_report.checks])
    plt = _pyplot()
    figure, axes = plt.subplots(2, 1, figsize=(8.0, 5.8), sharex=True)
    for column, label in enumerate(("roll", "pitch", "yaw")):
        axes[0].plot(times, attitudes[:, column], label=label)
    axes[0].set_ylabel("attitude [rad]")
    axes[0].legend(ncol=3, fontsize=8)
    axes[0].grid(alpha=0.25)
    axes[1].scatter(
        crossing_times,
        crossing_clearances,
        color="#4daf4a",
        label="cuboid clearance at crossing",
    )
    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].set_ylabel("whole-body clearance [m]")
    axes[1].set_xlabel("time [s]")
    axes[1].grid(alpha=0.25)
    axes[1].legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(output, dpi=180)
    plt.close(figure)
    return output


def plot_method_comparison(rows: Sequence[dict[str, Any]], path: str | Path) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    methods = list(dict.fromkeys(str(row["method"]) for row in rows))
    success = []
    for method in methods:
        values = [bool(row["safe_success"]) for row in rows if row["method"] == method]
        success.append(float(np.mean(values)) if values else float("nan"))
    plt = _pyplot()
    figure, axis = plt.subplots(figsize=(8.2, 4.5))
    axis.bar(np.arange(len(methods)), success, color="#377eb8")
    axis.set_xticks(np.arange(len(methods)), methods, rotation=20, ha="right")
    axis.set_ylim(0.0, 1.0)
    axis.set_ylabel("converged and model-safe rate")
    axis.grid(axis="y", alpha=0.25)
    figure.tight_layout()
    figure.savefig(output, dpi=180)
    plt.close(figure)
    return output


__all__ = [
    "plot_attitude_and_clearance",
    "plot_candidate_recovery",
    "plot_method_comparison",
]
