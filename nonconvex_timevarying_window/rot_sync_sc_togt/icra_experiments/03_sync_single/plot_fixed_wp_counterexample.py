#!/usr/bin/env python3
"""Plot the retained U-shaped Fixed-WP collision counterexample."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
REPOSITORY_ROOT = HERE.parents[3]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

import run_experiment as experiment
from nonconvex_timevarying_window.rot_sync_sc_togt.collision import _slab_cross_section
from nonconvex_timevarying_window.rot_sync_sc_togt.geometry import basis_from_normal, rotation_2d


DEFAULT_CASE = (
    HERE / "fixed_wp_u_search" / "balanced_candidate_comparison" / "scenarios"
    / "U_r1p90_w4p50_p1p1" / "Fixed-WP"
)


def _closed(vertices: np.ndarray) -> np.ndarray:
    return np.vstack((vertices, vertices[0]))


def _physical_vertices(ratio: float) -> np.ndarray:
    source = np.asarray((
        (-2.5, -2.5), (2.5, -2.5), (2.5, 2.5), (0.5, 2.5),
        (0.5, -0.5), (-0.5, -0.5), (-0.5, 2.5), (-2.5, 2.5),
    ), dtype=float)
    source_inradius = 1.1715728752460564
    scale = ratio * experiment.PLANNING_RHO / source_inradius
    return (source - np.asarray((0.0, 1.0))) * scale


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", type=Path, default=DEFAULT_CASE)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    case = args.case.resolve()
    result = json.loads((case / "result.json").read_text(encoding="utf-8"))
    protocol = json.loads((case.parents[2] / "search_protocol.json").read_text(encoding="utf-8"))
    with np.load(case / "trajectory_raw.npz", allow_pickle=False) as archive:
        data = {key: np.asarray(archive[key]) for key in archive.files}

    row = result["row"]
    physical = _physical_vertices(float(row["size_ratio"]))
    safe = np.asarray(protocol["geometries"][0]["safe_vertices"], dtype=float)
    fixed_q = np.asarray(result["selected_local_points"][0], dtype=float)
    collision_indices = np.flatnonzero(data["collision"])
    if not len(collision_indices):
        raise RuntimeError("selected trajectory has no audited collision sample")
    first = int(collision_indices[0])
    instant = float(data["time"][first])

    basis, normal = basis_from_normal(experiment.WINDOW_NORMAL)
    omega, phase = float(row["omega"]), float(row["phase"])
    window = SimpleNamespace(
        center=experiment.WINDOW_CENTER,
        normal=normal,
        thickness=experiment.THICKNESS,
        physical_polygon=physical,
        rotated_basis=lambda t: basis @ rotation_2d(phase + omega * float(t)),
    )
    section = _slab_cross_section(
        window, instant, data["position"][first], data["body_rotation"][first],
        experiment.BODY, tolerance=1.0e-9,
    )
    section_xy = np.asarray(section.exterior.coords, dtype=float)

    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    figure, axes = plt.subplots(1, 2, figsize=(10.2, 4.5), constrained_layout=True)
    for axis in axes:
        axis.fill(*_closed(physical).T, color="#D9D9D9", alpha=0.75, label="physical aperture")
        axis.plot(*_closed(physical).T, color="#333333", linewidth=1.5)
        axis.fill(*_closed(safe).T, color="#4C78A8", alpha=0.22, label="inset safe region")
        axis.plot(*_closed(safe).T, color="#4C78A8", linewidth=1.5, linestyle="--")
        axis.plot(0.0, 0.0, marker="+", markersize=11, markeredgewidth=2, color="#222222", label="rotation axis")
        axis.set_aspect("equal")
        axis.set_xlabel("window-local $u$ (m)")
        axis.set_ylabel("window-local $v$ (m)")
        axis.grid(alpha=0.18)

    axes[0].plot(*fixed_q, marker="o", markersize=6, color="#F58518", label="fixed waypoint")
    axes[0].set_title("Both physical and inset regions remain U-shaped")
    axes[0].legend(loc="lower right", fontsize=8)

    axes[1].fill(*section_xy.T, color="#E45756", alpha=0.45, label="cuboid/slab section")
    axes[1].plot(*section_xy.T, color="#B22222", linewidth=2)
    axes[1].set_title(f"First audited collision: $t={instant:.3f}$ s")
    axes[1].legend(loc="lower right", fontsize=8)
    figure.suptitle(
        f"Fixed-WP counterexample — balanced U, ratio 1.9, "
        f"$\\omega={omega:g}$ rad/s, $\\theta_0={phase:g}$ rad\n"
        f"{row['colliding_samples']} colliding samples; dynamics within limits",
        fontsize=12,
    )
    output = args.output.resolve() if args.output else case.parents[2] / "counterexample.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=220)
    figure.savefig(output.with_suffix(".pdf"))
    print(output)


if __name__ == "__main__":
    main()
