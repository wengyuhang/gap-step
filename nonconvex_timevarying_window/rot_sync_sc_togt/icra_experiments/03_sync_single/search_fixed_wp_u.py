#!/usr/bin/env python3
"""Reproducible Fixed-WP counterexample search on topology-preserving U offsets.

This is an exploratory search adapter, not part of the frozen formal grid.  It
uses the same body, 0.14 m gate thickness, objective, solver budget, and 1 ms
independent audit as ``run_experiment.py``.  Every requested grid point is kept.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from shapely.geometry import Polygon

HERE = Path(__file__).resolve().parent
REPOSITORY_ROOT = HERE.parents[3]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

import run_experiment as experiment


def _numbers(text: str) -> tuple[float, ...]:
    return tuple(float(value) for value in text.split(",") if value.strip())


def _reflex_vertices(vertices: np.ndarray) -> int:
    edge_in = vertices - np.roll(vertices, 1, axis=0)
    edge_out = np.roll(vertices, -1, axis=0) - vertices
    cross = edge_in[:, 0] * edge_out[:, 1] - edge_in[:, 1] * edge_out[:, 0]
    return int(np.count_nonzero(cross < -1.0e-10))


def _balanced_u_source() -> experiment.DenseBoundary:
    """A U whose two arms and bottom bar all have the same width.

    The repository's canonical U has a much shallower bottom notch and loses
    its arms under a tight inset.  This profile permits a tight inset while
    preserving the same eight-vertex, two-reflex-vertex U topology.
    """
    vertices = np.asarray((
        (-2.5, -2.5), (2.5, -2.5), (2.5, 2.5), (0.5, 2.5),
        (0.5, -0.5), (-0.5, -0.5), (-0.5, 2.5), (-2.5, 2.5),
    ), dtype=float)
    return experiment.DenseBoundary(vertices, vertices.copy(), tuple(range(len(vertices))))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ratios", default="2.0")
    parser.add_argument("--omegas", default="15,18,24,30")
    parser.add_argument("--phases", default="0.3,1.1,2.2")
    parser.add_argument(
        "--methods", nargs="+", choices=experiment.METHODS,
        default=["Fixed-WP"],
    )
    parser.add_argument("--output", type=Path, default=HERE / "fixed_wp_u_search" / "extreme_speed")
    parser.add_argument("--budget", type=float, default=180.0)
    parser.add_argument("--max-iterations", type=int, default=120)
    parser.add_argument("--vertex-count", type=int, default=96)
    parser.add_argument("--quadrature-order", type=int, default=96)
    parser.add_argument("--u-profile", choices=("canonical", "balanced"), default="canonical")
    args = parser.parse_args()

    ratios, omegas, phases = map(_numbers, (args.ratios, args.omegas, args.phases))
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=True)
    weights = json.loads((HERE / "focused_results" / "frozen_weights.json").read_text(encoding="utf-8"))
    config = experiment.make_config(weights, args.max_iterations)
    rows = []
    geometry_records = []

    for ratio in ratios:
        source = _balanced_u_source() if args.u_profile == "balanced" else None
        axis = np.asarray((0.0, 1.0)) if args.u_profile == "balanced" else None
        geometry = experiment.prepare_geometry(
            "U", ratio, vertex_count=args.vertex_count,
            quadrature_order=args.quadrature_order,
            canonical_axis=axis, source_boundary=source,
        )
        physical = Polygon(geometry.gate.dense_boundary.vertices)
        safe = Polygon(geometry.gate.safe_region.vertices)
        topology = {
            "ratio": ratio,
            "physical_vertex_count": len(geometry.gate.dense_boundary.vertices),
            "physical_reflex_vertex_count": _reflex_vertices(geometry.gate.dense_boundary.vertices),
            "physical_components": 1,
            "physical_holes": len(physical.interiors),
            "safe_vertex_count": len(geometry.gate.safe_region.vertices),
            "safe_reflex_vertex_count": _reflex_vertices(geometry.gate.safe_region.vertices),
            "safe_components": 1,
            "safe_holes": len(safe.interiors),
            "safe_is_nonconvex": not safe.equals(safe.convex_hull),
            "safe_area": safe.area,
            "safe_vertices": geometry.gate.safe_region.vertices,
        }
        geometry_records.append(topology)
        if not (topology["safe_vertex_count"] == 8 and topology["safe_reflex_vertex_count"] == 2
                and topology["safe_is_nonconvex"]):
            raise RuntimeError(f"ratio {ratio:g} does not preserve the U-shaped safe region")

        for omega in omegas:
            for phase in phases:
                name = experiment.scenario_name("U", ratio, omega, phase)
                scenario = experiment.build_scenario(geometry, omega, phase, name=name)
                for method in args.methods:
                    target = root / "scenarios" / name / method
                    row, _ = experiment.run_one(
                        scenario, geometry, method, config,
                        float(weights["collision_weight"]), args.budget, target,
                    )
                    rows.append(row)
                    print(
                        name, method, "PASS", row["trajectory_pass"],
                        "CATEGORY", row["failure_category"],
                        "HITS", row["colliding_samples"],
                        "DYNREL", row["max_dynamic_relative_violation"],
                        "T", row["flight_time"], flush=True,
                    )

    experiment.write_summary(root / "results.csv", rows)
    experiment.write_json(root / "results.json", rows)
    experiment.write_json(root / "search_protocol.json", {
        "purpose": "Fixed-WP collision search with a topology-preserving U safe region",
        "status": "exploratory_not_part_of_frozen_formal_grid",
        "u_profile": args.u_profile,
        "ratios": ratios,
        "omegas": omegas,
        "phases": phases,
        "methods": args.methods,
        "budget_seconds_each": args.budget,
        "max_iterations": args.max_iterations,
        "body_half_extents_m": experiment.BODY.half_extents,
        "planning_envelope_radius_m": experiment.PLANNING_RHO,
        "gate_thickness_m": experiment.THICKNESS,
        "audit": "<=1ms plus critical-time refinement; sampled check, not continuous-time proof",
        "formal_dynamic_limits_unchanged": True,
        "weights": weights,
        "geometries": geometry_records,
    })


if __name__ == "__main__":
    main()
