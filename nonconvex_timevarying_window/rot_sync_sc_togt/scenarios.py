"""Closed multi-window tracks for rotation-synchronised SC/MINCO experiments."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike

from nonconvex_timevarying_window.sc_dynatogt.collision import CuboidBody
from nonconvex_timevarying_window.sc_dynatogt.boundary import DenseBoundary
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import (
    PreprocessedGate,
    PreprocessingConfig,
    e1_boundaries,
    preprocess_boundary,
)

from .geometry import RotatingWindow, basis_from_normal


DEFAULT_BODY = CuboidBody()
DEFAULT_RHO = DEFAULT_BODY.circumscribed_radius
REALISTIC_BODY = CuboidBody((0.30, 0.30, 0.09))
REALISTIC_RHO = REALISTIC_BODY.circumscribed_radius
REALISTIC_SHAPE_SCALES = {
    "L": 0.72,
    "U": 0.65,
    "star": 0.68,
    "limacon": 0.60,
    "wavy": 0.65,
    "line_bezier": 0.65,
}


@dataclass(frozen=True)
class RotSyncScenario:
    name: str
    start_state: BoundaryState
    goal_state: BoundaryState
    windows: tuple[RotatingWindow, ...]
    description: str
    body: CuboidBody = DEFAULT_BODY
    difficulty: str = "custom"
    design_basis: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "windows", tuple(self.windows))
        object.__setattr__(self, "design_basis", tuple(self.design_basis))
        if not self.windows:
            raise ValueError("a scenario needs at least one window")
        if any(window.rho + 1.0e-12 < self.body.circumscribed_radius for window in self.windows):
            raise ValueError("window rho must contain the complete cuboid body")

    @property
    def closed(self) -> bool:
        return bool(np.array_equal(self.start_state.matrix, self.goal_state.matrix))


def preprocess_shape_catalog(
    *,
    rho: float = DEFAULT_RHO,
    quadrature_order: int = 32,
    vertex_count: int | None = None,
    shape_names: Sequence[str] | None = None,
    shape_scales: Mapping[str, float] | None = None,
) -> dict[str, PreprocessedGate]:
    """Reuse the SC-DynaTOGT inset/SC pipeline for all six existing apertures.

    ``vertex_count=None`` selects the adaptive production vertex ladder from
    :class:`PreprocessingConfig`; an integer is retained for controlled tests.
    """

    config = PreprocessingConfig(
        **({} if vertex_count is None else {"vertex_counts": (int(vertex_count),)}),
        offset_distance=float(rho),
        sc_fit_options={"quadrature_order": int(quadrature_order), "max_nfev": 1200},
    )
    boundaries = e1_boundaries()
    definitions = {
        "L": boundaries["l_shape"],
        "U": boundaries["u_shape"],
        "star": boundaries["five_point_star"],
        "limacon": boundaries["limacon"],
        "wavy": boundaries["wavy"],
        "line_bezier": boundaries["line_bezier_mixed"],
    }
    selected = tuple(definitions) if shape_names is None else tuple(shape_names)
    unknown = set(selected) - set(definitions)
    if unknown:
        raise ValueError(f"unknown gate shapes: {sorted(unknown)}")
    scales = {} if shape_scales is None else dict(shape_scales)
    unknown_scales = set(scales) - set(definitions)
    if unknown_scales:
        raise ValueError(f"unknown scaled gate shapes: {sorted(unknown_scales)}")
    if any(not np.isfinite(value) or value <= 0.0 for value in scales.values()):
        raise ValueError("gate shape scales must be finite and positive")

    def scaled_boundary(name: str) -> DenseBoundary:
        boundary = definitions[name]
        scale = float(scales.get(name, 1.0))
        return DenseBoundary(
            boundary.vertices * scale,
            boundary.corners * scale,
            boundary.corner_indices,
        )

    return {
        name: preprocess_boundary(scaled_boundary(name), name=name, config=config)
        for name in selected
    }


def _window(
    gate: PreprocessedGate,
    *,
    center: ArrayLike,
    normal: ArrayLike,
    theta0: float,
    omega: float,
    thickness: float,
    rho: float,
) -> RotatingWindow:
    basis, unit_normal = basis_from_normal(normal)
    return RotatingWindow(
        name=gate.name,
        gate=gate,
        center=center,
        plane_basis=basis,
        normal=unit_normal,
        theta0=theta0,
        omega=omega,
        thickness=thickness,
        rho=rho,
    )


def build_smoke_scenario(
    catalog: Mapping[str, PreprocessedGate] | None = None,
    *,
    rho: float = DEFAULT_RHO,
    body: CuboidBody = DEFAULT_BODY,
) -> RotSyncScenario:
    gates = (
        preprocess_shape_catalog(rho=rho, vertex_count=64, shape_names=("L",))
        if catalog is None
        else dict(catalog)
    )
    altitude = 1.8
    start = BoundaryState(np.asarray((-4.5, 0.0, altitude)))
    goal = BoundaryState(np.asarray((4.5, 0.0, altitude)))
    window = _window(
        gates["L"],
        center=(0.0, 0.0, altitude),
        normal=(1.0, 0.0, 0.0),
        theta0=0.30,
        omega=0.75,
        thickness=0.14,
        rho=rho,
    )
    return RotSyncScenario(
        "single_L_smoke",
        start,
        goal,
        (window,),
        "Single L aperture: MINCO -> analytic rotation Sync -> MINCO.",
        body,
    )


def _closed_track(
    name: str,
    centers: ArrayLike,
    start: ArrayLike,
    gates: Sequence[PreprocessedGate],
    *,
    omegas: Sequence[float],
    theta0: Sequence[float],
    thicknesses: Sequence[float],
    rho: float,
    description: str,
    body: CuboidBody,
    difficulty: str = "custom",
    design_basis: Sequence[str] = (),
) -> RotSyncScenario:
    center_values = np.asarray(centers, dtype=float)
    start_value = np.asarray(start, dtype=float)
    count = len(gates)
    if center_values.shape != (count, 3) or start_value.shape != (3,):
        raise ValueError("closed-track centers/start have invalid shape")
    if not all(len(values) == count for values in (omegas, theta0, thicknesses)):
        raise ValueError("one omega, theta0 and thickness are required per window")
    windows = []
    for index, gate in enumerate(gates):
        previous = start_value if index == 0 else center_values[index - 1]
        following = start_value if index == count - 1 else center_values[index + 1]
        tangent = following - previous
        windows.append(
            _window(
                gate,
                center=center_values[index],
                normal=tangent,
                theta0=theta0[index],
                omega=omegas[index],
                thickness=thicknesses[index],
                rho=rho,
            )
        )
    state = BoundaryState(start_value)
    return RotSyncScenario(
        name,
        state,
        state,
        tuple(windows),
        description,
        body,
        difficulty,
        tuple(design_basis),
    )


def scenario_difficulty_metrics(scenario: RotSyncScenario) -> dict[str, object]:
    """Return geometry/motion descriptors saved with every formal run."""

    anchors = np.vstack(
        (
            scenario.start_state.position,
            *(window.center for window in scenario.windows),
            scenario.goal_state.position,
        )
    )
    legs = np.diff(anchors, axis=0)
    lengths = np.linalg.norm(legs, axis=1)
    turn_angles = []
    for incoming, outgoing in zip(legs[:-1], legs[1:]):
        denominator = float(np.linalg.norm(incoming) * np.linalg.norm(outgoing))
        cosine = float(np.dot(incoming, outgoing) / denominator)
        turn_angles.append(float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))))
    omega = np.abs(np.asarray([window.omega for window in scenario.windows], dtype=float))
    shapes = tuple(window.name for window in scenario.windows)
    return {
        "difficulty": scenario.difficulty,
        "gate_count": len(scenario.windows),
        "shape_sequence": shapes,
        "unique_shape_count": len(set(shapes)),
        "nominal_route_length": float(np.sum(lengths)),
        "longest_leg": float(np.max(lengths)),
        "altitude_range": float(np.ptp(anchors[:, 2])),
        "maximum_turn_angle_deg": float(max(turn_angles, default=0.0)),
        "mean_abs_omega": float(np.mean(omega)),
        "maximum_abs_omega": float(np.max(omega)),
    }


def build_multi_scenarios(
    catalog: Mapping[str, PreprocessedGate] | None = None,
    *,
    rho: float = DEFAULT_RHO,
    body: CuboidBody = DEFAULT_BODY,
) -> tuple[RotSyncScenario, ...]:
    """Return three geometrically distinct closed L/U/star race tracks."""

    items = (
        preprocess_shape_catalog(
            rho=rho,
            vertex_count=64,
            shape_names=("L", "U", "star"),
        )
        if catalog is None
        else dict(catalog)
    )
    gates = (items["L"], items["U"], items["star"])
    common = {
        "gates": gates,
        "omegas": (0.62, -0.78, 0.95),
        "theta0": (0.15, -0.35, 0.55),
        "thicknesses": (0.12, 0.16, 0.14),
        "rho": rho,
        "body": body,
    }
    return (
        _closed_track(
            "closed_triangle",
            ((0.0, -2.6, 1.8), (4.3, 1.7, 2.2), (-4.0, 2.0, 1.6)),
            (0.0, -6.4, 1.8),
            description="Planar triangular loop with three differently spinning apertures.",
            **common,
        ),
        _closed_track(
            "closed_slalom",
            ((-2.7, -2.2, 1.5), (3.8, -0.1, 2.8), (-0.8, 3.7, 1.9)),
            (-5.5, -4.6, 1.8),
            description="Asymmetric slalom loop with a pronounced altitude change.",
            **common,
        ),
        _closed_track(
            "closed_spatial",
            ((-0.5, -2.8, 1.0), (3.4, 1.0, 4.0), (-3.7, 2.5, 2.4)),
            (-4.8, -4.4, 2.0),
            description="Three-dimensional closed loop stressing PVAJ interface matching.",
            **common,
        ),
    )


def build_formal_scenarios(
    catalog: Mapping[str, PreprocessedGate] | None = None,
    *,
    rho: float = DEFAULT_RHO,
    body: CuboidBody = DEFAULT_BODY,
) -> tuple[RotSyncScenario, ...]:
    """Four deterministic closed tracks with increasing geometric difficulty.

    The suite varies gate count, route length, altitude excursion, turn angle,
    spin rate and non-convex boundary type.  It is deterministic because the
    optimisation method itself is deterministic; each track is a separate
    benchmark rather than a random-seed replicate.
    """

    items = preprocess_shape_catalog(rho=rho, vertex_count=None, quadrature_order=64) if catalog is None else dict(catalog)
    race_basis = (
        "AlphaPilot RSS 2020: compact, tight multi-gate racing course",
        "TOGT ICRA 2024: gate shape/size and irregular multi-gate layout matter",
    )

    spatial_basis = (
        "Flightmare CoRL 2020: collision-free planning in varied 3D scenes",
        "HJB-RL RSS 2021: longer ordered multi-gate race sequence",
    )
    return (
        _closed_track(
            "D1_compact_planar",
            ((-4.0, -2.0, 1.8), (4.0, -2.0, 1.9), (0.0, 5.0, 1.8)),
            (0.0, -7.0, 1.8),
            (items["L"], items["limacon"], items["U"]),
            omegas=(0.28, -0.36, 0.44),
            theta0=(0.10, -0.24, 0.35),
            thicknesses=(0.12, 0.12, 0.14),
            rho=rho,
            description="D1 easy: compact, almost planar three-gate closed lap.",
            body=body,
            difficulty="D1-easy",
            design_basis=race_basis,
        ),
        _closed_track(
            "D2_spatial_slalom",
            ((-4.5, -3.0, 1.3), (4.0, -4.0, 2.6), (6.0, 3.0, 1.7), (-2.0, 6.0, 3.2)),
            (-8.0, -6.0, 1.6),
            (items["wavy"], items["star"], items["U"], items["line_bezier"]),
            omegas=(0.48, -0.62, 0.72, -0.82),
            theta0=(-0.20, 0.34, -0.48, 0.61),
            thicknesses=(0.12, 0.15, 0.14, 0.16),
            rho=rho,
            description="D2 medium: four-gate asymmetric 3D slalom with mixed boundaries.",
            body=body,
            difficulty="D2-medium",
            design_basis=race_basis + spatial_basis[:1],
        ),
        _closed_track(
            "D3_uzh_irregular",
            np.asarray(
                (
                    (-1.1, -1.6, 3.6),
                    (9.2, 6.6, 1.0),
                    (9.2, -4.0, 1.2),
                    (-4.5, -6.0, 3.5),
                    (4.75, -0.9, 1.2),
                    (-2.8, 6.8, 1.2),
                )
            )
            * np.asarray((1.25, 1.25, 1.0)),
            (-9.0, 3.0, 3.2),
            tuple(items[name] for name in ("L", "U", "star", "limacon", "wavy", "line_bezier")),
            omegas=(0.68, -0.78, 0.88, -0.70, 1.08, -1.18),
            theta0=(0.00, -0.30, 0.52, -0.70, 0.91, -1.05),
            thicknesses=(0.12, 0.14, 0.16, 0.12, 0.15, 0.18),
            rho=rho,
            description="D3 hard: six-shape irregular closed track derived from the TOGT UZH order pattern.",
            body=body,
            difficulty="D3-hard",
            design_basis=race_basis + spatial_basis,
        ),
        _closed_track(
            "D4_split_s_endurance",
            ((-6.0, -2.0, 1.0), (-1.0, 3.0, 5.5), (5.0, 5.0, 2.0), (8.0, 0.0, 6.0), (3.0, -5.0, 1.4), (-8.0, 5.0, 4.8)),
            (-10.0, -6.0, 2.0),
            tuple(items[name] for name in ("star", "line_bezier", "L", "wavy", "U", "limacon")),
            omegas=(0.92, -1.04, 1.16, -1.28, 1.40, -1.52),
            theta0=(0.18, -0.37, 0.56, -0.75, 0.94, -1.13),
            thicknesses=(0.16, 0.18, 0.14, 0.16, 0.18, 0.14),
            rho=rho,
            description="D4 extreme: six-gate Split-S-style endurance lap with repeated altitude reversals.",
            body=body,
            difficulty="D4-extreme",
            design_basis=race_basis + spatial_basis,
        ),
    )


def build_realistic_extreme_scenario(
    catalog: Mapping[str, PreprocessedGate] | None = None,
    *,
    rho: float = REALISTIC_RHO,
    body: CuboidBody = REALISTIC_BODY,
) -> RotSyncScenario:
    """Large-body, widely spaced version of the hardest six-window track."""

    items = (
        preprocess_shape_catalog(
            rho=rho,
            vertex_count=None,
            quadrature_order=64,
            shape_scales=REALISTIC_SHAPE_SCALES,
        )
        if catalog is None
        else dict(catalog)
    )
    return _closed_track(
        "D4_realistic_spread",
        (
            (-9.0, -3.0, 1.0),
            (-1.5, 4.5, 6.5),
            (7.5, 7.5, 2.0),
            (12.0, 0.0, 7.0),
            (4.5, -7.5, 1.4),
            (-12.0, 7.5, 5.5),
        ),
        (-15.0, -9.0, 2.5),
        tuple(
            items[name]
            for name in ("star", "line_bezier", "L", "wavy", "U", "limacon")
        ),
        omegas=(0.92, -1.04, 1.16, -1.28, 1.40, -1.52),
        theta0=(0.18, -0.37, 0.56, -0.75, 0.94, -1.13),
        thicknesses=(0.18, 0.20, 0.16, 0.18, 0.20, 0.16),
        rho=rho,
        description=(
            "Realistic extreme rerun: 0.60 x 0.60 x 0.18 m body envelope "
            "and widely separated six-gate Split-S closed lap."
        ),
        body=body,
        difficulty="D4-realistic-extreme",
        design_basis=(
            "DJI F450: 450 mm diagonal wheelbase, 10 inch recommended propellers",
            "Conservative 0.60 m square propeller envelope with 0.18 m body height",
            "Widely separated 3D Split-S gate layout",
        ),
    )


__all__ = [
    "DEFAULT_BODY",
    "DEFAULT_RHO",
    "REALISTIC_BODY",
    "REALISTIC_RHO",
    "REALISTIC_SHAPE_SCALES",
    "RotSyncScenario",
    "build_formal_scenarios",
    "build_multi_scenarios",
    "build_realistic_extreme_scenario",
    "build_smoke_scenario",
    "preprocess_shape_catalog",
    "scenario_difficulty_metrics",
]
