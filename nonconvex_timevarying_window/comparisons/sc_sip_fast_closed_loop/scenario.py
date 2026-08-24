"""Wide, scrambled closed-loop benchmark with six rapidly moving windows."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.boundary import (
    BSpline, Bezier, BoundarySegment, CircularArc, DenseBoundary, Line,
)
from nonconvex_timevarying_window.sc_dynatogt.collision import CuboidBody
from nonconvex_timevarying_window.sc_dynatogt.environment import (
    MotionProfile, SCDynamicWindow, SCWindowTrack,
)
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import PreprocessingConfig
from nonconvex_timevarying_window.sc_dynatogt.scenarios import (
    preprocess_for_fixed_world_clearance,
)


@dataclass(frozen=True)
class FastClosedLoopScenario:
    name: str
    mode: str
    track: SCWindowTrack
    preprocessed_gates: tuple
    sip_boundaries: tuple[tuple[BoundarySegment, ...], ...]
    body: CuboidBody
    net_clearance: float


def _line_ring(vertices: np.ndarray) -> tuple[Line, ...]:
    points = np.asarray(vertices, dtype=float)
    return tuple(Line(points[i], points[(i + 1) % len(points)]) for i in range(len(points)))


def _primitive_boundaries() -> tuple[tuple[str, tuple[BoundarySegment, ...]], ...]:
    """Exact physical curves; dense samples are created only for SC fitting."""

    l_vertices = np.asarray(
        [(-3.2, -3.0), (3.2, -3.0), (3.2, -0.35), (0.75, -0.35),
         (0.75, 3.0), (-3.2, 3.0)], dtype=float,
    )
    circle = (CircularArc((0.0, 0.0), 3.15, 0.0, 2.0 * math.pi),)
    bezier_notch: tuple[BoundarySegment, ...] = (
        Line((-3.30, -2.55), (3.30, -2.55)),
        Line((3.30, -2.55), (3.30, 2.45)),
        Bezier(((3.30, 2.45), (2.45, 2.55), (2.05, 0.65), (0.90, 0.70))),
        Bezier(((0.90, 0.70), (-0.10, 0.78), (-1.20, 2.65), (-3.30, 2.45))),
        Line((-3.30, 2.45), (-3.30, -2.55)),
    )
    bspline_wave: tuple[BoundarySegment, ...] = (
        Line((-3.45, -2.55), (3.45, -2.55)),
        Line((3.45, -2.55), (3.45, 1.65)),
        BSpline(
            ((3.45, 1.65), (2.65, 3.10), (1.65, 0.55), (0.55, 2.30),
             (-0.65, 0.45), (-1.95, 3.00), (-3.45, 1.65)), degree=3,
        ),
        Line((-3.45, 1.65), (-3.45, -2.55)),
    )
    capsule: tuple[BoundarySegment, ...] = (
        Line((-2.15, -2.45), (2.15, -2.45)),
        CircularArc((2.15, 0.0), 2.45, -0.5 * math.pi, 0.5 * math.pi),
        Line((2.15, 2.45), (-2.15, 2.45)),
        CircularArc((-2.15, 0.0), 2.45, 0.5 * math.pi, 1.5 * math.pi),
    )
    rounded_diamond: tuple[BoundarySegment, ...] = (
        Bezier(((0.0, -3.35), (1.15, -3.30), (2.85, -2.05), (3.25, -0.45))),
        Bezier(((3.25, -0.45), (3.65, 1.10), (1.55, 2.95), (0.0, 3.20))),
        Bezier(((0.0, 3.20), (-1.45, 3.00), (-3.60, 1.20), (-3.25, -0.45))),
        Bezier(((-3.25, -0.45), (-2.85, -2.05), (-1.10, -3.30), (0.0, -3.35))),
    )
    return (
        ("L_polygon", _line_ring(l_vertices)),
        ("circle_arc", circle),
        ("bezier_notch", bezier_notch),
        ("bspline_wave", bspline_wave),
        ("arc_capsule", capsule),
        ("bezier_diamond", rounded_diamond),
    )


def _rpy_with_normal(previous: np.ndarray, center: np.ndarray) -> np.ndarray:
    direction = np.asarray(center, dtype=float) - np.asarray(previous, dtype=float)
    normal = direction / np.linalg.norm(direction)
    pitch = math.acos(float(np.clip(normal[2], -1.0, 1.0)))
    yaw = math.atan2(float(normal[1]), float(normal[0]))
    return np.asarray((0.0, pitch, yaw), dtype=float)


def build_fast_closed_loop_scenario(
    preprocessing_config: PreprocessingConfig | None = None,
    *,
    body: CuboidBody | None = None,
    net_clearance: float = 0.015,
) -> FastClosedLoopScenario:
    """Build a difficult wide-domain closed-loop SC/SIP comparison.

    Window storage order is unrelated to traversal order.  The route spans
    28 m in x, 27 m in y and 10 m in z, repeatedly crossing the domain.
    Every window simultaneously translates, rotates in full RPY, and scales;
    scale amplitudes up to 0.60 retain a strict minimum scale of 0.40.

    SC uses a time-independent inset equal to the cuboid circumscribed radius
    plus net clearance, divided by minimum scale.  SIP retains the exact
    primitive curves and exact oriented-cuboid distance.
    """

    vehicle = CuboidBody() if body is None else body
    required_center_clearance = vehicle.conservative_center_clearance(net_clearance)
    base_settings = preprocessing_config or PreprocessingConfig(
        vertex_counts=(64, 128, 256),
        sc_fit_options={"quadrature_order": 32, "max_nfev": 1_200},
    )
    settings = replace(base_settings, offset_distance=required_center_clearance)

    endpoint = np.asarray((0.0, -18.0, 4.0), dtype=float)
    centers = np.asarray(
        [(12.0, -10.0, 8.0), (-13.0, 9.0, 3.0), (4.0, 14.0, 10.0),
         (-12.0, -12.0, 7.0), (14.0, 6.0, 4.0), (-2.0, 0.0, 13.0)],
        dtype=float,
    )
    order = (3, 4, 1, 5, 0, 2)
    primitive_definitions = _primitive_boundaries()
    definitions = tuple(
        (name, DenseBoundary.from_segments(segments))
        for name, segments in primitive_definitions
    )
    motions = (
        MotionProfile(np.asarray((1.40, 1.10, 0.90)), np.asarray((0.72, 0.58, 0.82)), 0.55,
                      translation_period=2.60, rotation_period=2.25, scale_period=2.90, phase=0.25),
        MotionProfile(np.asarray((1.75, 0.95, 1.15)), np.asarray((0.60, 0.78, 0.68)), 0.48,
                      translation_period=2.30, rotation_period=2.70, scale_period=2.45, phase=1.05),
        MotionProfile(np.asarray((1.10, 1.80, 1.00)), np.asarray((0.82, 0.66, 0.74)), 0.60,
                      translation_period=2.80, rotation_period=2.15, scale_period=3.10, phase=1.70),
        MotionProfile(np.asarray((1.85, 1.25, 0.85)), np.asarray((0.68, 0.84, 0.62)), 0.52,
                      translation_period=2.20, rotation_period=2.50, scale_period=2.35, phase=2.30),
        MotionProfile(np.asarray((1.30, 1.65, 1.20)), np.asarray((0.76, 0.70, 0.88)), 0.57,
                      translation_period=2.55, rotation_period=2.05, scale_period=2.75, phase=2.85),
        MotionProfile(np.asarray((1.60, 1.45, 1.30)), np.asarray((0.86, 0.64, 0.80)), 0.50,
                      translation_period=2.40, rotation_period=2.35, scale_period=2.20, phase=3.40),
    )
    gates = tuple(
        preprocess_for_fixed_world_clearance(
            boundary, name=name, settings=settings, motion=motion
        )
        for (name, boundary), motion in zip(definitions, motions)
    )

    angles: list[np.ndarray | None] = [None] * len(centers)
    previous = endpoint
    for window_index in order:
        angles[window_index] = _rpy_with_normal(previous, centers[window_index])
        previous = centers[window_index]
    resolved_angles = tuple(np.asarray(angle, dtype=float) for angle in angles)

    windows = tuple(
        SCDynamicWindow(
            name=gate.name, sc_map=gate.sc_map, safe_polygon=gate.safe_polygon,
            center0=center, angles0=angle, motion=motion,
            physical_boundary=gate.dense_boundary.vertices,
            required_world_clearance=settings.offset_distance,
            reference_local_clearance=gate.safe_region.distance,
        )
        for gate, center, angle, motion in zip(gates, centers, resolved_angles, motions)
    )
    track = SCWindowTrack(
        "wide_scrambled_fast_closed_loop_6", start=endpoint, goal=endpoint.copy(),
        windows=windows, order=order,
    )
    return FastClosedLoopScenario(
        track.name, "full", track, gates,
        tuple(segments for _, segments in primitive_definitions), vehicle,
        float(net_clearance),
    )


__all__ = ["FastClosedLoopScenario", "build_fast_closed_loop_scenario"]
