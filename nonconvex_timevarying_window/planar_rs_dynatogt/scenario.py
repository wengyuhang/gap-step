"""Deterministic six-window benchmark with compact, original curve models."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.boundary import (
    Bezier,
    BoundarySegment,
    CircularArc,
    DenseBoundary,
    Line,
)
from nonconvex_timevarying_window.sc_dynatogt.environment import (
    MotionProfile,
    SCDynamicWindow,
    SCWindowTrack,
)
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import (
    PreprocessedGate,
    PreprocessingConfig,
    preprocess_boundary,
)
from nonconvex_timevarying_window.sc_dynatogt.scenarios import Scenario

from .model import PlanarRSMotion, make_planar_problem


def _lines(points: list[tuple[float, float]]) -> tuple[Line, ...]:
    return tuple(
        Line(points[i], points[(i + 1) % len(points)]) for i in range(len(points))
    )


def _wavy_beziers(
    *, radius: float = 1.85, wave: float = 0.38, lobes: int = 3
) -> tuple[Bezier, ...]:
    """Closed polar wave represented by six native cubic Bezier pieces.

    The cubic control points are Hermite data of
    ``r(theta) = radius + wave*cos(lobes*theta)``.  The resulting Beziers,
    rather than their dense SC samples, are the official collision boundary.
    """

    count = 6
    step = 2.0 * math.pi / count

    def point_and_derivative(theta: float) -> tuple[np.ndarray, np.ndarray]:
        r = radius + wave * math.cos(lobes * theta)
        dr = -wave * lobes * math.sin(lobes * theta)
        c, s = math.cos(theta), math.sin(theta)
        return np.array([r * c, r * s]), np.array([dr * c - r * s, dr * s + r * c])

    pieces: list[Bezier] = []
    for index in range(count):
        theta0, theta1 = index * step, (index + 1) * step
        p0, d0 = point_and_derivative(theta0)
        p1, d1 = point_and_derivative(theta1)
        pieces.append(Bezier((p0, p0 + step * d0 / 3.0, p1 - step * d1 / 3.0, p1)))
    return tuple(pieces)


def benchmark_boundaries() -> tuple[tuple[str, tuple[BoundarySegment, ...]], ...]:
    """Return the exact primitive list used by both preprocessing and SIP."""

    l_shape = _lines(
        [(-2.0, -2.0), (2.0, -2.0), (2.0, -0.5), (0.5, -0.5), (0.5, 2.0), (-2.0, 2.0)]
    )
    u_shape = _lines(
        [
            (-2.5, -2.0),
            (2.5, -2.0),
            (2.5, 2.0),
            (0.8, 2.0),
            (0.8, 1.0),
            (-0.8, 1.0),
            (-0.8, 2.0),
            (-2.5, 2.0),
        ]
    )
    star_points: list[tuple[float, float]] = []
    for index in range(10):
        angle = -0.5 * math.pi + index * math.pi / 5.0
        radius = 2.5 if index % 2 == 0 else 1.15
        star_points.append((radius * math.cos(angle), radius * math.sin(angle)))
    star = _lines(star_points)
    circle = tuple(
        CircularArc((0.0, 0.0), 2.0, index * math.pi / 2.0, (index + 1) * math.pi / 2.0)
        for index in range(4)
    )
    wavy = _wavy_beziers()
    line_bezier = (
        Line((-2.25, -1.7), (2.25, -1.7)),
        Line((2.25, -1.7), (2.25, 1.65)),
        Bezier(((2.25, 1.65), (1.75, 1.65), (1.35, 0.20), (0.55, 0.30))),
        Bezier(((0.55, 0.30), (-0.15, 0.38), (-0.75, 1.65), (-2.25, 1.65))),
        Line((-2.25, 1.65), (-2.25, -1.7)),
    )
    return (
        ("L", l_shape),
        ("U", u_shape),
        ("star", star),
        ("circle", circle),
        ("wavy_bezier", wavy),
        ("line_bezier", line_bezier),
    )


def build_benchmark(
    preprocessing_config: PreprocessingConfig | None = None,
    *,
    cache_directory: str | Path | None = None,
) -> tuple[Scenario, object]:
    """Build an irregular closed course and its fixed-plane RS SIP problem."""

    primitives = benchmark_boundaries()
    centers = np.array(
        [
            [-2.4, -3.5, 6.4],
            [20.2, 14.5, 1.8],
            [20.2, -8.8, 2.2],
            [-9.9, -13.2, 6.3],
            [10.5, -2.0, 2.2],
            [-6.2, 15.0, 2.2],
        ]
    )
    angles = np.deg2rad(
        np.array(
            [
                [0.0, -90.0, 0.0],
                [0.0, -90.0, -20.0],
                [0.0, -90.0, -130.0],
                [0.0, -90.0, 180.0],
                [0.0, -90.0, 70.0],
                [0.0, -90.0, 200.0],
            ]
        )
    )
    endpoint = np.array([-16.0, 4.0, 3.2])
    settings = preprocessing_config or PreprocessingConfig(
        vertex_counts=(256,),
        # This map is only an optimization coordinate transform.  The final
        # collision proof evaluates the original primitives below, so a
        # slightly relaxed prevertex residual does not weaken certification.
        sc_fit_options={
            "quadrature_order": 32,
            "max_nfev": 1200,
            "parameter_tolerance": 5e-6,
        },
    )
    motions = tuple(
        PlanarRSMotion(
            angle_amplitude=0.95 + 0.13 * index,
            angle_period=2.8 + 0.31 * index,
            scale_amplitude=0.42,
            scale_period=3.2 + 0.27 * index,
            phase=0.55 * index,
        )
        for index in range(len(primitives))
    )
    cache_root = None if cache_directory is None else Path(cache_directory).expanduser()
    local_settings = replace(
        settings,
        offset_distance=settings.offset_distance
        / min(motion.minimum_scale for motion in motions),
    )
    gates: list[PreprocessedGate] = []
    for name, segments in primitives:
        cached = None if cache_root is None else cache_root / name
        if cached is not None and (cached / "manifest.json").is_file():
            gate = PreprocessedGate.load(cached)
            if gate.config != local_settings:
                raise ValueError(
                    f"cached preprocessing settings do not match for {name}"
                )
        else:
            gate = preprocess_boundary(
                DenseBoundary.from_segments(segments), name=name, config=local_settings
            )
            if cached is not None:
                gate.save(cached)
        gates.append(gate)
    windows = tuple(
        SCDynamicWindow(
            name=gate.name,
            sc_map=gate.sc_map,
            safe_polygon=gate.safe_polygon,
            center0=center,
            angles0=angle,
            motion=MotionProfile.static(),
            physical_boundary=gate.dense_boundary.vertices,
            required_world_clearance=settings.offset_distance,
            reference_local_clearance=gate.safe_region.distance,
        )
        for gate, center, angle in zip(gates, centers, angles)
    )
    track = SCWindowTrack(
        "planar_rs_irregular_closed",
        endpoint,
        endpoint,
        windows,
        tuple(range(len(windows))),
    )
    scenario = Scenario(track.name, "full", track, tuple(gates))
    problem = make_planar_problem(
        scenario.track,
        motions,
        tuple(segments for _, segments in primitives),
    )
    return scenario, problem


def build_ordinary(
    preprocessing_config: PreprocessingConfig | None = None,
    *,
    cache_directory: str | Path | None = None,
):
    """Build the reproducible one-window end-to-end timing case."""

    segments = tuple(
        CircularArc((0.0, 0.0), 2.0, index * math.pi / 2.0, (index + 1) * math.pi / 2.0)
        for index in range(4)
    )
    motion = PlanarRSMotion(
        angle_amplitude=1.25,
        angle_period=2.7,
        scale_amplitude=0.42,
        scale_period=3.1,
        phase=0.3,
    )
    settings = preprocessing_config or PreprocessingConfig(
        vertex_counts=(128,),
        sc_fit_options={"quadrature_order": 32, "parameter_tolerance": 5e-6},
    )
    local_settings = replace(
        settings, offset_distance=settings.offset_distance / motion.minimum_scale
    )
    cached = (
        None
        if cache_directory is None
        else Path(cache_directory).expanduser() / "circle"
    )
    if cached is not None and (cached / "manifest.json").is_file():
        gate = PreprocessedGate.load(cached)
        if gate.config != local_settings:
            raise ValueError("cached ordinary preprocessing settings do not match")
    else:
        gate = preprocess_boundary(
            DenseBoundary.from_segments(segments), name="circle", config=local_settings
        )
        if cached is not None:
            gate.save(cached)
    window = SCDynamicWindow(
        name="circle",
        sc_map=gate.sc_map,
        safe_polygon=gate.safe_polygon,
        center0=np.array([0.0, 0.0, 1.4]),
        angles0=np.array([0.0, math.pi / 2.0, 0.0]),
        motion=MotionProfile.static(),
        physical_boundary=gate.dense_boundary.vertices,
        required_world_clearance=settings.offset_distance,
        reference_local_clearance=gate.safe_region.distance,
    )
    track = SCWindowTrack(
        "planar_rs_ordinary_one",
        np.array([-4.0, 0.0, 1.4]),
        np.array([4.0, 0.0, 1.4]),
        (window,),
        (0,),
    )
    scenario = Scenario(track.name, "full", track, (gate,))
    return scenario, make_planar_problem(track, (motion,), (segments,))


__all__ = ["benchmark_boundaries", "build_benchmark", "build_ordinary"]
