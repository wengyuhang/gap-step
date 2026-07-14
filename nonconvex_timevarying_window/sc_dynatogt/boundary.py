"""Boundary ingestion and Chang-style uniform boundary resampling.

This module deliberately stops at boundary preprocessing.  The Chang--Gotsman--
Hormann work is used only for cumulative-arclength resampling with optional
corner retention; interior points are handled by the SC map in
:mod:`sc_mapping`.

All polygons are represented by an ``(n, 2)`` array without a repeated closing
vertex.  Edges nevertheless form a closed cycle, including ``vertices[-1]`` to
``vertices[0]``.
"""

from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol, Sequence, runtime_checkable

import numpy as np
from numpy.typing import ArrayLike, NDArray


FloatArray = NDArray[np.float64]

DENSE_CHORD_TOLERANCE = 0.001
DENSE_MAX_CHORD = 0.01
POLYGON_ERROR_TOLERANCE = 0.005
CONCAVITY_ERROR_TOLERANCE = 0.003
CONCAVITY_ANGLE_THRESHOLD_DEG = -5.0
ADAPTIVE_VERTEX_COUNTS = (256, 512, 1024, 2048, 3200)

_GEOMETRY_EPS = 1.0e-12
_CORNER_MATCH_TOLERANCE = 1.0e-9


class BoundaryError(ValueError):
    """Base class for malformed or inadmissible boundaries."""


class BoundaryValidationError(BoundaryError):
    """Raised when a ring is not a CCW, nonzero-area simple polygon."""


class BoundaryPreprocessError(BoundaryError):
    """Raised when no allowed adaptive vertex count meets the error rules."""

    def __init__(self, message: str, reports: Sequence["ResampleReport"] = ()) -> None:
        super().__init__(message)
        self.reports = tuple(reports)


def _point(value: ArrayLike, *, name: str = "point") -> FloatArray:
    out = np.asarray(value, dtype=np.float64)
    if out.shape != (2,) or not np.all(np.isfinite(out)):
        raise BoundaryError(f"{name} must be a finite two-vector")
    return out.copy()


def _vertices(value: ArrayLike, *, name: str = "vertices") -> FloatArray:
    out = np.asarray(value, dtype=np.float64)
    if out.ndim != 2 or out.shape[1:] != (2,) or len(out) < 3:
        raise BoundaryError(f"{name} must have shape (n, 2), n >= 3")
    if not np.all(np.isfinite(out)):
        raise BoundaryError(f"{name} contains NaN or infinity")
    out = out.copy()
    if len(out) > 1 and np.array_equal(out[0], out[-1]):
        out = out[:-1]
    if len(out) < 3:
        raise BoundaryError(f"{name} has fewer than three distinct ring positions")
    return out


def signed_area(vertices: ArrayLike) -> float:
    """Return the shoelace signed area (positive for counter-clockwise)."""

    p = _vertices(vertices)
    q = np.roll(p, -1, axis=0)
    return float(0.5 * np.sum(p[:, 0] * q[:, 1] - p[:, 1] * q[:, 0]))


@dataclass(frozen=True)
class PolygonValidation:
    """Validation facts for one closed ring."""

    vertex_count: int
    signed_area: float
    is_ccw: bool
    has_nonzero_area: bool
    has_nonzero_edges: bool
    is_simple: bool
    has_holes: bool
    valid: bool
    errors: tuple[str, ...] = ()


def _fallback_is_simple(vertices: FloatArray) -> bool:
    """Dependency-free O(n^2) fallback used only if Shapely is unavailable."""

    def orient(a: FloatArray, b: FloatArray, c: FloatArray) -> float:
        return float(np.cross(b - a, c - a))

    def on_segment(a: FloatArray, p: FloatArray, b: FloatArray) -> bool:
        return (
            min(a[0], b[0]) - _GEOMETRY_EPS <= p[0] <= max(a[0], b[0]) + _GEOMETRY_EPS
            and min(a[1], b[1]) - _GEOMETRY_EPS <= p[1] <= max(a[1], b[1]) + _GEOMETRY_EPS
            and abs(orient(a, p, b)) <= _GEOMETRY_EPS
        )

    def intersects(a: FloatArray, b: FloatArray, c: FloatArray, d: FloatArray) -> bool:
        o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
        if ((o1 > _GEOMETRY_EPS and o2 < -_GEOMETRY_EPS) or (o1 < -_GEOMETRY_EPS and o2 > _GEOMETRY_EPS)) and (
            (o3 > _GEOMETRY_EPS and o4 < -_GEOMETRY_EPS) or (o3 < -_GEOMETRY_EPS and o4 > _GEOMETRY_EPS)
        ):
            return True
        return (
            (abs(o1) <= _GEOMETRY_EPS and on_segment(a, c, b))
            or (abs(o2) <= _GEOMETRY_EPS and on_segment(a, d, b))
            or (abs(o3) <= _GEOMETRY_EPS and on_segment(c, a, d))
            or (abs(o4) <= _GEOMETRY_EPS and on_segment(c, b, d))
        )

    n = len(vertices)
    for i in range(n):
        a, b = vertices[i], vertices[(i + 1) % n]
        for j in range(i + 1, n):
            if i == j or (i + 1) % n == j or (j + 1) % n == i:
                continue
            if intersects(a, b, vertices[j], vertices[(j + 1) % n]):
                return False
    return True


def validate_polygon(
    vertices: ArrayLike,
    *,
    require_ccw: bool = True,
    raise_on_error: bool = False,
    area_epsilon: float = _GEOMETRY_EPS,
) -> PolygonValidation:
    """Validate closure semantics, simplicity, orientation, and nonzero area.

    A single ring cannot encode a legitimate hole.  Shapely is used only as a
    robust validator when installed; it is never used for the inward offset.
    """

    p = _vertices(vertices)
    lengths = np.linalg.norm(np.roll(p, -1, axis=0) - p, axis=1)
    has_nonzero_edges = bool(np.all(lengths > _GEOMETRY_EPS))
    area = signed_area(p)
    has_nonzero_area = abs(area) > float(area_epsilon)
    is_ccw = area > 0.0
    has_holes = False
    try:
        from shapely.geometry import LinearRing, Polygon

        ring = LinearRing(p)
        polygon = Polygon(ring)
        is_simple = bool(ring.is_simple and polygon.is_valid)
        has_holes = bool(len(polygon.interiors))
    except ImportError:  # pragma: no cover - normal project dependencies include Shapely
        is_simple = _fallback_is_simple(p)

    errors: list[str] = []
    if not has_nonzero_edges:
        errors.append("boundary contains a zero-length edge")
    if not has_nonzero_area:
        errors.append("boundary area is zero")
    if require_ccw and not is_ccw:
        errors.append("boundary is not counter-clockwise")
    if not is_simple:
        errors.append("boundary is self-intersecting or otherwise invalid")
    if has_holes:
        errors.append("boundary contains holes")
    result = PolygonValidation(
        vertex_count=len(p),
        signed_area=area,
        is_ccw=is_ccw,
        has_nonzero_area=has_nonzero_area,
        has_nonzero_edges=has_nonzero_edges,
        is_simple=is_simple,
        has_holes=has_holes,
        valid=not errors,
        errors=tuple(errors),
    )
    if raise_on_error and not result.valid:
        raise BoundaryValidationError("; ".join(result.errors))
    return result


@runtime_checkable
class BoundarySegment(Protocol):
    """Minimal common interface for a parameterized boundary segment."""

    def evaluate(self, u: float) -> FloatArray: ...

    def is_straight(self) -> bool: ...

    def preserve_start_as_corner(self) -> bool: ...

    def preserve_end_as_corner(self) -> bool: ...


def _unit_parameter(u: float) -> float:
    value = float(u)
    if not math.isfinite(value) or value < -_GEOMETRY_EPS or value > 1.0 + _GEOMETRY_EPS:
        raise BoundaryError("curve parameter must lie in [0, 1]")
    return min(1.0, max(0.0, value))


@dataclass(frozen=True)
class Line:
    """Straight segment with endpoints included in ``u in [0, 1]``."""

    start: ArrayLike
    end: ArrayLike
    preserve_start: bool = True
    preserve_end: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(self, "start", _point(self.start, name="start"))
        object.__setattr__(self, "end", _point(self.end, name="end"))
        if np.linalg.norm(self.end - self.start) <= _GEOMETRY_EPS:
            raise BoundaryError("line endpoints must be different")

    def evaluate(self, u: float) -> FloatArray:
        t = _unit_parameter(u)
        return (1.0 - t) * self.start + t * self.end

    def is_straight(self) -> bool:
        return True

    def preserve_start_as_corner(self) -> bool:
        return bool(self.preserve_start)

    def preserve_end_as_corner(self) -> bool:
        return bool(self.preserve_end)

    # The design document gives the C++ spellings; retain them as compatibility aliases.
    isStraight = is_straight
    preserveStartAsCorner = preserve_start_as_corner
    preserveEndAsCorner = preserve_end_as_corner


@dataclass(frozen=True)
class CircularArc:
    """Circular arc parameterized by center, radius, and endpoint angles."""

    center: ArrayLike
    radius: float
    start_angle: float
    end_angle: float
    ccw: bool = True
    preserve_start: bool = False
    preserve_end: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "center", _point(self.center, name="center"))
        radius = float(self.radius)
        if not math.isfinite(radius) or radius <= 0.0:
            raise BoundaryError("arc radius must be positive and finite")
        object.__setattr__(self, "radius", radius)
        start, end = float(self.start_angle), float(self.end_angle)
        if not math.isfinite(start) or not math.isfinite(end):
            raise BoundaryError("arc angles must be finite")
        sweep = end - start
        if self.ccw:
            while sweep <= 0.0:
                sweep += 2.0 * math.pi
        else:
            while sweep >= 0.0:
                sweep -= 2.0 * math.pi
        object.__setattr__(self, "_sweep", sweep)

    def evaluate(self, u: float) -> FloatArray:
        angle = float(self.start_angle) + _unit_parameter(u) * self._sweep
        return self.center + self.radius * np.asarray([math.cos(angle), math.sin(angle)], dtype=np.float64)

    def is_straight(self) -> bool:
        return False

    def preserve_start_as_corner(self) -> bool:
        return bool(self.preserve_start)

    def preserve_end_as_corner(self) -> bool:
        return bool(self.preserve_end)

    isStraight = is_straight
    preserveStartAsCorner = preserve_start_as_corner
    preserveEndAsCorner = preserve_end_as_corner


@dataclass(frozen=True)
class Bezier:
    """Bézier segment of arbitrary degree, evaluated by de Casteljau."""

    control_points: ArrayLike
    preserve_start: bool = False
    preserve_end: bool = False

    def __post_init__(self) -> None:
        points = np.asarray(self.control_points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1:] != (2,) or len(points) < 2 or not np.all(np.isfinite(points)):
            raise BoundaryError("Bezier control_points must have shape (n, 2), n >= 2")
        object.__setattr__(self, "control_points", points.copy())

    def evaluate(self, u: float) -> FloatArray:
        t = _unit_parameter(u)
        work = self.control_points.copy()
        for count in range(len(work) - 1, 0, -1):
            work[:count] = (1.0 - t) * work[:count] + t * work[1 : count + 1]
        return work[0].copy()

    def is_straight(self) -> bool:
        return False

    def preserve_start_as_corner(self) -> bool:
        return bool(self.preserve_start)

    def preserve_end_as_corner(self) -> bool:
        return bool(self.preserve_end)

    isStraight = is_straight
    preserveStartAsCorner = preserve_start_as_corner
    preserveEndAsCorner = preserve_end_as_corner


@dataclass(frozen=True)
class BSpline:
    """Non-rational B-spline segment with an optional clamped knot vector."""

    control_points: ArrayLike
    degree: int = 3
    knots: ArrayLike | None = None
    preserve_start: bool = False
    preserve_end: bool = False
    _spline: object = field(init=False, repr=False, compare=False)
    _domain: tuple[float, float] = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        points = np.asarray(self.control_points, dtype=np.float64)
        degree = int(self.degree)
        if points.ndim != 2 or points.shape[1:] != (2,) or not np.all(np.isfinite(points)):
            raise BoundaryError("BSpline control_points must have shape (n, 2)")
        if degree < 1 or len(points) < degree + 1:
            raise BoundaryError("BSpline requires degree >= 1 and at least degree + 1 control points")
        if self.knots is None:
            interior_count = len(points) - degree - 1
            interior = np.linspace(0.0, 1.0, interior_count + 2, dtype=np.float64)[1:-1]
            knots = np.concatenate((np.zeros(degree + 1), interior, np.ones(degree + 1)))
        else:
            knots = np.asarray(self.knots, dtype=np.float64)
        if knots.shape != (len(points) + degree + 1,) or not np.all(np.isfinite(knots)) or np.any(np.diff(knots) < 0.0):
            raise BoundaryError("invalid BSpline knot vector")
        lo, hi = float(knots[degree]), float(knots[-degree - 1])
        if hi - lo <= _GEOMETRY_EPS:
            raise BoundaryError("BSpline parameter domain is empty")
        try:
            from scipy.interpolate import BSpline as SciPyBSpline
        except ImportError as exc:  # pragma: no cover - SciPy is a declared project dependency
            raise ImportError("BSpline boundary support requires scipy") from exc
        object.__setattr__(self, "control_points", points.copy())
        object.__setattr__(self, "degree", degree)
        object.__setattr__(self, "knots", knots.copy())
        object.__setattr__(self, "_domain", (lo, hi))
        object.__setattr__(self, "_spline", SciPyBSpline(knots, points, degree, extrapolate=False))

    def evaluate(self, u: float) -> FloatArray:
        t = _unit_parameter(u)
        lo, hi = self._domain
        return np.asarray(self._spline(lo + t * (hi - lo)), dtype=np.float64)

    def is_straight(self) -> bool:
        return False

    def preserve_start_as_corner(self) -> bool:
        return bool(self.preserve_start)

    def preserve_end_as_corner(self) -> bool:
        return bool(self.preserve_end)

    isStraight = is_straight
    preserveStartAsCorner = preserve_start_as_corner
    preserveEndAsCorner = preserve_end_as_corner


def _distance_to_segment(point: FloatArray, a: FloatArray, b: FloatArray) -> float:
    edge = b - a
    denominator = float(np.dot(edge, edge))
    if denominator <= _GEOMETRY_EPS:
        return float(np.linalg.norm(point - a))
    t = float(np.clip(np.dot(point - a, edge) / denominator, 0.0, 1.0))
    return float(np.linalg.norm(point - (a + t * edge)))


def _densify_segment(
    segment: BoundarySegment,
    *,
    chord_tolerance: float,
    max_chord: float,
    max_depth: int,
) -> FloatArray:
    start, end = _point(segment.evaluate(0.0)), _point(segment.evaluate(1.0))
    output: list[FloatArray] = [start]

    def recurse(ua: float, a: FloatArray, ub: float, b: FloatArray, depth: int) -> None:
        uc = 0.5 * (ua + ub)
        c = _point(segment.evaluate(uc))
        chord_error = _distance_to_segment(c, a, b)
        if chord_error <= chord_tolerance and np.linalg.norm(b - a) <= max_chord:
            output.append(b)
            return
        if depth >= max_depth:
            raise BoundaryPreprocessError(
                f"curve densification did not reach chord tolerance {chord_tolerance:g} "
                f"and maximum chord {max_chord:g} within {max_depth} levels"
            )
        recurse(ua, a, uc, c, depth + 1)
        recurse(uc, c, ub, b, depth + 1)

    recurse(0.0, start, 1.0, end, 0)
    return np.asarray(output, dtype=np.float64)


def _read_xy_csv(path: str | Path, *, min_points: int = 3) -> FloatArray:
    rows: list[tuple[float, float]] = []
    with Path(path).expanduser().open("r", encoding="utf-8-sig", newline="") as stream:
        for line_number, row in enumerate(csv.reader(stream), start=1):
            if not row or all(not value.strip() for value in row) or row[0].lstrip().startswith("#"):
                continue
            if len(row) < 2:
                raise BoundaryError(f"{path}:{line_number}: expected x,y")
            try:
                x, y = float(row[0]), float(row[1])
            except ValueError:
                if not rows and row[0].strip().lower() == "x" and row[1].strip().lower() == "y":
                    continue
                raise BoundaryError(f"{path}:{line_number}: x and y must be numeric") from None
            if not math.isfinite(x) or not math.isfinite(y):
                raise BoundaryError(f"{path}:{line_number}: x and y must be finite")
            rows.append((x, y))
    if len(rows) < min_points:
        raise BoundaryError(f"{path}: expected at least {min_points} point(s)")
    return np.asarray(rows, dtype=np.float64)


def _match_corner_indices(vertices: FloatArray, corners: FloatArray, tolerance: float = _CORNER_MATCH_TOLERANCE) -> tuple[int, ...]:
    indices: list[int] = []
    for corner in corners:
        distances = np.linalg.norm(vertices - corner, axis=1)
        index = int(np.argmin(distances))
        if distances[index] > tolerance:
            raise BoundaryError(f"forced corner {corner.tolist()} is not on a dense-boundary vertex")
        if index not in indices:
            indices.append(index)
    return tuple(indices)


@dataclass(frozen=True)
class DenseBoundary:
    """Validated dense CCW ring and its exact, forced corner vertices."""

    vertices: ArrayLike
    corners: ArrayLike = field(default_factory=lambda: np.empty((0, 2), dtype=np.float64))
    corner_indices: tuple[int, ...] | None = None

    def __post_init__(self) -> None:
        vertices = _vertices(self.vertices)
        validate_polygon(vertices, require_ccw=True, raise_on_error=True)
        corners = np.asarray(self.corners, dtype=np.float64)
        if corners.size == 0:
            corners = np.empty((0, 2), dtype=np.float64)
        if corners.ndim != 2 or corners.shape[1:] != (2,) or not np.all(np.isfinite(corners)):
            raise BoundaryError("corners must have shape (r, 2)")
        if self.corner_indices is None:
            indices = _match_corner_indices(vertices, corners)
        else:
            indices = tuple(int(i) for i in self.corner_indices)
            if len(indices) != len(corners) or any(i < 0 or i >= len(vertices) for i in indices):
                raise BoundaryError("corner_indices must match corners and index dense vertices")
            if any(np.linalg.norm(vertices[i] - corner) > _CORNER_MATCH_TOLERANCE for i, corner in zip(indices, corners)):
                raise BoundaryError("corner_indices do not identify the supplied corner coordinates")
        order = np.argsort(np.asarray(indices, dtype=np.int64)) if indices else np.empty(0, dtype=np.int64)
        indices = tuple(indices[int(i)] for i in order)
        corners = corners[order] if len(order) else corners
        if len(set(indices)) != len(indices):
            raise BoundaryError("forced corners must be distinct")
        object.__setattr__(self, "vertices", vertices)
        object.__setattr__(self, "corners", corners.copy())
        object.__setattr__(self, "corner_indices", indices)

    @property
    def corner_mask(self) -> NDArray[np.bool_]:
        mask = np.zeros(len(self.vertices), dtype=bool)
        mask[list(self.corner_indices)] = True
        return mask

    @classmethod
    def from_csv(
        cls,
        path: str | Path,
        *,
        corners_path: str | Path | None = None,
        corner_indices: Sequence[int] | None = None,
    ) -> "DenseBoundary":
        vertices = _read_xy_csv(path)
        if corners_path is not None and corner_indices is not None:
            raise BoundaryError("provide corners_path or corner_indices, not both")
        if corners_path is not None:
            corners = _read_xy_csv(corners_path, min_points=1)
            indices = None
        elif corner_indices is not None:
            indices = tuple(int(i) for i in corner_indices)
            corners = vertices[np.asarray(indices, dtype=np.int64)]
        else:
            corners, indices = np.empty((0, 2), dtype=np.float64), ()
        return cls(vertices, corners, indices)

    @classmethod
    def from_segments(
        cls,
        segments: Sequence[BoundarySegment],
        *,
        chord_tolerance: float = DENSE_CHORD_TOLERANCE,
        max_chord: float = DENSE_MAX_CHORD,
        max_depth: int = 32,
        continuity_tolerance: float = 1.0e-8,
    ) -> "DenseBoundary":
        return densify_boundary(
            segments,
            chord_tolerance=chord_tolerance,
            max_chord=max_chord,
            max_depth=max_depth,
            continuity_tolerance=continuity_tolerance,
        )


def densify_boundary(
    segments: Sequence[BoundarySegment],
    *,
    chord_tolerance: float = DENSE_CHORD_TOLERANCE,
    max_chord: float = DENSE_MAX_CHORD,
    max_depth: int = 32,
    continuity_tolerance: float = 1.0e-8,
) -> DenseBoundary:
    """Recursively densify every curve using the document's one common rule."""

    if not segments:
        raise BoundaryError("at least one boundary segment is required")
    if chord_tolerance <= 0.0 or max_chord <= 0.0:
        raise BoundaryError("densification tolerances must be positive")
    pieces: list[FloatArray] = []
    endpoints: list[tuple[FloatArray, FloatArray]] = []
    for index, segment in enumerate(segments):
        if not isinstance(segment, BoundarySegment):
            raise BoundaryError(f"segment {index} does not implement the boundary-segment interface")
        piece = _densify_segment(
            segment,
            chord_tolerance=float(chord_tolerance),
            max_chord=float(max_chord),
            max_depth=int(max_depth),
        )
        pieces.append(piece)
        endpoints.append((piece[0], piece[-1]))
    for index, (_, end) in enumerate(endpoints):
        next_start = endpoints[(index + 1) % len(endpoints)][0]
        if np.linalg.norm(end - next_start) > continuity_tolerance:
            raise BoundaryValidationError(f"segments {index} and {(index + 1) % len(segments)} do not meet")

    combined = [pieces[0]]
    combined.extend(piece[1:] for piece in pieces[1:])
    vertices = np.concatenate(combined, axis=0)
    if np.linalg.norm(vertices[-1] - vertices[0]) <= continuity_tolerance:
        vertices = vertices[:-1]

    corner_indices: list[int] = []
    corner_points: list[FloatArray] = []
    offset = 0
    for index, (segment, piece) in enumerate(zip(segments, pieces)):
        # A join is a true corner if either adjacent segment explicitly says so.
        previous = segments[(index - 1) % len(segments)]
        if segment.preserve_start_as_corner() or previous.preserve_end_as_corner():
            corner_indices.append(offset)
            corner_points.append(piece[0])
        offset += len(piece) - 1
    # The final closing endpoint is omitted, so all offsets already address vertices.
    unique: dict[int, FloatArray] = {}
    for index, point in zip(corner_indices, corner_points):
        unique[index % len(vertices)] = point
    ordered_indices = tuple(sorted(unique))
    corners = np.asarray([vertices[i] for i in ordered_indices], dtype=np.float64)
    return DenseBoundary(vertices, corners, ordered_indices)


def cumulative_arclength(vertices: ArrayLike) -> tuple[FloatArray, float]:
    """Return ``s[0]=0 ... s[n]=L`` for all edges of the closed ring."""

    p = _vertices(vertices)
    lengths = np.linalg.norm(np.roll(p, -1, axis=0) - p, axis=1)
    if np.any(lengths <= _GEOMETRY_EPS):
        raise BoundaryValidationError("boundary contains a zero-length edge")
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    return cumulative, float(cumulative[-1])


def _interpolate_arclength(vertices: FloatArray, cumulative: FloatArray, length: float, positions: ArrayLike) -> FloatArray:
    tau = np.mod(np.asarray(positions, dtype=np.float64), length)
    indices = np.searchsorted(cumulative, tau, side="right") - 1
    indices = np.clip(indices, 0, len(vertices) - 1)
    edge_lengths = cumulative[indices + 1] - cumulative[indices]
    fraction = (tau - cumulative[indices]) / edge_lengths
    return (1.0 - fraction[:, None]) * vertices[indices] + fraction[:, None] * vertices[(indices + 1) % len(vertices)]


def _largest_remainder_quotas(total: int, interval_lengths: FloatArray) -> NDArray[np.int64]:
    exact = total * interval_lengths / float(np.sum(interval_lengths))
    quotas = np.floor(exact).astype(np.int64)
    missing = int(total - np.sum(quotas))
    if missing:
        # Stable sorting makes ties deterministic in boundary order.
        ranking = np.argsort(-(exact - quotas), kind="stable")
        quotas[ranking[:missing]] += 1
    return quotas


@dataclass(frozen=True)
class ResampleReport:
    """Acceptance evidence for one candidate value of ``m``."""

    target_count: int
    max_boundary_error: float
    max_concavity_error: float
    concave_probe_count: int
    corners_preserved: bool
    is_simple: bool
    is_ccw: bool
    has_nonzero_area: bool
    has_holes: bool
    accepted: bool
    elapsed_seconds: float
    failure_reasons: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, object]:
        return {
            "target_count": self.target_count,
            "max_boundary_error": self.max_boundary_error,
            "max_concavity_error": self.max_concavity_error,
            "concave_probe_count": self.concave_probe_count,
            "corners_preserved": self.corners_preserved,
            "is_simple": self.is_simple,
            "is_ccw": self.is_ccw,
            "has_nonzero_area": self.has_nonzero_area,
            "has_holes": self.has_holes,
            "accepted": self.accepted,
            "elapsed_seconds": self.elapsed_seconds,
            "failure_reasons": self.failure_reasons,
        }


@dataclass(frozen=True)
class SampledBoundary:
    """A Chang-resampled polygon plus exact corner provenance and reports."""

    vertices: ArrayLike
    corner_mask: ArrayLike
    corners: ArrayLike
    source_vertex_count: int
    reports: tuple[ResampleReport, ...] = ()

    def __post_init__(self) -> None:
        vertices = _vertices(self.vertices)
        mask = np.asarray(self.corner_mask, dtype=bool)
        corners = np.asarray(self.corners, dtype=np.float64)
        if mask.shape != (len(vertices),):
            raise BoundaryError("corner_mask must have one value per sampled vertex")
        if corners.size == 0:
            corners = np.empty((0, 2), dtype=np.float64)
        if corners.ndim != 2 or corners.shape[1:] != (2,):
            raise BoundaryError("corners must have shape (r, 2)")
        object.__setattr__(self, "vertices", vertices)
        object.__setattr__(self, "corner_mask", mask.copy())
        object.__setattr__(self, "corners", corners.copy())

    @property
    def m(self) -> int:
        return len(self.vertices)

    @property
    def report(self) -> ResampleReport | None:
        return self.reports[-1] if self.reports else None

    @property
    def corner_indices(self) -> tuple[int, ...]:
        return tuple(int(i) for i in np.flatnonzero(self.corner_mask))

    def __array__(self, dtype: np.dtype | None = None) -> FloatArray:
        return np.asarray(self.vertices, dtype=dtype)


def chang_uniform_resample(
    boundary: DenseBoundary | ArrayLike,
    target_count: int,
    *,
    corners: ArrayLike | None = None,
) -> SampledBoundary:
    """Uniformly sample a closed ring by cumulative arclength.

    Forced corners divide the cycle into intervals.  ``m-r`` interior samples
    are apportioned by interval length with the largest-remainder method, then
    placed uniformly inside each interval.  Corner values are copied verbatim.
    """

    if isinstance(boundary, DenseBoundary):
        dense = boundary.vertices
        forced = boundary.corners if corners is None else np.asarray(corners, dtype=np.float64)
        forced_indices = boundary.corner_indices if corners is None else _match_corner_indices(dense, forced)
    else:
        dense = _vertices(boundary)
        forced = np.empty((0, 2), dtype=np.float64) if corners is None else np.asarray(corners, dtype=np.float64)
        if forced.size == 0:
            forced = np.empty((0, 2), dtype=np.float64)
        if forced.ndim != 2 or forced.shape[1:] != (2,):
            raise BoundaryError("corners must have shape (r, 2)")
        forced_indices = _match_corner_indices(dense, forced)
    validate_polygon(dense, require_ccw=True, raise_on_error=True)
    m = int(target_count)
    if m < 3:
        raise BoundaryError("target_count must be at least three")
    if m < len(forced_indices):
        raise BoundaryError("target_count is smaller than the number of forced corners")

    cumulative, total_length = cumulative_arclength(dense)
    if not forced_indices:
        positions = np.arange(m, dtype=np.float64) * total_length / m
        sampled = _interpolate_arclength(dense, cumulative, total_length, positions)
        mask = np.zeros(m, dtype=bool)
        return SampledBoundary(sampled, mask, forced, len(dense))

    order = np.argsort(np.asarray(forced_indices, dtype=np.int64), kind="stable")
    indices = np.asarray(forced_indices, dtype=np.int64)[order]
    forced = forced[order]
    sigmas = cumulative[indices]
    interval_lengths = np.mod(np.roll(sigmas, -1) - sigmas, total_length)
    if np.any(interval_lengths <= _GEOMETRY_EPS):
        raise BoundaryError("forced corners must occupy distinct arclength positions")
    quotas = _largest_remainder_quotas(m - len(forced), interval_lengths)

    output: list[FloatArray] = []
    corner_flags: list[bool] = []
    for corner, sigma, interval_length, quota in zip(forced, sigmas, interval_lengths, quotas):
        output.append(corner.copy())
        corner_flags.append(True)
        if quota:
            positions = sigma + np.arange(1, int(quota) + 1, dtype=np.float64) * interval_length / (int(quota) + 1)
            interior = _interpolate_arclength(dense, cumulative, total_length, positions)
            output.extend(interior)
            corner_flags.extend([False] * len(interior))
    sampled = np.asarray(output, dtype=np.float64)
    if len(sampled) != m:  # defensive assertion around quota arithmetic
        raise RuntimeError(f"resampling produced {len(sampled)} vertices, expected {m}")
    return SampledBoundary(sampled, np.asarray(corner_flags, dtype=bool), forced, len(dense))


def distances_to_polygon_boundary(points: ArrayLike, polygon: ArrayLike) -> FloatArray:
    """Exact point-to-segment distances, vectorized in memory-bounded blocks."""

    probes = np.asarray(points, dtype=np.float64)
    edges_a = _vertices(polygon)
    if probes.ndim != 2 or probes.shape[1:] != (2,) or not np.all(np.isfinite(probes)):
        raise BoundaryError("points must have shape (n, 2) and be finite")
    edges_b = np.roll(edges_a, -1, axis=0)
    edge = edges_b - edges_a
    denominator = np.sum(edge * edge, axis=1)
    if np.any(denominator <= _GEOMETRY_EPS):
        raise BoundaryValidationError("polygon contains a zero-length edge")
    result = np.empty(len(probes), dtype=np.float64)
    block_size = max(1, min(len(probes), 1_000_000 // len(edges_a)))
    for start in range(0, len(probes), block_size):
        block = probes[start : start + block_size]
        delta = block[:, None, :] - edges_a[None, :, :]
        fraction = np.clip(np.sum(delta * edge[None, :, :], axis=2) / denominator[None, :], 0.0, 1.0)
        nearest_delta = delta - fraction[:, :, None] * edge[None, :, :]
        squared = np.sum(nearest_delta * nearest_delta, axis=2)
        result[start : start + len(block)] = np.sqrt(np.min(squared, axis=1))
    return result


def signed_turn_angles(vertices: ArrayLike) -> FloatArray:
    """Signed local turning angles in radians; a CCW-ring concavity is negative."""

    p = _vertices(vertices)
    incoming = p - np.roll(p, 1, axis=0)
    outgoing = np.roll(p, -1, axis=0) - p
    cross = incoming[:, 0] * outgoing[:, 1] - incoming[:, 1] * outgoing[:, 0]
    dot = np.sum(incoming * outgoing, axis=1)
    return np.arctan2(cross, dot)


def _evaluate_resample(
    dense: DenseBoundary,
    sampled: SampledBoundary,
    *,
    boundary_tolerance: float,
    concavity_tolerance: float,
    concavity_angle_threshold_deg: float,
    elapsed_seconds: float,
) -> ResampleReport:
    validation = validate_polygon(sampled.vertices, require_ccw=True)
    distances = distances_to_polygon_boundary(dense.vertices, sampled.vertices)
    max_error = float(np.max(distances))
    concave_mask = (signed_turn_angles(dense.vertices) < math.radians(concavity_angle_threshold_deg)) & ~dense.corner_mask
    max_concavity_error = float(np.max(distances[concave_mask])) if np.any(concave_mask) else 0.0
    corner_values = sampled.vertices[sampled.corner_mask]
    corners_preserved = len(corner_values) == len(dense.corners) and all(
        any(np.array_equal(value, corner) for value in corner_values) for corner in dense.corners
    )
    failures: list[str] = []
    if not validation.is_simple:
        failures.append("sampled polygon is not simple")
    if not validation.is_ccw:
        failures.append("sampled polygon is not counter-clockwise")
    if not validation.has_nonzero_area:
        failures.append("sampled polygon has zero area")
    if validation.has_holes:
        failures.append("sampled polygon has holes")
    if not corners_preserved:
        failures.append("not all forced corners were preserved exactly")
    if max_error > boundary_tolerance:
        failures.append(f"maximum boundary error {max_error:.9g} exceeds {boundary_tolerance:.9g}")
    if max_concavity_error > concavity_tolerance:
        failures.append(f"maximum non-corner concavity error {max_concavity_error:.9g} exceeds {concavity_tolerance:.9g}")
    return ResampleReport(
        target_count=sampled.m,
        max_boundary_error=max_error,
        max_concavity_error=max_concavity_error,
        concave_probe_count=int(np.count_nonzero(concave_mask)),
        corners_preserved=corners_preserved,
        is_simple=validation.is_simple,
        is_ccw=validation.is_ccw,
        has_nonzero_area=validation.has_nonzero_area,
        has_holes=validation.has_holes,
        accepted=not failures,
        elapsed_seconds=float(elapsed_seconds),
        failure_reasons=tuple(failures),
    )


def adaptive_chang_resample(
    boundary: DenseBoundary | ArrayLike,
    *,
    corners: ArrayLike | None = None,
    vertex_counts: Sequence[int] = ADAPTIVE_VERTEX_COUNTS,
    boundary_tolerance: float = POLYGON_ERROR_TOLERANCE,
    concavity_tolerance: float = CONCAVITY_ERROR_TOLERANCE,
    concavity_angle_threshold_deg: float = CONCAVITY_ANGLE_THRESHOLD_DEG,
) -> SampledBoundary:
    """Try ``256,512,1024,2048,3200`` and return the first accepted polygon.

    If the final candidate still violates the 5 mm global or 3 mm concavity
    rule (or topology/corner checks), :class:`BoundaryPreprocessError` marks the
    preprocessing failure and exposes all attempt reports.
    """

    if isinstance(boundary, DenseBoundary):
        dense = boundary if corners is None else DenseBoundary(boundary.vertices, np.asarray(corners, dtype=np.float64))
    else:
        dense = DenseBoundary(
            boundary,
            np.empty((0, 2), dtype=np.float64) if corners is None else np.asarray(corners, dtype=np.float64),
        )
    counts = tuple(int(value) for value in vertex_counts)
    if not counts or any(value < 3 for value in counts) or any(b <= a for a, b in zip(counts, counts[1:])):
        raise BoundaryError("vertex_counts must be a nonempty strictly increasing sequence of counts >= 3")
    reports: list[ResampleReport] = []
    for count in counts:
        started = time.perf_counter()
        sampled = chang_uniform_resample(dense, count)
        report = _evaluate_resample(
            dense,
            sampled,
            boundary_tolerance=float(boundary_tolerance),
            concavity_tolerance=float(concavity_tolerance),
            concavity_angle_threshold_deg=float(concavity_angle_threshold_deg),
            elapsed_seconds=time.perf_counter() - started,
        )
        reports.append(report)
        if report.accepted:
            return SampledBoundary(
                sampled.vertices,
                sampled.corner_mask,
                sampled.corners,
                sampled.source_vertex_count,
                tuple(reports),
            )
    summary = "; ".join(f"m={report.target_count}: {', '.join(report.failure_reasons)}" for report in reports)
    raise BoundaryPreprocessError(f"boundary preprocessing failed through m={counts[-1]} ({summary})", reports)


# Descriptive aliases used by callers and the command-line layer.
LineSegment = Line
CircularArcSegment = CircularArc
BezierSegment = Bezier
BSplineSegment = BSpline
load_boundary_csv = DenseBoundary.from_csv
densify_segments = densify_boundary
uniform_resample = chang_uniform_resample
resample_boundary = chang_uniform_resample
adaptive_resample = adaptive_chang_resample


__all__ = [
    "ADAPTIVE_VERTEX_COUNTS",
    "BSpline",
    "BSplineSegment",
    "Bezier",
    "BezierSegment",
    "BoundaryError",
    "BoundaryPreprocessError",
    "BoundarySegment",
    "BoundaryValidationError",
    "CONCAVITY_ANGLE_THRESHOLD_DEG",
    "CONCAVITY_ERROR_TOLERANCE",
    "CircularArc",
    "CircularArcSegment",
    "DENSE_CHORD_TOLERANCE",
    "DENSE_MAX_CHORD",
    "DenseBoundary",
    "Line",
    "LineSegment",
    "POLYGON_ERROR_TOLERANCE",
    "PolygonValidation",
    "ResampleReport",
    "SampledBoundary",
    "adaptive_chang_resample",
    "adaptive_resample",
    "chang_uniform_resample",
    "cumulative_arclength",
    "densify_boundary",
    "densify_segments",
    "distances_to_polygon_boundary",
    "load_boundary_csv",
    "resample_boundary",
    "signed_area",
    "signed_turn_angles",
    "uniform_resample",
    "validate_polygon",
]
