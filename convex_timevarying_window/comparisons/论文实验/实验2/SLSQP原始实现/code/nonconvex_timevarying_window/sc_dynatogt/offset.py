"""Clipper2-based construction and validation of the safe inward polygon."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .boundary import (
    BoundaryError,
    PolygonValidation,
    SampledBoundary,
    signed_turn_angles,
    validate_polygon,
)


FloatArray = NDArray[np.float64]

PHYSICAL_MARGIN = 0.300
GEOMETRIC_MARGIN = 0.005
NUMERICAL_MARGIN = 0.010
DEFAULT_INWARD_OFFSET = PHYSICAL_MARGIN + GEOMETRIC_MARGIN + NUMERICAL_MARGIN
DEFAULT_MITER_LIMIT = 2.0
DEFAULT_MIN_SAFE_AREA = 0.1
DEFAULT_ARC_TOLERANCE = 0.001
DEFAULT_CLIPPER_SCALE = 1_000_000.0


class OffsetError(RuntimeError):
    """Base class for safe-polygon construction failures."""


class OffsetDependencyError(OffsetError):
    """Raised when the required Clipper2 Python binding is unavailable."""


class OffsetValidationError(OffsetError):
    """Raised when the offset violates the experiment's topology/area rules."""

    def __init__(self, message: str, diagnostics: "OffsetDiagnostics") -> None:
        super().__init__(message)
        self.diagnostics = diagnostics


@dataclass(frozen=True)
class OffsetMetadata:
    """Join provenance, including pyclipr's per-path join limitation.

    Clipper2 supports one join type per added path, not one type per vertex.  A
    negative offset makes convex input corners the intersection of inward edge
    offsets for both Round and Miter, so global Round is exact when all forced
    corners are convex.  If a forced reflex corner needs a true Miter join, the
    whole path is submitted as Miter; densely sampled smooth vertices then form
    a documented high-resolution miter approximation rather than being replaced
    by a Shapely-buffer algorithm.
    """

    backend: str
    requested_corner_join: str
    requested_smooth_join: str
    applied_path_join: str
    join_strategy: str
    per_vertex_join_supported: bool
    per_vertex_request_exact: bool
    corner_count: int
    reflex_corner_count: int
    smooth_vertex_count: int
    miter_limit: float
    arc_tolerance: float
    scale_factor: float

    def to_dict(self) -> dict[str, object]:
        return {
            "backend": self.backend,
            "requested_corner_join": self.requested_corner_join,
            "requested_smooth_join": self.requested_smooth_join,
            "applied_path_join": self.applied_path_join,
            "join_strategy": self.join_strategy,
            "per_vertex_join_supported": self.per_vertex_join_supported,
            "per_vertex_request_exact": self.per_vertex_request_exact,
            "corner_count": self.corner_count,
            "reflex_corner_count": self.reflex_corner_count,
            "smooth_vertex_count": self.smooth_vertex_count,
            "miter_limit": self.miter_limit,
            "arc_tolerance": self.arc_tolerance,
            "scale_factor": self.scale_factor,
        }


@dataclass(frozen=True)
class OffsetDiagnostics:
    """All mandatory checks, also attached to validation exceptions."""

    nonempty: bool
    component_count: int
    hole_count: int
    area: float
    area_threshold: float
    single_component: bool
    hole_free: bool
    simple: bool
    ccw: bool
    positive_area: bool
    area_above_threshold: bool
    valid: bool
    failure_reasons: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, object]:
        return {
            "nonempty": self.nonempty,
            "component_count": self.component_count,
            "hole_count": self.hole_count,
            "area": self.area,
            "area_threshold": self.area_threshold,
            "single_component": self.single_component,
            "hole_free": self.hole_free,
            "simple": self.simple,
            "ccw": self.ccw,
            "positive_area": self.positive_area,
            "area_above_threshold": self.area_above_threshold,
            "valid": self.valid,
            "failure_reasons": self.failure_reasons,
        }


@dataclass(frozen=True)
class OffsetResult:
    """One valid, CCW, nonempty, single-component, hole-free safe polygon."""

    vertices: FloatArray
    distance: float
    area: float
    metadata: OffsetMetadata
    diagnostics: OffsetDiagnostics
    validation: PolygonValidation

    def __post_init__(self) -> None:
        vertices = np.asarray(self.vertices, dtype=np.float64)
        if vertices.ndim != 2 or vertices.shape[1:] != (2,) or len(vertices) < 3:
            raise OffsetError("offset result vertices must have shape (n, 2), n >= 3")
        object.__setattr__(self, "vertices", vertices.copy())

    @property
    def safe_polygon(self) -> FloatArray:
        return self.vertices

    @property
    def join_metadata(self) -> OffsetMetadata:
        return self.metadata

    def __array__(self, dtype: np.dtype | None = None) -> FloatArray:
        return np.asarray(self.vertices, dtype=dtype)


def _coerce_input(
    polygon: SampledBoundary | ArrayLike,
    corner_mask: ArrayLike | None,
    corner_indices: Sequence[int] | None,
) -> tuple[FloatArray, NDArray[np.bool_]]:
    if isinstance(polygon, SampledBoundary):
        vertices = polygon.vertices.copy()
        inherited_mask = polygon.corner_mask
    else:
        vertices = np.asarray(polygon, dtype=np.float64)
        if vertices.ndim == 2 and len(vertices) > 1 and np.array_equal(vertices[0], vertices[-1]):
            vertices = vertices[:-1].copy()
        inherited_mask = np.zeros(len(vertices), dtype=bool) if vertices.ndim >= 1 else np.empty(0, dtype=bool)
    # Boundary validation supplies precise shape/finite diagnostics.
    validation = validate_polygon(vertices, require_ccw=True, raise_on_error=False)
    if not validation.valid:
        raise BoundaryError("invalid offset input: " + "; ".join(validation.errors))
    if corner_mask is not None and corner_indices is not None:
        raise BoundaryError("provide corner_mask or corner_indices, not both")
    if corner_mask is not None:
        mask = np.asarray(corner_mask, dtype=bool)
        if mask.shape != (len(vertices),):
            raise BoundaryError("corner_mask must have one value per polygon vertex")
    elif corner_indices is not None:
        mask = np.zeros(len(vertices), dtype=bool)
        indices = tuple(int(index) for index in corner_indices)
        if any(index < 0 or index >= len(vertices) for index in indices):
            raise BoundaryError("corner_indices are outside the polygon")
        mask[list(indices)] = True
    else:
        mask = inherited_mask.copy()
    return vertices, mask


def _join_plan(
    vertices: FloatArray,
    corner_mask: NDArray[np.bool_],
    *,
    miter_limit: float,
    arc_tolerance: float,
    scale_factor: float,
) -> tuple[str, OffsetMetadata]:
    turns = signed_turn_angles(vertices)
    reflex_corners = corner_mask & (turns < -1.0e-10)
    corner_count = int(np.count_nonzero(corner_mask))
    reflex_count = int(np.count_nonzero(reflex_corners))
    smooth_count = len(vertices) - corner_count

    if reflex_count:
        applied = "Miter"
        if smooth_count:
            strategy = "global_miter_preserves_reflex_corners_smooth_vertices_are_dense_miter_approximation"
            exact = False
        else:
            strategy = "global_miter_all_vertices_are_true_corners"
            exact = True
    else:
        applied = "Round"
        if corner_count:
            strategy = "global_round_convex_inward_corner_geometry_equals_miter"
        else:
            strategy = "global_round_all_vertices_are_smooth"
        exact = True
    metadata = OffsetMetadata(
        backend="pyclipr (Clipper2)",
        requested_corner_join="Miter",
        requested_smooth_join="Round",
        applied_path_join=applied,
        join_strategy=strategy,
        per_vertex_join_supported=False,
        per_vertex_request_exact=exact,
        corner_count=corner_count,
        reflex_corner_count=reflex_count,
        smooth_vertex_count=smooth_count,
        miter_limit=float(miter_limit),
        arc_tolerance=float(arc_tolerance),
        scale_factor=float(scale_factor),
    )
    return applied, metadata


def _walk_tree(node: object) -> list[object]:
    output: list[object] = []
    for child in getattr(node, "children", ()):  # pyclipr PolyTreeD
        output.append(child)
        output.extend(_walk_tree(child))
    return output


def _empty_diagnostics(min_area: float, component_count: int = 0, hole_count: int = 0) -> OffsetDiagnostics:
    failures = ["offset is empty"] if component_count == 0 else []
    if component_count != 1:
        failures.append(f"offset has {component_count} connected components")
    if hole_count:
        failures.append(f"offset has {hole_count} holes")
    failures.extend(("offset polygon is not simple", "offset area is not positive", f"offset area is not greater than {min_area:g}"))
    return OffsetDiagnostics(
        nonempty=False,
        component_count=component_count,
        hole_count=hole_count,
        area=0.0,
        area_threshold=float(min_area),
        single_component=component_count == 1,
        hole_free=hole_count == 0,
        simple=False,
        ccw=False,
        positive_area=False,
        area_above_threshold=False,
        valid=False,
        failure_reasons=tuple(failures),
    )


def inward_offset(
    polygon: SampledBoundary | ArrayLike,
    distance: float = DEFAULT_INWARD_OFFSET,
    *,
    corner_mask: ArrayLike | None = None,
    corner_indices: Sequence[int] | None = None,
    miter_limit: float = DEFAULT_MITER_LIMIT,
    min_area: float = DEFAULT_MIN_SAFE_AREA,
    arc_tolerance: float = DEFAULT_ARC_TOLERANCE,
    scale_factor: float = DEFAULT_CLIPPER_SCALE,
) -> OffsetResult:
    """Deflate a CCW simple polygon and enforce every safe-region condition.

    The primary and only offset engine is pyclipr's Clipper2 binding.  Shapely
    may be used by :func:`validate_polygon` to inspect the result, but there is
    intentionally no Shapely-buffer fallback.
    """

    amount = float(distance)
    if not np.isfinite(amount) or amount <= 0.0:
        raise BoundaryError("inward offset distance must be positive and finite")
    if miter_limit != DEFAULT_MITER_LIMIT:
        # The function remains configurable for controlled studies, while the documented default is fixed.
        miter_limit = float(miter_limit)
    if not np.isfinite(miter_limit) or miter_limit <= 0.0:
        raise BoundaryError("miter_limit must be positive and finite")
    min_area = float(min_area)
    arc_tolerance = float(arc_tolerance)
    scale_factor = float(scale_factor)
    if min_area < 0.0 or arc_tolerance < 0.0 or scale_factor <= 0.0:
        raise BoundaryError("min_area and arc_tolerance must be nonnegative; scale_factor must be positive")

    vertices, mask = _coerce_input(polygon, corner_mask, corner_indices)
    applied_join, metadata = _join_plan(
        vertices,
        mask,
        miter_limit=miter_limit,
        arc_tolerance=arc_tolerance,
        scale_factor=scale_factor,
    )
    try:
        import pyclipr
    except ImportError as exc:  # pragma: no cover - dependency is installed in the project environment
        raise OffsetDependencyError("safe inward offsets require pyclipr (Clipper2); no Shapely-buffer fallback is used") from exc

    clipper = pyclipr.ClipperOffset()
    clipper.scaleFactor = scale_factor
    clipper.miterLimit = miter_limit
    # pyclipr applies ``scaleFactor`` to path coordinates before invoking
    # ClipperOffset, and exposes arcTolerance in those scaled units.  The
    # public API stays in metres, so scale the tolerance exactly once here.
    clipper.arcTolerance = arc_tolerance * scale_factor
    clipper.preserveCollinear = False
    join_type = pyclipr.JoinType.Miter if applied_join == "Miter" else pyclipr.JoinType.Round
    clipper.addPath(vertices, join_type, pyclipr.EndType.Polygon)
    tree = clipper.executeTree(-amount)

    all_nodes = _walk_tree(tree)
    top_level = [node for node in getattr(tree, "children", ()) if not bool(getattr(node, "isHole", False))]
    holes = [node for node in all_nodes if bool(getattr(node, "isHole", False))]
    component_count, hole_count = len(top_level), len(holes)
    if component_count == 0:
        diagnostics = _empty_diagnostics(min_area, component_count, hole_count)
        raise OffsetValidationError("safe inward offset failed: " + "; ".join(diagnostics.failure_reasons), diagnostics)
    if component_count != 1 or hole_count:
        area = float(sum(abs(float(getattr(node, "area", 0.0))) for node in top_level))
        failures: list[str] = []
        if component_count != 1:
            failures.append(f"offset has {component_count} connected components")
        if hole_count:
            failures.append(f"offset has {hole_count} holes")
        diagnostics = OffsetDiagnostics(
            nonempty=True,
            component_count=component_count,
            hole_count=hole_count,
            area=area,
            area_threshold=min_area,
            single_component=component_count == 1,
            hole_free=hole_count == 0,
            simple=False,
            ccw=False,
            positive_area=area > 0.0,
            area_above_threshold=area > min_area,
            valid=False,
            failure_reasons=tuple(failures),
        )
        raise OffsetValidationError("safe inward offset failed: " + "; ".join(failures), diagnostics)

    safe = np.asarray(getattr(top_level[0], "polygon"), dtype=np.float64)
    if safe.ndim != 2 or safe.shape[1:] != (2,) or len(safe) < 3:
        diagnostics = _empty_diagnostics(min_area, component_count, hole_count)
        raise OffsetValidationError("safe inward offset failed: malformed Clipper2 output", diagnostics)
    # Clipper2 normally returns positive rings, but normalize defensively before strict validation.
    preliminary = validate_polygon(safe, require_ccw=False)
    if preliminary.signed_area < 0.0:
        safe = safe[::-1].copy()
    validation = validate_polygon(safe, require_ccw=True)
    area = float(validation.signed_area)
    failures = list(validation.errors)
    if area <= min_area:
        failures.append(f"offset area {area:.9g} is not greater than {min_area:.9g}")
    diagnostics = OffsetDiagnostics(
        nonempty=True,
        component_count=1,
        hole_count=0,
        area=area,
        area_threshold=min_area,
        single_component=True,
        hole_free=True,
        simple=validation.is_simple,
        ccw=validation.is_ccw,
        positive_area=area > 0.0,
        area_above_threshold=area > min_area,
        valid=not failures,
        failure_reasons=tuple(failures),
    )
    if failures:
        raise OffsetValidationError("safe inward offset failed: " + "; ".join(failures), diagnostics)
    return OffsetResult(safe, amount, area, metadata, diagnostics, validation)


compute_inward_offset = inward_offset
build_safe_polygon = inward_offset


__all__ = [
    "DEFAULT_ARC_TOLERANCE",
    "DEFAULT_CLIPPER_SCALE",
    "DEFAULT_INWARD_OFFSET",
    "DEFAULT_MIN_SAFE_AREA",
    "DEFAULT_MITER_LIMIT",
    "GEOMETRIC_MARGIN",
    "NUMERICAL_MARGIN",
    "OffsetDependencyError",
    "OffsetDiagnostics",
    "OffsetError",
    "OffsetMetadata",
    "OffsetResult",
    "OffsetValidationError",
    "PHYSICAL_MARGIN",
    "build_safe_polygon",
    "compute_inward_offset",
    "inward_offset",
]
