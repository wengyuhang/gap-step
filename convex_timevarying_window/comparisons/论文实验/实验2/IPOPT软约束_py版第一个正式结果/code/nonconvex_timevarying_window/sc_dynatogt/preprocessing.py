"""Offline preprocessing for non-convex Schwarz--Christoffel gates.

The pipeline in this module is intentionally narrow and auditable:

``dense boundary -> Chang arclength resampling -> Clipper2 inset -> disk SC``.

Chang--Gotsman--Hormann boundary processing is not used to parameterize the
interior.  Every interior point is produced by :class:`~.sc_mapping.SCDiskMap`.
The production defaults below are the fixed values from the experiment plan;
the configurability exists mainly for small unit tests and controlled ablations.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .boundary import (
    ADAPTIVE_VERTEX_COUNTS,
    CONCAVITY_ANGLE_THRESHOLD_DEG,
    CONCAVITY_ERROR_TOLERANCE,
    DENSE_CHORD_TOLERANCE,
    DENSE_MAX_CHORD,
    POLYGON_ERROR_TOLERANCE,
    Bezier,
    BoundarySegment,
    DenseBoundary,
    Line,
    ResampleReport,
    SampledBoundary,
    adaptive_chang_resample,
)
from .offset import (
    DEFAULT_ARC_TOLERANCE,
    DEFAULT_CLIPPER_SCALE,
    DEFAULT_INWARD_OFFSET,
    DEFAULT_MIN_SAFE_AREA,
    DEFAULT_MITER_LIMIT,
    OffsetDiagnostics,
    OffsetMetadata,
    OffsetResult,
    inward_offset,
)
from .sc_mapping import SCDiskMap


FloatArray = NDArray[np.float64]
ARTIFACT_FORMAT_VERSION = 1


def _json_primitive(value: Any) -> Any:
    """Convert configuration values to strict JSON without using pickle."""

    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        return {str(key): _json_primitive(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_primitive(item) for item in value]
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    raise TypeError(f"preprocessing metadata value {value!r} is not JSON serializable")


@dataclass(frozen=True)
class PreprocessingConfig:
    """Fixed experiment defaults for the complete offline gate pipeline."""

    vertex_counts: tuple[int, ...] = ADAPTIVE_VERTEX_COUNTS
    boundary_tolerance: float = POLYGON_ERROR_TOLERANCE
    concavity_tolerance: float = CONCAVITY_ERROR_TOLERANCE
    concavity_angle_threshold_deg: float = CONCAVITY_ANGLE_THRESHOLD_DEG
    offset_distance: float = DEFAULT_INWARD_OFFSET
    miter_limit: float = DEFAULT_MITER_LIMIT
    min_safe_area: float = DEFAULT_MIN_SAFE_AREA
    arc_tolerance: float = DEFAULT_ARC_TOLERANCE
    clipper_scale: float = DEFAULT_CLIPPER_SCALE
    sc_fit_options: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        counts = tuple(int(value) for value in self.vertex_counts)
        if not counts or any(value < 3 for value in counts) or any(b <= a for a, b in zip(counts, counts[1:])):
            raise ValueError("vertex_counts must be strictly increasing integers >= 3")
        object.__setattr__(self, "vertex_counts", counts)
        object.__setattr__(self, "sc_fit_options", dict(self.sc_fit_options))

    def to_dict(self) -> dict[str, Any]:
        return _json_primitive(asdict(self))

    @classmethod
    def from_dict(cls, values: Mapping[str, Any]) -> "PreprocessingConfig":
        data = dict(values)
        data["vertex_counts"] = tuple(int(value) for value in data["vertex_counts"])
        data["sc_fit_options"] = dict(data.get("sc_fit_options", {}))
        return cls(**data)


@dataclass(frozen=True)
class PreprocessedGate:
    """All persisted products of one successful offline preprocessing run."""

    name: str
    dense_boundary: DenseBoundary
    sampled_boundary: SampledBoundary
    safe_region: OffsetResult
    sc_map: SCDiskMap
    config: PreprocessingConfig

    @property
    def safe_polygon(self) -> FloatArray:
        return self.safe_region.vertices

    @property
    def selected_vertex_count(self) -> int:
        return self.sampled_boundary.m

    def save(self, directory: str | Path) -> Path:
        """Write a portable, non-pickle preprocessing artifact directory."""

        root = Path(directory).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        arrays_path = root / "geometry.npz"
        with arrays_path.open("wb") as stream:
            np.savez_compressed(
                stream,
                dense_vertices=self.dense_boundary.vertices,
                dense_corners=self.dense_boundary.corners,
                dense_corner_indices=np.asarray(self.dense_boundary.corner_indices, dtype=np.int64),
                sampled_vertices=self.sampled_boundary.vertices,
                sampled_corner_mask=self.sampled_boundary.corner_mask,
                sampled_corners=self.sampled_boundary.corners,
                sampled_source_vertex_count=np.asarray(
                    self.sampled_boundary.source_vertex_count, dtype=np.int64
                ),
                safe_vertices=self.safe_region.vertices,
            )
        self.sc_map.save(root / "sc_map.npz")
        manifest = {
            "format_version": ARTIFACT_FORMAT_VERSION,
            "name": self.name,
            "config": self.config.to_dict(),
            "resample_reports": [report.to_dict() for report in self.sampled_boundary.reports],
            "offset_distance": self.safe_region.distance,
            "offset_area": self.safe_region.area,
            "offset_metadata": self.safe_region.metadata.to_dict(),
            "offset_diagnostics": self.safe_region.diagnostics.to_dict(),
            "files": {"geometry": arrays_path.name, "sc_map": "sc_map.npz"},
        }
        (root / "manifest.json").write_text(
            json.dumps(_json_primitive(manifest), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        return root

    @classmethod
    def load(cls, directory: str | Path) -> "PreprocessedGate":
        """Load and cross-check an artifact written by :meth:`save`."""

        root = Path(directory).expanduser()
        manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
        version = int(manifest.get("format_version", -1))
        if version != ARTIFACT_FORMAT_VERSION:
            raise ValueError(f"unsupported preprocessing artifact version {version}")
        files = manifest.get("files", {})
        geometry_path = root / str(files.get("geometry", "geometry.npz"))
        sc_path = root / str(files.get("sc_map", "sc_map.npz"))
        with np.load(geometry_path, allow_pickle=False) as arrays:
            dense = DenseBoundary(
                np.asarray(arrays["dense_vertices"], dtype=np.float64),
                np.asarray(arrays["dense_corners"], dtype=np.float64),
                tuple(int(value) for value in arrays["dense_corner_indices"]),
            )
            reports = tuple(
                ResampleReport(
                    **{
                        **dict(report),
                        "failure_reasons": tuple(report.get("failure_reasons", ())),
                    }
                )
                for report in manifest.get("resample_reports", ())
            )
            sampled = SampledBoundary(
                np.asarray(arrays["sampled_vertices"], dtype=np.float64),
                np.asarray(arrays["sampled_corner_mask"], dtype=bool),
                np.asarray(arrays["sampled_corners"], dtype=np.float64),
                int(arrays["sampled_source_vertex_count"]),
                reports,
            )
            safe_vertices = np.asarray(arrays["safe_vertices"], dtype=np.float64)

        metadata_values = dict(manifest["offset_metadata"])
        diagnostics_values = dict(manifest["offset_diagnostics"])
        diagnostics_values["failure_reasons"] = tuple(diagnostics_values.get("failure_reasons", ()))
        metadata = OffsetMetadata(**metadata_values)
        diagnostics = OffsetDiagnostics(**diagnostics_values)
        from .boundary import validate_polygon

        validation = validate_polygon(safe_vertices, require_ccw=True, raise_on_error=True)
        safe = OffsetResult(
            safe_vertices,
            float(manifest["offset_distance"]),
            float(manifest["offset_area"]),
            metadata,
            diagnostics,
            validation,
        )
        sc_map = SCDiskMap.load(sc_path)
        if sc_map.vertices.shape != safe.vertices.shape or not np.allclose(
            sc_map.vertices, safe.vertices, rtol=0.0, atol=1.0e-12
        ):
            raise ValueError("saved SC polygon does not match the saved safe polygon")
        return cls(
            name=str(manifest.get("name", "gate")),
            dense_boundary=dense,
            sampled_boundary=sampled,
            safe_region=safe,
            sc_map=sc_map,
            config=PreprocessingConfig.from_dict(manifest["config"]),
        )


def preprocess_boundary(
    boundary: DenseBoundary | ArrayLike,
    *,
    name: str = "gate",
    corners: ArrayLike | None = None,
    config: PreprocessingConfig | None = None,
) -> PreprocessedGate:
    """Run the document's complete offline pipeline for one boundary."""

    settings = PreprocessingConfig() if config is None else config
    if isinstance(boundary, DenseBoundary):
        if corners is None:
            dense = boundary
        else:
            dense = DenseBoundary(boundary.vertices, np.asarray(corners, dtype=np.float64))
    else:
        dense = DenseBoundary(
            boundary,
            np.empty((0, 2), dtype=np.float64) if corners is None else corners,
        )
    sampled = adaptive_chang_resample(
        dense,
        vertex_counts=settings.vertex_counts,
        boundary_tolerance=settings.boundary_tolerance,
        concavity_tolerance=settings.concavity_tolerance,
        concavity_angle_threshold_deg=settings.concavity_angle_threshold_deg,
    )
    safe = inward_offset(
        sampled,
        distance=settings.offset_distance,
        miter_limit=settings.miter_limit,
        min_area=settings.min_safe_area,
        arc_tolerance=settings.arc_tolerance,
        scale_factor=settings.clipper_scale,
    )
    mapping = SCDiskMap.fit(safe.vertices, **dict(settings.sc_fit_options))
    return PreprocessedGate(str(name), dense, sampled, safe, mapping, settings)


def preprocess_segments(
    segments: Sequence[BoundarySegment],
    *,
    name: str = "gate",
    config: PreprocessingConfig | None = None,
    chord_tolerance: float = DENSE_CHORD_TOLERANCE,
    max_chord: float = DENSE_MAX_CHORD,
    max_depth: int = 32,
) -> PreprocessedGate:
    """Densify line/arc/Bézier/B-spline segments, then run the pipeline."""

    dense = DenseBoundary.from_segments(
        segments,
        chord_tolerance=chord_tolerance,
        max_chord=max_chord,
        max_depth=max_depth,
    )
    return preprocess_boundary(dense, name=name, config=config)


def preprocess_csv(
    boundary_path: str | Path,
    *,
    corners_path: str | Path | None = None,
    corner_indices: Sequence[int] | None = None,
    name: str | None = None,
    config: PreprocessingConfig | None = None,
) -> PreprocessedGate:
    """Read an ordered dense-boundary CSV (and optional corners), then preprocess."""

    dense = DenseBoundary.from_csv(
        boundary_path,
        corners_path=corners_path,
        corner_indices=corner_indices,
    )
    gate_name = Path(boundary_path).stem if name is None else name
    return preprocess_boundary(dense, name=gate_name, config=config)


def load_preprocessed_gate(directory: str | Path) -> PreprocessedGate:
    """Functional alias for :meth:`PreprocessedGate.load`."""

    return PreprocessedGate.load(directory)


def l_shape_boundary() -> DenseBoundary:
    """Canonical meter-scale L-shaped polygon for experiment E1."""

    vertices = np.asarray(
        [(-2.0, -2.0), (2.0, -2.0), (2.0, -0.5), (0.5, -0.5), (0.5, 2.0), (-2.0, 2.0)],
        dtype=np.float64,
    )
    return DenseBoundary(vertices, vertices, tuple(range(len(vertices))))


def u_shape_boundary() -> DenseBoundary:
    """Canonical meter-scale U-shaped polygon for experiment E1."""

    vertices = np.asarray(
        [
            (-2.50, -2.0),
            (2.50, -2.0),
            (2.50, 2.0),
            (0.80, 2.0),
            (0.80, 1.00),
            (-0.80, 1.00),
            (-0.80, 2.0),
            (-2.50, 2.0),
        ],
        dtype=np.float64,
    )
    return DenseBoundary(vertices, vertices, tuple(range(len(vertices))))


def five_point_star_boundary(*, outer_radius: float = 2.5, inner_radius: float = 1.15) -> DenseBoundary:
    """Simple five-point star polygon with all ten vertices forced as corners."""

    outer = float(outer_radius)
    inner = float(inner_radius)
    if not (math.isfinite(outer) and math.isfinite(inner) and outer > inner > 0.0):
        raise ValueError("star radii must satisfy outer_radius > inner_radius > 0")
    angles = -0.5 * math.pi + np.arange(10, dtype=np.float64) * math.pi / 5.0
    radii = np.where(np.arange(10) % 2 == 0, outer, inner)
    vertices = np.column_stack((radii * np.cos(angles), radii * np.sin(angles)))
    return DenseBoundary(vertices, vertices, tuple(range(len(vertices))))


@dataclass(frozen=True)
class _AnalyticClosedCurve:
    evaluator: Callable[[float], ArrayLike]

    def evaluate(self, u: float) -> FloatArray:
        value = np.asarray(self.evaluator(float(u)), dtype=np.float64)
        if value.shape != (2,) or not np.all(np.isfinite(value)):
            raise ValueError("analytic boundary evaluator must return a finite two-vector")
        return value

    def is_straight(self) -> bool:
        return False

    def preserve_start_as_corner(self) -> bool:
        return False

    def preserve_end_as_corner(self) -> bool:
        return False


def _dense_analytic_boundary(evaluator: Callable[[float], ArrayLike]) -> DenseBoundary:
    return DenseBoundary.from_segments([_AnalyticClosedCurve(evaluator)])


def limacon_boundary(*, base_radius: float = 2.1, cosine_amplitude: float = 0.72) -> DenseBoundary:
    """Smooth, non-self-intersecting limaçon boundary for experiment E1."""

    a, b = float(base_radius), float(cosine_amplitude)
    if not (math.isfinite(a) and math.isfinite(b) and a > abs(b) > 0.0):
        raise ValueError("limaçon parameters must satisfy base_radius > |cosine_amplitude| > 0")

    def evaluate(u: float) -> FloatArray:
        theta = 2.0 * math.pi * u
        radius = a + b * math.cos(theta)
        return np.asarray((radius * math.cos(theta), radius * math.sin(theta)), dtype=np.float64)

    return _dense_analytic_boundary(evaluate)


def wavy_boundary(*, base_radius: float = 2.15, amplitude: float = 0.35, lobes: int = 5) -> DenseBoundary:
    """Smooth radial wave boundary for experiment E1."""

    radius, wave, count = float(base_radius), float(amplitude), int(lobes)
    if not (math.isfinite(radius) and math.isfinite(wave) and radius > abs(wave) > 0.0 and count >= 2):
        raise ValueError("wave parameters require base_radius > |amplitude| > 0 and lobes >= 2")

    def evaluate(u: float) -> FloatArray:
        theta = 2.0 * math.pi * u
        r = radius + wave * math.cos(count * theta)
        return np.asarray((r * math.cos(theta), r * math.sin(theta)), dtype=np.float64)

    return _dense_analytic_boundary(evaluate)


def line_bezier_mixed_boundary() -> DenseBoundary:
    """Closed non-convex boundary composed of straight and cubic Bézier pieces."""

    segments: list[BoundarySegment] = [
        Line((-2.25, -1.7), (2.25, -1.7)),
        Line((2.25, -1.7), (2.25, 1.65)),
        Bezier(((2.25, 1.65), (1.75, 1.65), (1.35, 0.20), (0.55, 0.30))),
        Bezier(((0.55, 0.30), (-0.15, 0.38), (-0.75, 1.65), (-2.25, 1.65))),
        Line((-2.25, 1.65), (-2.25, -1.7)),
    ]
    return DenseBoundary.from_segments(segments)


def e1_boundaries() -> dict[str, DenseBoundary]:
    """Build the six exact boundary families required by experiment E1."""

    return {
        "l_shape": l_shape_boundary(),
        "u_shape": u_shape_boundary(),
        "five_point_star": five_point_star_boundary(),
        "limacon": limacon_boundary(),
        "wavy": wavy_boundary(),
        "line_bezier_mixed": line_bezier_mixed_boundary(),
    }


def main(argv: list[str] | None = None) -> int:
    """Command-line entry for the complete offline preprocessing pipeline."""

    parser = argparse.ArgumentParser(description="Preprocess one non-convex gate and fit its disk SC map")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--input", type=Path, help="ordered dense-boundary CSV")
    source.add_argument("--shape", choices=tuple(e1_boundaries()), help="built-in E1 boundary")
    parser.add_argument("--corners", type=Path, help="optional true-corner CSV")
    parser.add_argument("--outdir", required=True, type=Path)
    parser.add_argument("--name")
    parser.add_argument("--vertex-counts", default="256,512,1024,2048,3200")
    parser.add_argument("--offset", type=float, default=DEFAULT_INWARD_OFFSET)
    parser.add_argument("--quadrature-order", type=int, default=64)
    args = parser.parse_args(argv)
    try:
        counts = tuple(int(value.strip()) for value in args.vertex_counts.split(",") if value.strip())
    except ValueError as error:
        parser.error(f"invalid --vertex-counts: {error}")
    if not counts:
        parser.error("--vertex-counts must contain at least one positive integer")
    if any(value < 3 for value in counts):
        parser.error("every --vertex-counts value must be at least three")
    if args.shape is not None and args.corners is not None:
        parser.error("--corners is only valid together with --input")
    try:
        config = PreprocessingConfig(
            vertex_counts=counts,
            offset_distance=args.offset,
            sc_fit_options={"quadrature_order": args.quadrature_order},
        )
    except ValueError as error:
        parser.error(str(error))
    if args.shape is not None:
        gate = preprocess_boundary(
            e1_boundaries()[args.shape], name=args.name or args.shape, config=config
        )
    else:
        gate = preprocess_csv(
            args.input,
            corners_path=args.corners,
            name=args.name,
            config=config,
        )
    gate.save(args.outdir)
    print(
        json.dumps(
            {
                "name": gate.name,
                "sampled_vertices": gate.selected_vertex_count,
                "safe_vertices": len(gate.safe_polygon),
                "offset_distance": gate.safe_region.distance,
                "outdir": str(args.outdir),
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "ARTIFACT_FORMAT_VERSION",
    "PreprocessedGate",
    "PreprocessingConfig",
    "e1_boundaries",
    "five_point_star_boundary",
    "l_shape_boundary",
    "limacon_boundary",
    "line_bezier_mixed_boundary",
    "load_preprocessed_gate",
    "main",
    "preprocess_boundary",
    "preprocess_csv",
    "preprocess_segments",
    "u_shape_boundary",
    "wavy_boundary",
]
