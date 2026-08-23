"""Zero-fixed-inset preprocessing for WBSC-DynaTOGT gates."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np
from numpy.typing import ArrayLike, NDArray

from nonconvex_timevarying_window.sc_dynatogt.boundary import (
    ADAPTIVE_VERTEX_COUNTS,
    CONCAVITY_ANGLE_THRESHOLD_DEG,
    CONCAVITY_ERROR_TOLERANCE,
    POLYGON_ERROR_TOLERANCE,
    DenseBoundary,
    ResampleReport,
    SampledBoundary,
    adaptive_chang_resample,
    validate_polygon,
)
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap


FloatArray = NDArray[np.float64]
ARTIFACT_FORMAT_VERSION = 1


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


@dataclass(frozen=True)
class WBPreprocessingConfig:
    """Boundary/SC settings with no pose-independent physical inset."""

    vertex_counts: tuple[int, ...] = ADAPTIVE_VERTEX_COUNTS
    boundary_tolerance: float = POLYGON_ERROR_TOLERANCE
    concavity_tolerance: float = CONCAVITY_ERROR_TOLERANCE
    concavity_angle_threshold_deg: float = CONCAVITY_ANGLE_THRESHOLD_DEG
    sc_fit_options: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        counts = tuple(int(value) for value in self.vertex_counts)
        if not counts or any(value < 3 for value in counts):
            raise ValueError("vertex_counts must contain integers >= 3")
        if any(right <= left for left, right in zip(counts, counts[1:])):
            raise ValueError("vertex_counts must be strictly increasing")
        object.__setattr__(self, "vertex_counts", counts)
        object.__setattr__(self, "sc_fit_options", dict(self.sc_fit_options))

    def to_dict(self) -> dict[str, Any]:
        return _jsonable(asdict(self))

    @classmethod
    def from_dict(cls, values: Mapping[str, Any]) -> "WBPreprocessingConfig":
        data = dict(values)
        data["vertex_counts"] = tuple(int(value) for value in data["vertex_counts"])
        data["sc_fit_options"] = dict(data.get("sc_fit_options", {}))
        return cls(**data)


@dataclass(frozen=True)
class WBPreprocessedGate:
    """Physical candidate polygon and its disk SC map."""

    name: str
    dense_boundary: DenseBoundary
    sampled_boundary: SampledBoundary
    candidate_polygon: FloatArray
    sc_map: SCDiskMap
    config: WBPreprocessingConfig

    def __post_init__(self) -> None:
        polygon = np.asarray(self.candidate_polygon, dtype=float)
        validate_polygon(polygon, require_ccw=True, raise_on_error=True)
        if polygon.shape != self.sc_map.vertices.shape or not np.allclose(
            polygon, self.sc_map.vertices, atol=1.0e-12, rtol=0.0
        ):
            raise ValueError("candidate polygon must match the SC map polygon")
        object.__setattr__(self, "candidate_polygon", polygon.copy())

    @property
    def safe_polygon(self) -> FloatArray:
        """Compatibility alias; safety is imposed online, not by this polygon."""

        return self.candidate_polygon

    @property
    def selected_vertex_count(self) -> int:
        return self.sampled_boundary.m

    def save(self, directory: str | Path) -> Path:
        root = Path(directory)
        root.mkdir(parents=True, exist_ok=True)
        with (root / "geometry.npz").open("wb") as stream:
            np.savez_compressed(
                stream,
                dense_vertices=self.dense_boundary.vertices,
                dense_corners=self.dense_boundary.corners,
                dense_corner_indices=np.asarray(self.dense_boundary.corner_indices, dtype=np.int64),
                sampled_vertices=self.sampled_boundary.vertices,
                sampled_corner_mask=self.sampled_boundary.corner_mask,
                sampled_corners=self.sampled_boundary.corners,
                sampled_source_vertex_count=np.asarray(self.sampled_boundary.source_vertex_count, dtype=np.int64),
                candidate_vertices=self.candidate_polygon,
            )
        self.sc_map.save(root / "sc_map.npz")
        manifest = {
            "format_version": ARTIFACT_FORMAT_VERSION,
            "name": self.name,
            "fixed_inward_offset": 0.0,
            "safety_model": "online_pose_dependent_oriented_cuboid",
            "config": self.config.to_dict(),
            "resample_reports": [report.to_dict() for report in self.sampled_boundary.reports],
            "files": {"geometry": "geometry.npz", "sc_map": "sc_map.npz"},
        }
        (root / "manifest.json").write_text(
            json.dumps(_jsonable(manifest), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        return root

    @classmethod
    def load(cls, directory: str | Path) -> "WBPreprocessedGate":
        root = Path(directory)
        manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
        if int(manifest.get("format_version", -1)) != ARTIFACT_FORMAT_VERSION:
            raise ValueError("unsupported WBSC preprocessing artifact version")
        if float(manifest.get("fixed_inward_offset", -1.0)) != 0.0:
            raise ValueError("WBSC artifacts must not contain a fixed physical inset")
        with np.load(root / manifest["files"]["geometry"], allow_pickle=False) as arrays:
            dense = DenseBoundary(
                arrays["dense_vertices"],
                arrays["dense_corners"],
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
                arrays["sampled_vertices"],
                arrays["sampled_corner_mask"],
                arrays["sampled_corners"],
                int(arrays["sampled_source_vertex_count"]),
                reports,
            )
            candidate = np.asarray(arrays["candidate_vertices"], dtype=float)
        mapping = SCDiskMap.load(root / manifest["files"]["sc_map"])
        return cls(
            str(manifest["name"]),
            dense,
            sampled,
            candidate,
            mapping,
            WBPreprocessingConfig.from_dict(manifest["config"]),
        )


def preprocess_boundary(
    boundary: DenseBoundary | ArrayLike,
    *,
    name: str = "gate",
    corners: ArrayLike | None = None,
    config: WBPreprocessingConfig | None = None,
) -> WBPreprocessedGate:
    """Chang-resample a physical aperture and fit SC without fixed erosion."""

    settings = WBPreprocessingConfig() if config is None else config
    if isinstance(boundary, DenseBoundary):
        dense = boundary if corners is None else DenseBoundary(boundary.vertices, corners)
    else:
        dense = DenseBoundary(
            boundary,
            np.empty((0, 2), dtype=float) if corners is None else corners,
        )
    sampled = adaptive_chang_resample(
        dense,
        vertex_counts=settings.vertex_counts,
        boundary_tolerance=settings.boundary_tolerance,
        concavity_tolerance=settings.concavity_tolerance,
        concavity_angle_threshold_deg=settings.concavity_angle_threshold_deg,
    )
    candidate = np.asarray(sampled.vertices, dtype=float)
    validate_polygon(candidate, require_ccw=True, raise_on_error=True)
    mapping = SCDiskMap.fit(candidate, **dict(settings.sc_fit_options))
    return WBPreprocessedGate(str(name), dense, sampled, candidate, mapping, settings)


__all__ = [
    "ARTIFACT_FORMAT_VERSION",
    "WBPreprocessedGate",
    "WBPreprocessingConfig",
    "preprocess_boundary",
]
