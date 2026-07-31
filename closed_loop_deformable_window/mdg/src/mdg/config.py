"""Strict, serializable configuration for MDG."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Any, Mapping

import yaml


@dataclass(frozen=True)
class SafetyConfig:
    drone_collision_radius: float = 0.15
    tracking_margin: float = 0.05
    geometry_discretization_margin: float = 0.02
    safety_radius: float = 0.22

    def __post_init__(self) -> None:
        total = (
            self.drone_collision_radius
            + self.tracking_margin
            + self.geometry_discretization_margin
        )
        if min(
            self.drone_collision_radius,
            self.tracking_margin,
            self.geometry_discretization_margin,
        ) < 0.0:
            raise ValueError("safety margins must be nonnegative")
        if abs(total - self.safety_radius) > 1.0e-12:
            raise ValueError("safety_radius must equal the three component margins")


@dataclass(frozen=True)
class DiscConfig:
    grid_resolution: float = 0.04
    nms_ratio: float = 0.8
    min_radius: float = 0.05
    max_disks_per_gate: int = 5


@dataclass(frozen=True)
class TrackingConfig:
    gate_sample_dt: float = 0.10
    match_distance_scale: float = 0.5
    match_radius_scale: float = 0.3
    match_radius_weight: float = 0.5
    match_cost_max: float = 2.0
    max_gap_steps: int = 1
    min_length_steps: int = 3
    validation_dt: float = 0.02
    shrink_margin: float = 0.01
    enable_validation_shrink: bool = True


@dataclass(frozen=True)
class GraphConfig:
    dt_coarse: float = 0.10
    dt_fine: float = 0.02
    v_max_fallback: float = 8.0
    a_max: float = 12.0
    feasibility_ratio: float = 0.8
    refine_time_radius: float = 0.30
    refine_competing_tracks: int = 2
    enable_refine: bool = True


@dataclass(frozen=True)
class DifficultyConfig:
    translation_amplitude: float
    rotation_amplitude_deg: float
    scale_min: float
    scale_max: float
    shape_change_ratio: float


DEFAULT_DIFFICULTIES: dict[str, DifficultyConfig] = {
    "low": DifficultyConfig(0.20, 10.0, 0.90, 1.10, 0.10),
    "medium": DifficultyConfig(0.50, 25.0, 0.75, 1.25, 0.25),
    "high": DifficultyConfig(0.80, 40.0, 0.55, 1.35, 0.40),
}


@dataclass(frozen=True)
class ScenarioConfig:
    planning_horizon: float = 60.0
    world_size: tuple[float, float, float] = (20.0, 20.0, 4.0)
    gate_height_range: tuple[float, float] = (1.0, 2.8)
    gate_spacing_range: tuple[float, float] = (2.5, 4.0)
    curve_boundary_samples: int = 256
    motion_control_point_dt: float = 2.0
    envelope_validation_dt: float = 0.05
    max_resamples: int = 100


@dataclass(frozen=True)
class BackendConfig:
    max_iterations: int = 32
    samples_per_segment: int = 17
    validation_samples_per_segment: int = 129
    initial_speed: float = 4.0
    interval_penalty: float = 1.0e5
    max_lazy_repairs: int = 5
    time_weight: float = 1.0
    snap_weight: float = 0.0


@dataclass(frozen=True)
class ValidationConfig:
    gate_plane_error_max: float = 0.001
    closed_loop_position_error_max: float = 0.01
    closed_loop_velocity_error_max: float = 0.02
    closed_loop_attitude_error_deg_max: float = 1.0
    closed_loop_body_rate_error_max: float = 0.05
    minimum_clearance_tolerance: float = -0.002
    waypoint_tolerance: float = 2.0e-6
    interval_time_tolerance: float = 0.001
    dynamic_relative_tolerance: float = 0.001


@dataclass(frozen=True)
class RuntimeConfig:
    workers: int = 8
    overwrite: bool = False
    save_video: bool = True
    video_fps: int = 20
    video_duration: float = 12.0


@dataclass(frozen=True)
class MDGConfig:
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    disks: DiscConfig = field(default_factory=DiscConfig)
    tracking: TrackingConfig = field(default_factory=TrackingConfig)
    graph: GraphConfig = field(default_factory=GraphConfig)
    scenario: ScenarioConfig = field(default_factory=ScenarioConfig)
    backend: BackendConfig = field(default_factory=BackendConfig)
    validation: ValidationConfig = field(default_factory=ValidationConfig)
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


_SECTIONS = {
    "safety": SafetyConfig,
    "disks": DiscConfig,
    "tracking": TrackingConfig,
    "graph": GraphConfig,
    "scenario": ScenarioConfig,
    "backend": BackendConfig,
    "validation": ValidationConfig,
    "runtime": RuntimeConfig,
}


def _strict_section(cls: type, value: Mapping[str, Any], name: str):
    allowed = {item.name for item in fields(cls)}
    unknown = set(value) - allowed
    if unknown:
        raise ValueError(f"unknown keys in {name}: {sorted(unknown)}")
    normalized = dict(value)
    for key in ("world_size", "gate_height_range", "gate_spacing_range"):
        if key in normalized:
            normalized[key] = tuple(float(x) for x in normalized[key])
    return cls(**normalized)


def config_from_mapping(data: Mapping[str, Any]) -> MDGConfig:
    unknown = set(data) - set(_SECTIONS)
    if unknown:
        raise ValueError(f"unknown top-level config keys: {sorted(unknown)}")
    values: dict[str, Any] = {}
    for name, cls in _SECTIONS.items():
        raw = data.get(name, {})
        if not isinstance(raw, Mapping):
            raise ValueError(f"config section {name!r} must be a mapping")
        values[name] = _strict_section(cls, raw, name)
    return MDGConfig(**values)


def load_config(path: str | Path | None = None) -> MDGConfig:
    if path is None:
        return MDGConfig()
    source = Path(path)
    payload = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    if not isinstance(payload, Mapping):
        raise ValueError("configuration root must be a mapping")
    return config_from_mapping(payload)


def dump_config(config: MDGConfig, path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        yaml.safe_dump(config.to_dict(), sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return target


__all__ = [
    "BackendConfig",
    "DEFAULT_DIFFICULTIES",
    "DifficultyConfig",
    "DiscConfig",
    "GraphConfig",
    "MDGConfig",
    "RuntimeConfig",
    "SafetyConfig",
    "ScenarioConfig",
    "TrackingConfig",
    "ValidationConfig",
    "config_from_mapping",
    "dump_config",
    "load_config",
]
