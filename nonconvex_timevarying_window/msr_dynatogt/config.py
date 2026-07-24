"""Configuration for MSR-DynaTOGT search, validation, and repair."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Literal

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    DynamicLimits,
    ObjectiveWeights,
    PenaltyWeights,
)
from nonconvex_timevarying_window.sc_dynatogt.optimizer import OptimizationConfig


RepairMode = Literal["uniform", "local"]


@dataclass(frozen=True)
class InitializationConfig:
    """Deterministic catalog of local-optimizer starting points."""

    random_starts: int = 2
    turn_aware_starts: int = 1
    dispersed_starts: int = 3
    temporal_noise_std: float = 0.06
    spatial_noise_std: float = 0.22
    turn_fraction: float = 0.55
    dispersed_disk_radius: float = 0.72

    def __post_init__(self) -> None:
        if min(self.random_starts, self.turn_aware_starts, self.dispersed_starts) < 0:
            raise ValueError("initialization counts must be nonnegative")
        if self.temporal_noise_std < 0.0 or self.spatial_noise_std < 0.0:
            raise ValueError("initialization noise scales must be nonnegative")
        if not 0.0 < self.turn_fraction < 1.0:
            raise ValueError("turn_fraction must lie in (0, 1)")
        if not 0.0 < self.dispersed_disk_radius < 1.0:
            raise ValueError("dispersed_disk_radius must lie in (0, 1)")

    @property
    def total_starts(self) -> int:
        return 1 + self.random_starts + self.turn_aware_starts + self.dispersed_starts


@dataclass(frozen=True)
class FeasibilityConfig:
    """High-density *sampled* checks; these are not continuous certificates."""

    samples_per_segment: int = 129
    limit_relative_tolerance: float = 1.0e-6
    waypoint_tolerance: float = 2.0e-7
    plane_tolerance: float = 1.0e-7

    def __post_init__(self) -> None:
        if self.samples_per_segment < 9:
            raise ValueError("high-density checking needs at least 9 samples per segment")
        if min(
            self.limit_relative_tolerance,
            self.waypoint_tolerance,
            self.plane_tolerance,
        ) < 0.0:
            raise ValueError("feasibility tolerances must be nonnegative")


@dataclass(frozen=True)
class RepairConfig:
    """Time-dilation search followed by one full SC-DynaTOGT refinement."""

    enabled: bool = True
    mode: RepairMode = "local"
    expansion_factor: float = 1.25
    maximum_scale: float = 8.0
    binary_iterations: int = 12
    local_neighbor_radius: int = 1
    reoptimization_penalty_multiplier: float = 4.0

    def __post_init__(self) -> None:
        if self.mode not in {"uniform", "local"}:
            raise ValueError("repair mode must be 'uniform' or 'local'")
        if self.expansion_factor <= 1.0:
            raise ValueError("repair expansion_factor must exceed one")
        if self.maximum_scale <= 1.0:
            raise ValueError("repair maximum_scale must exceed one")
        if self.binary_iterations < 1 or self.local_neighbor_radius < 0:
            raise ValueError("repair binary iterations/radius are invalid")
        if self.reoptimization_penalty_multiplier < 1.0:
            raise ValueError("repair penalty multiplier must be at least one")


@dataclass(frozen=True)
class CandidatePoolConfig:
    max_candidates: int = 12
    time_relative_tolerance: float = 2.0e-4
    waypoint_absolute_tolerance: float = 2.0e-3
    duration_absolute_tolerance: float = 2.0e-3

    def __post_init__(self) -> None:
        if self.max_candidates < 1:
            raise ValueError("candidate pool must retain at least one candidate")
        if min(
            self.time_relative_tolerance,
            self.waypoint_absolute_tolerance,
            self.duration_absolute_tolerance,
        ) < 0.0:
            raise ValueError("candidate deduplication tolerances must be nonnegative")


def default_optimization_config(*, smoke: bool = False) -> OptimizationConfig:
    """Match SC-DynaTOGT physics while allowing a bounded smoke runtime."""

    return OptimizationConfig(
        initial_speed=1.0,
        max_iterations=24 if smoke else 0,
        samples_per_segment=8 if smoke else None,
        include_window_time_gradient=True,
        objective_weights=ObjectiveWeights(time=1.0, snap_energy=0.0),
        penalty_weights=PenaltyWeights(
            velocity=0.0,
            collective_thrust=0.0,
            body_rate=1.0,
            rotor_thrust=1.0,
        ),
        dynamic_limits=DynamicLimits(
            max_velocity=60.0,
            max_body_rate_xy=10.0,
            max_body_rate_z=10.0,
            min_rotor_thrust=0.25,
            max_rotor_thrust=5.0,
        ),
    )


@dataclass(frozen=True)
class MSRConfig:
    """Complete algorithm configuration."""

    optimization: OptimizationConfig = field(default_factory=default_optimization_config)
    initialization: InitializationConfig = field(default_factory=InitializationConfig)
    feasibility: FeasibilityConfig = field(default_factory=FeasibilityConfig)
    repair: RepairConfig = field(default_factory=RepairConfig)
    candidate_pool: CandidatePoolConfig = field(default_factory=CandidatePoolConfig)

    @classmethod
    def for_suite(
        cls,
        suite: Literal["smoke", "formal"],
        *,
        repair_mode: RepairMode = "local",
    ) -> "MSRConfig":
        if suite == "smoke":
            return cls(
                optimization=default_optimization_config(smoke=True),
                initialization=InitializationConfig(
                    random_starts=1,
                    turn_aware_starts=1,
                    dispersed_starts=1,
                ),
                feasibility=FeasibilityConfig(samples_per_segment=65),
                repair=RepairConfig(mode=repair_mode, binary_iterations=8),
            )
        if suite == "formal":
            return cls(
                optimization=default_optimization_config(smoke=False),
                repair=RepairConfig(mode=repair_mode),
            )
        raise ValueError("suite must be 'smoke' or 'formal'")

    def repair_optimization(self) -> OptimizationConfig:
        """Return a full L-BFGS-B configuration biased toward feasibility."""

        weights = self.optimization.penalty_weights
        multiplier = self.repair.reoptimization_penalty_multiplier
        return replace(
            self.optimization,
            penalty_weights=PenaltyWeights(
                velocity=weights.velocity * multiplier,
                collective_thrust=weights.collective_thrust * multiplier,
                body_rate=weights.body_rate * multiplier,
                rotor_thrust=weights.rotor_thrust * multiplier,
            ),
        )


__all__ = [
    "CandidatePoolConfig",
    "FeasibilityConfig",
    "InitializationConfig",
    "MSRConfig",
    "RepairConfig",
    "RepairMode",
    "default_optimization_config",
]
