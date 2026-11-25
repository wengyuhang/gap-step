"""Numerical validation required by the SC-DynaTOGT experiment plan."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Callable

import numpy as np
from numpy.typing import ArrayLike, NDArray
from shapely import contains_xy
from shapely.geometry import Polygon

from .environment import SCDynamicWindow
from .optimizer import JointTOGTObjective
from .sc_mapping import SCDiskMap


FloatArray = NDArray[np.float64]


def componentwise_relative_error(analytic: ArrayLike, numeric: ArrayLike) -> FloatArray:
    """Symmetric componentwise relative error with a roundoff floor."""

    left = np.asarray(analytic, dtype=float)
    right = np.asarray(numeric, dtype=float)
    scale = np.maximum.reduce((np.abs(left), np.abs(right), np.full_like(left, 1.0e-12)))
    return np.abs(left - right) / scale


@dataclass(frozen=True)
class GradientCheckReport:
    sample_count: int
    component_count: int
    step: float
    median_relative_error: float
    p99_relative_error: float
    maximum_relative_error: float
    median_pass: bool
    p99_pass: bool

    @property
    def passed(self) -> bool:
        return self.median_pass and self.p99_pass


@dataclass(frozen=True)
class MappingValidationReport:
    sample_count: int
    batch_size: int
    seed: int
    inside_count: int
    outside_count: int
    nan_count: int
    inf_count: int
    degenerate_jacobian_count: int
    minimum_abs_determinant: float
    passed: bool


def _gradient_report(errors: list[float], samples: int, step: float) -> GradientCheckReport:
    values = np.asarray(errors, dtype=float)
    median = float(np.median(values))
    p99 = float(np.quantile(values, 0.99))
    return GradientCheckReport(
        sample_count=samples,
        component_count=len(values),
        step=float(step),
        median_relative_error=median,
        p99_relative_error=p99,
        maximum_relative_error=float(np.max(values)),
        median_pass=median < 1.0e-5,
        p99_pass=p99 < 1.0e-3,
    )


def check_window_gradients(
    window: SCDynamicWindow,
    *,
    sample_count: int = 100,
    seed: int = 0,
    time_range: tuple[float, float] = (0.0, 10.0),
    step: float = 1.0e-6,
) -> GradientCheckReport:
    """Check both spatial and time adjoints using centered differences."""

    if sample_count < 1 or step <= 0.0 or time_range[1] <= time_range[0]:
        raise ValueError("invalid gradient-check settings")
    rng = np.random.default_rng(seed)
    errors: list[float] = []
    eye = np.eye(2)
    for _ in range(sample_count):
        d = rng.normal(size=2)
        instant = float(rng.uniform(*time_range))
        upstream = rng.normal(size=3)
        analytic_d, analytic_t = window.get_grad(d, instant, upstream)
        numeric_d = np.asarray(
            [
                upstream
                @ (
                    window.to_point(d + step * eye[index], instant)
                    - window.to_point(d - step * eye[index], instant)
                )
                / (2.0 * step)
                for index in range(2)
            ]
        )
        numeric_t = float(
            upstream
            @ (
                window.to_point(d, instant + step)
                - window.to_point(d, instant - step)
            )
            / (2.0 * step)
        )
        errors.extend(componentwise_relative_error(analytic_d, numeric_d).tolist())
        errors.append(float(componentwise_relative_error(analytic_t, numeric_t)))
    return _gradient_report(errors, sample_count, step)


def check_joint_objective_gradient(
    objective: JointTOGTObjective,
    x: ArrayLike | None = None,
    *,
    step: float = 1.0e-6,
) -> GradientCheckReport:
    """Check the complete ``[K,D]`` adjoint, including moving-window time."""

    values = objective.initial_guess() if x is None else np.asarray(x, dtype=float)
    _, analytic = objective.value_and_gradient(values)
    numeric = np.empty_like(values)
    for index in range(len(values)):
        direction = np.zeros_like(values)
        direction[index] = step
        numeric[index] = (
            objective.evaluate(values + direction).cost
            - objective.evaluate(values - direction).cost
        ) / (2.0 * step)
    errors = componentwise_relative_error(analytic, numeric).tolist()
    return _gradient_report(errors, 1, step)


def _map_batch(sc_map: SCDiskMap, disk_points: FloatArray) -> tuple[FloatArray, FloatArray]:
    """Evaluate the SC part of ``Psi(B(d))`` for one disk batch."""

    points = sc_map.evaluate_many(disk_points)
    determinants = np.linalg.det(sc_map.jacobian_many(disk_points))
    return np.asarray(points, dtype=float), np.asarray(determinants, dtype=float)


def validate_sc_mapping(
    sc_map: SCDiskMap,
    *,
    sample_count: int = 1_000_000,
    seed: int = 0,
    batch_size: int = 4096,
    determinant_tolerance: float = 1.0e-14,
    progress: Callable[[int, int], None] | None = None,
) -> MappingValidationReport:
    """Run the plan's ``d ~ N(0, 4I)`` legality/stability experiment.

    Samples are generated and checked in bounded-memory batches.  The full
    protocol default is one million points; unit tests deliberately pass a
    smaller count without changing the production default.
    """

    if sample_count < 1 or batch_size < 1 or determinant_tolerance < 0.0:
        raise ValueError("invalid mapping-validation settings")
    rng = np.random.default_rng(seed)
    polygon = Polygon(sc_map.vertices)
    inside = outside = nan = inf = degenerate = 0
    min_determinant = float("inf")
    completed = 0
    while completed < sample_count:
        count = min(batch_size, sample_count - completed)
        d = rng.normal(0.0, 2.0, size=(count, 2))
        radii = np.sqrt(1.0 + np.einsum("ij,ij->i", d, d))
        disk = d / radii[:, None]
        points, determinants = _map_batch(sc_map, disk)
        # The required online map is q(d) = Psi(B(d)), not Psi alone.  For
        # B(d)=d/sqrt(1+||d||^2), det(J_B)=(1+||d||^2)^-2.
        determinants *= np.reciprocal(radii**4)
        nan_mask = np.isnan(points).any(axis=1) | np.isnan(determinants)
        inf_mask = np.isinf(points).any(axis=1) | np.isinf(determinants)
        finite = ~(nan_mask | inf_mask)
        nan += int(np.count_nonzero(nan_mask))
        inf += int(np.count_nonzero(inf_mask))
        if np.any(finite):
            flags = contains_xy(polygon, points[finite, 0], points[finite, 1])
            inside += int(np.count_nonzero(flags))
            outside += int(len(flags) - np.count_nonzero(flags))
            abs_determinants = np.abs(determinants[finite])
            degenerate += int(np.count_nonzero(abs_determinants <= determinant_tolerance))
            min_determinant = min(min_determinant, float(np.min(abs_determinants)))
        completed += count
        if progress is not None:
            progress(completed, sample_count)
    passed = outside == nan == inf == degenerate == 0 and inside == sample_count
    return MappingValidationReport(
        sample_count=sample_count,
        batch_size=batch_size,
        seed=seed,
        inside_count=inside,
        outside_count=outside,
        nan_count=nan,
        inf_count=inf,
        degenerate_jacobian_count=degenerate,
        minimum_abs_determinant=min_determinant,
        passed=passed,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Validate an offline SC map")
    parser.add_argument("--map", required=True, type=Path, help="SCDiskMap .npz file")
    parser.add_argument("--samples", type=int, default=1_000_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--batch-size", type=int, default=4096)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    report = validate_sc_mapping(
        SCDiskMap.load(args.map),
        sample_count=args.samples,
        seed=args.seed,
        batch_size=args.batch_size,
    )
    payload = json.dumps(asdict(report), ensure_ascii=False, indent=2)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    print(payload)
    return 0 if report.passed else 2


if __name__ == "__main__":  # pragma: no cover - exercised by CLI smoke tests
    raise SystemExit(main())


__all__ = [
    "GradientCheckReport",
    "MappingValidationReport",
    "check_joint_objective_gradient",
    "check_window_gradients",
    "componentwise_relative_error",
    "validate_sc_mapping",
]
