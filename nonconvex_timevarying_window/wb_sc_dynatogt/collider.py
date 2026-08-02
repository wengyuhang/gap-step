"""A deliberately simple attitude-aware cuboid model."""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class ColliderConfig:
    """Physical cuboid dimensions and explicit safety margins.

    The default dimensions are normalized to the legacy drone's ``0.300 m``
    maximum radius.  Replacing them with measured CAD dimensions requires no
    change elsewhere in the planner.
    """

    length: float = 0.5300801927129876
    width: float = 0.25522379649143845
    height: float = 0.11779559838066389
    geometric_margin: float = 0.005
    numerical_margin: float = 0.010

    def __post_init__(self) -> None:
        values = (self.length, self.width, self.height)
        if any(not np.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("cuboid dimensions must be finite and positive")
        if self.geometric_margin < 0.0 or self.numerical_margin < 0.0:
            raise ValueError("cuboid margins must be nonnegative")

    @property
    def clearance(self) -> float:
        return float(self.geometric_margin + self.numerical_margin)

    @property
    def maximum_radius(self) -> float:
        return float(0.5 * np.linalg.norm([self.length, self.width, self.height]))

    @property
    def legacy_sphere_radius(self) -> float:
        return 0.315

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


class CuboidCollider:
    """Eight body-frame corners of an oriented rectangular box."""

    def __init__(self, config: ColliderConfig | None = None) -> None:
        self.config = ColliderConfig() if config is None else config
        half = 0.5 * np.array(
            [self.config.length, self.config.width, self.config.height], dtype=float
        )
        self._corners = np.asarray(
            [
                [sx * half[0], sy * half[1], sz * half[2]]
                for sx in (-1.0, 1.0)
                for sy in (-1.0, 1.0)
                for sz in (-1.0, 1.0)
            ],
            dtype=float,
        )

    @property
    def corners(self) -> FloatArray:
        return self._corners.copy()

    @property
    def points(self) -> FloatArray:
        """Compatibility alias used by candidate-set experiments."""

        return self.corners

    @property
    def maximum_radius(self) -> float:
        return float(np.linalg.norm(self._corners, axis=1).max())

    def scaled_corners(self, scale: float) -> FloatArray:
        value = float(scale)
        if not np.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError("cuboid scale must lie in [0, 1]")
        return value * self._corners

    def edge_points(self, samples_per_edge: int = 9) -> FloatArray:
        """Sample all 12 cuboid edges, including their endpoints."""

        count = int(samples_per_edge)
        if count < 2:
            raise ValueError("samples_per_edge must be at least two")
        half = 0.5 * np.array(
            [self.config.length, self.config.width, self.config.height], dtype=float
        )
        values: list[np.ndarray] = []
        fractions = np.linspace(-1.0, 1.0, count)
        for varying_axis in range(3):
            fixed_axes = [axis for axis in range(3) if axis != varying_axis]
            for first_sign in (-1.0, 1.0):
                for second_sign in (-1.0, 1.0):
                    points = np.zeros((count, 3), dtype=float)
                    points[:, varying_axis] = fractions * half[varying_axis]
                    points[:, fixed_axes[0]] = first_sign * half[fixed_axes[0]]
                    points[:, fixed_axes[1]] = second_sign * half[fixed_axes[1]]
                    values.append(points)
        stacked = np.vstack(values)
        return np.unique(np.round(stacked, decimals=15), axis=0)

    def manifest(self) -> dict[str, object]:
        return {
            "model": "oriented_cuboid",
            "config": self.config.to_dict(),
            "maximum_radius": self.maximum_radius,
            "corner_count": 8,
            "hard_constraint_edge_points": len(self.edge_points()),
        }


__all__ = ["ColliderConfig", "CuboidCollider"]
