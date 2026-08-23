"""Oriented cuboid body model used by the continuous safety verifier."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class CuboidBody:
    """A body-frame axis-aligned cuboid specified by positive half extents."""

    half_extents: np.ndarray

    def __post_init__(self) -> None:
        values = np.asarray(self.half_extents, dtype=float)
        if values.shape != (3,) or not np.all(np.isfinite(values)) or np.any(values <= 0.0):
            raise ValueError("half_extents must be a finite positive vector with shape (3,)")
        object.__setattr__(self, "half_extents", values.copy())

    @property
    def vertices_body(self) -> FloatArray:
        """Return the eight vertices in fixed binary-sign order, shape ``(8,3)``."""

        h = self.half_extents
        return np.asarray(
            [[sx * h[0], sy * h[1], sz * h[2]]
             for sx in (-1.0, 1.0)
             for sy in (-1.0, 1.0)
             for sz in (-1.0, 1.0)],
            dtype=float,
        )

    @property
    def edges(self) -> tuple[tuple[int, int], ...]:
        """Return the canonical 12 edges in lexicographic vertex-index order."""

        return tuple(
            (left, right)
            for left in range(8)
            for right in range(left + 1, 8)
            if (left ^ right) in (1, 2, 4)
        )
