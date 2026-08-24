"""Shared whole-body geometry for SC and certified SIP validation.

The vehicle is the closed oriented cuboid

``p + R_B diag(half_extents) [-1, 1]^3``.

Both algorithms use the exact Euclidean point-to-cuboid distance below.  SC
uses it for collision diagnostics and a circumscribed-radius inset for its
time-independent conformal map; SIP applies the same formula directly to the
original continuous boundary primitives.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class CuboidBody:
    """Square-bottom quadrotor body used by both SC and SIP."""

    half_extents: tuple[float, float, float] = (0.26504, 0.26504, 0.05890)

    def __post_init__(self) -> None:
        half = np.asarray(self.half_extents, dtype=float)
        if half.shape != (3,) or not np.all(np.isfinite(half)) or np.any(half <= 0.0):
            raise ValueError("half_extents must contain three finite positive values")
        if not np.isclose(half[0], half[1]) or not half[2] < half[0]:
            raise ValueError("body must be a square-bottom cuboid with h_B < ell_B")
        object.__setattr__(self, "half_extents", tuple(float(value) for value in half))

    @property
    def circumscribed_radius(self) -> float:
        """Radius of the smallest body-centred sphere containing the cuboid."""

        return float(np.linalg.norm(np.asarray(self.half_extents, dtype=float)))

    def conservative_center_clearance(self, net_clearance: float) -> float:
        """Center-to-frame distance sufficient for any body attitude."""

        clearance = float(net_clearance)
        if not np.isfinite(clearance) or clearance < 0.0:
            raise ValueError("net_clearance must be finite and nonnegative")
        return self.circumscribed_radius + clearance


def point_to_oriented_cuboid_distance_squared(
    points: ArrayLike,
    center: ArrayLike,
    rotation: ArrayLike,
    body: CuboidBody,
) -> float | FloatArray:
    """Return exact squared distances from world points to an oriented cuboid.

    ``rotation`` maps body coordinates to world coordinates.  ``points`` may
    have shape ``(3,)`` or ``(..., 3)``.
    """

    values = np.asarray(points, dtype=float)
    origin = np.asarray(center, dtype=float)
    attitude = np.asarray(rotation, dtype=float)
    if values.shape[-1:] != (3,):
        raise ValueError("points must have trailing dimension 3")
    if origin.shape != (3,) or attitude.shape != (3, 3):
        raise ValueError("center and rotation must have shapes (3,) and (3,3)")
    if not np.all(np.isfinite(values)) or not np.all(np.isfinite(origin)) or not np.all(np.isfinite(attitude)):
        raise ValueError("cuboid distance inputs must be finite")
    local = np.einsum("ij,...j->...i", attitude.T, values - origin)
    excess = np.maximum(np.abs(local) - np.asarray(body.half_extents), 0.0)
    squared = np.einsum("...i,...i->...", excess, excess)
    if squared.ndim == 0:
        return float(squared)
    return np.asarray(squared, dtype=float)


def whole_body_clearance_residual(
    points: ArrayLike,
    center: ArrayLike,
    rotation: ArrayLike,
    body: CuboidBody,
    clearance: float,
) -> float | FloatArray:
    """Return ``clearance**2 - distance(points, cuboid)**2``.

    Positive values are collisions/clearance violations; non-positive values
    satisfy the requested net separation.
    """

    required = float(clearance)
    if not np.isfinite(required) or required < 0.0:
        raise ValueError("clearance must be finite and nonnegative")
    distance2 = point_to_oriented_cuboid_distance_squared(points, center, rotation, body)
    return required * required - distance2


__all__ = [
    "CuboidBody",
    "point_to_oriented_cuboid_distance_squared",
    "whole_body_clearance_residual",
]
