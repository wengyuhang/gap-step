"""Fixed-plane non-convex windows that rotate only about their own normal."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

from nonconvex_timevarying_window.sc_dynatogt.preprocessing import PreprocessedGate


FloatArray = NDArray[np.float64]


def rotation_2d(theta: float) -> FloatArray:
    cosine, sine = math.cos(float(theta)), math.sin(float(theta))
    return np.asarray(((cosine, -sine), (sine, cosine)), dtype=float)


def basis_from_normal(normal: ArrayLike) -> tuple[FloatArray, FloatArray]:
    """Construct a right-handed orthonormal plane basis ``E`` and normal."""

    n = np.asarray(normal, dtype=float)
    if n.shape != (3,) or not np.all(np.isfinite(n)) or np.linalg.norm(n) < 1.0e-12:
        raise ValueError("normal must be a finite nonzero three-vector")
    n = n / np.linalg.norm(n)
    reference = np.asarray((0.0, 0.0, 1.0))
    if abs(float(reference @ n)) > 0.92:
        reference = np.asarray((1.0, 0.0, 0.0))
    e1 = np.cross(reference, n)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    return np.column_stack((e1, e2)), n


@dataclass(frozen=True)
class RotatingWindow:
    """A fixed-centre, fixed-plane aperture with uniform normal-axis spin."""

    name: str
    gate: PreprocessedGate
    center: ArrayLike
    plane_basis: ArrayLike
    normal: ArrayLike
    theta0: float
    omega: float
    thickness: float
    rho: float

    def __post_init__(self) -> None:
        center = np.asarray(self.center, dtype=float)
        basis = np.asarray(self.plane_basis, dtype=float)
        normal = np.asarray(self.normal, dtype=float)
        if center.shape != (3,) or basis.shape != (3, 2) or normal.shape != (3,):
            raise ValueError("center, plane_basis and normal must have shapes (3,), (3,2), (3,)")
        if not np.all(np.isfinite(center)) or not np.all(np.isfinite(basis)) or not np.all(np.isfinite(normal)):
            raise ValueError("window pose must be finite")
        if not np.allclose(basis.T @ basis, np.eye(2), atol=1.0e-10):
            raise ValueError("plane_basis columns must be orthonormal")
        if not np.isclose(np.linalg.norm(normal), 1.0, atol=1.0e-10):
            raise ValueError("normal must have unit length")
        if not np.allclose(basis.T @ normal, 0.0, atol=1.0e-10):
            raise ValueError("normal must be perpendicular to plane_basis")
        if float(np.cross(basis[:, 0], basis[:, 1]) @ normal) < 1.0 - 1.0e-10:
            raise ValueError("plane_basis and normal must form a right-handed frame")
        if not np.all(np.isfinite((self.theta0, self.omega, self.thickness, self.rho))):
            raise ValueError("window scalar parameters must be finite")
        if self.thickness <= 0.0 or self.rho <= 0.0:
            raise ValueError("thickness and rho must be positive")
        if not np.isclose(self.gate.safe_region.distance, self.rho, rtol=0.0, atol=1.0e-10):
            raise ValueError("preprocessed gate inset must equal rho")
        object.__setattr__(self, "center", center)
        object.__setattr__(self, "plane_basis", basis)
        object.__setattr__(self, "normal", normal)

    @property
    def clearance_distance(self) -> float:
        return 0.5 * float(self.thickness) + float(self.rho)

    @property
    def safe_polygon(self) -> FloatArray:
        return self.gate.safe_polygon

    @property
    def physical_polygon(self) -> FloatArray:
        return self.gate.dense_boundary.vertices

    def angle(self, absolute_time: float) -> float:
        return float(self.theta0 + self.omega * absolute_time)

    def local_point(self, latent: ArrayLike) -> FloatArray:
        return np.asarray(self.gate.sc_map.map_unconstrained(latent), dtype=float)

    def rotated_basis(self, absolute_time: float) -> FloatArray:
        return self.plane_basis @ rotation_2d(self.angle(absolute_time))

    def world_point(self, local_point: ArrayLike, absolute_time: float, z: float = 0.0) -> FloatArray:
        q = np.asarray(local_point, dtype=float)
        if q.shape != (2,):
            raise ValueError("local_point must have shape (2,)")
        return self.center + self.rotated_basis(absolute_time) @ q + self.normal * float(z)

    def boundary_at(self, absolute_time: float, *, safe: bool = False, z: float = 0.0) -> FloatArray:
        polygon = self.safe_polygon if safe else self.physical_polygon
        return (
            self.center[None, :]
            + (self.rotated_basis(absolute_time) @ polygon.T).T
            + self.normal[None, :] * float(z)
        )


__all__ = ["RotatingWindow", "basis_from_normal", "rotation_2d"]
