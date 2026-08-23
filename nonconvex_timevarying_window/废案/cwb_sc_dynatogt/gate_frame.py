"""Complete moving-gate frame adapter for the unchanged SC implementation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from nonconvex_timevarying_window.sc_dynatogt.environment import SCDynamicWindow


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class GateFrame:
    """Position, orthonormal in-plane basis, normal, scale, and derivatives."""

    center: FloatArray
    basis: FloatArray
    normal: FloatArray
    scale: float
    center_dot: FloatArray
    basis_dot: FloatArray
    normal_dot: FloatArray
    scale_dot: float


def frame_at(window: SCDynamicWindow, time: float) -> GateFrame:
    """Build a :class:`GateFrame` without mutating the base window class."""

    instant = float(time)
    if not np.isfinite(instant):
        raise ValueError("time must be finite")
    center, basis, scale, center_dot, basis_dot, scale_dot = window.state_at(instant)
    normal = np.cross(basis[:, 0], basis[:, 1])
    normal_dot = np.cross(basis_dot[:, 0], basis[:, 1]) + np.cross(
        basis[:, 0], basis_dot[:, 1]
    )
    if not np.all(np.isfinite(normal)) or abs(np.linalg.norm(normal) - 1.0) > 1.0e-7:
        raise ValueError("window basis does not define a finite unit normal")
    if not np.isfinite(scale) or scale <= 0.0:
        raise ValueError("window scale must be finite and positive")
    return GateFrame(
        np.asarray(center, dtype=float), np.asarray(basis, dtype=float), normal,
        float(scale), np.asarray(center_dot, dtype=float),
        np.asarray(basis_dot, dtype=float), normal_dot, float(scale_dot),
    )


__all__ = ["GateFrame", "frame_at"]
