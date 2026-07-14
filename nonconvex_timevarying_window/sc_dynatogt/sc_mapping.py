"""Schwarz--Christoffel maps from the unit disk to simple polygons.

This module deliberately implements the disk SC integral itself.  It does not
use harmonic coordinates, a triangle atlas, a Poisson solve, or any other
surrogate interior parameterisation.  The (expensive) prevertex problem is
solved offline; online evaluation only needs a one-dimensional complex
quadrature and the analytic SC derivative.

The implementation targets moderate vertex counts.  The disk parameter
problem is exponentially ill-conditioned for crowded prevertices, so fitting
has explicit vertex-count, minimum-gap, condition-number and reconstruction
checks instead of silently returning a bad map.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.optimize import least_squares
from scipy.special import roots_jacobi, roots_legendre
from shapely.geometry import Point, Polygon
from shapely.ops import polylabel


FloatArray = NDArray[np.float64]


class SCMappingError(RuntimeError):
    """Base class for failures which make an SC map unsafe to use."""


class SCParameterSolveError(SCMappingError):
    """The nonlinear disk prevertex problem did not meet its checks."""


class SCCrowdingError(SCParameterSolveError):
    """Prevertices are too crowded for reliable double-precision evaluation."""


def _clean_polygon(vertices: ArrayLike) -> FloatArray:
    """Validate a finite, counter-clockwise, simple polygon.

    A repeated closing point is accepted and removed.  Clockwise input is
    reversed, since the disk SC angle convention below assumes CCW order.
    """

    p = np.asarray(vertices, dtype=np.float64)
    if p.ndim != 2 or p.shape[1] != 2 or len(p) < 3:
        raise ValueError("vertices must have shape (n, 2), n >= 3")
    if not np.all(np.isfinite(p)):
        raise ValueError("vertices must be finite")
    if len(p) > 3 and np.linalg.norm(p[0] - p[-1]) <= 1e-14:
        p = p[:-1]
    edges = np.roll(p, -1, axis=0) - p
    scale = max(float(np.ptp(p, axis=0).max()), 1.0)
    if np.any(np.linalg.norm(edges, axis=1) <= 1e-13 * scale):
        raise ValueError("polygon contains a zero-length edge")
    polygon = Polygon(p)
    if not polygon.is_valid or polygon.is_empty:
        raise ValueError("vertices must form one simple non-self-intersecting polygon")
    area2 = float(np.sum(p[:, 0] * np.roll(p[:, 1], -1) - p[:, 1] * np.roll(p[:, 0], -1)))
    if abs(area2) <= 1e-13 * scale * scale:
        raise ValueError("polygon has zero area")
    if area2 < 0.0:
        p = p[::-1].copy()
    if len(polygon.interiors):
        raise ValueError("SC disk mapping requires a polygon without holes")
    return np.ascontiguousarray(p, dtype=np.float64)


def polygon_interior_angles(vertices: ArrayLike) -> FloatArray:
    """Return polygon interior angles ``beta`` in radians.

    For a CCW polygon the signed exterior turn at vertex ``k`` is
    ``atan2(cross(e_in,e_out), dot(e_in,e_out))`` and
    ``beta = pi - turn``.  Hence reflex vertices naturally have ``beta > pi``.
    """

    p = _clean_polygon(vertices)
    incoming = p - np.roll(p, 1, axis=0)
    outgoing = np.roll(p, -1, axis=0) - p
    cross = incoming[:, 0] * outgoing[:, 1] - incoming[:, 1] * outgoing[:, 0]
    dot = np.einsum("ij,ij->i", incoming, outgoing)
    turn = np.arctan2(cross, dot)
    beta = np.pi - turn
    if np.any(beta <= 1e-8) or np.any(beta >= 2.0 * np.pi - 1e-8):
        raise ValueError("polygon has a numerically degenerate interior angle")
    expected = (len(p) - 2) * np.pi
    if abs(float(beta.sum()) - expected) > 2e-8 * max(1.0, len(p)):
        raise ValueError("polygon interior angles are inconsistent")
    return beta.astype(np.float64)


def B(d: ArrayLike) -> FloatArray:
    """Map two unconstrained variables to the open unit disk.

    ``B(d) = d / sqrt(1 + ||d||^2)``.
    """

    x = np.asarray(d, dtype=np.float64)
    if x.shape != (2,) or not np.all(np.isfinite(x)):
        raise ValueError("d must be a finite vector with shape (2,)")
    # ``x @ x`` overflows already around 1e154.  Nested hypot performs the
    # same scaling without squaring a huge component.
    denominator = float(np.hypot(1.0, np.hypot(x[0], x[1])))
    result = x / denominator
    # At ||d|| above roughly eps**-1/2, rounding can erase the mathematically
    # positive ``+1`` in the denominator.  Preserve the open-disk contract.
    radius = float(np.hypot(result[0], result[1]))
    if radius >= 1.0:
        # Scaling by nextafter(1, 0) can itself round back to the same two
        # components.  Move each nonzero component inward by representable
        # steps until the computed hypot also satisfies the open-disk test.
        for _ in range(8):
            result = np.nextafter(result, np.zeros(2, dtype=np.float64))
            if float(np.hypot(result[0], result[1])) < 1.0:
                break
        else:  # Defensive fallback for unusual libm rounding behaviour.
            result *= 1.0 - 8.0 * np.finfo(float).eps
    return result


def jacobian_B(d: ArrayLike) -> FloatArray:
    """Analytic real Jacobian of :func:`B`."""

    x = np.asarray(d, dtype=np.float64)
    if x.shape != (2,) or not np.all(np.isfinite(x)):
        raise ValueError("d must be a finite vector with shape (2,)")
    inverse_scale = 1.0 / float(np.hypot(1.0, np.hypot(x[0], x[1])))
    scaled = x * inverse_scale
    # Algebraically this is s^-1/2 I - s^-3/2 x x^T.  Factoring out
    # s^-1/2 avoids both x*x and s**1.5 overflow.  At radii where the radial
    # eigenvalue is below the smallest representable float, returning zero is
    # the correct floating-point limit.
    return inverse_scale * (np.eye(2, dtype=np.float64) - np.outer(scaled, scaled))


def B_with_jacobian(d: ArrayLike) -> tuple[FloatArray, FloatArray]:
    """Return ``(B(d), J_B(d))`` without changing the documented formula."""

    return B(d), jacobian_B(d)


# Descriptive aliases used by higher-level window code.
unconstrained_to_disk = B
unconstrained_to_disk_jacobian = jacobian_B


def polylabel_point(vertices: ArrayLike, tolerance: float | None = None) -> FloatArray:
    """Find the pole of inaccessibility of a simple polygon with polylabel."""

    p = _clean_polygon(vertices)
    polygon = Polygon(p)
    diameter = max(float(np.linalg.norm(np.ptp(p, axis=0))), 1.0)
    tol = 1e-7 * diameter if tolerance is None else float(tolerance)
    if not np.isfinite(tol) or tol <= 0.0:
        raise ValueError("polylabel tolerance must be positive")
    q = polylabel(polygon, tolerance=tol)
    if q.is_empty or not polygon.contains(q):
        raise SCMappingError("polylabel failed to return a strict interior point")
    return np.asarray([q.x, q.y], dtype=np.float64)


def _to_complex(point: complex | ArrayLike) -> complex:
    if np.isscalar(point):
        z = complex(point)
    else:
        p = np.asarray(point, dtype=np.float64)
        if p.shape != (2,):
            raise ValueError("disk point must be complex or have shape (2,)")
        z = complex(float(p[0]), float(p[1]))
    if not np.isfinite(z.real) or not np.isfinite(z.imag):
        raise ValueError("disk point must be finite")
    return z


def _as_xy(z: complex) -> FloatArray:
    return np.asarray([z.real, z.imag], dtype=np.float64)


def _strict_open_complex(z: complex) -> complex:
    """Move a rounded unit-modulus complex value minimally into the disk."""

    value = complex(z)
    radius = float(np.hypot(value.real, value.imag))
    # A complex128 representation of an exact unit-circle prevertex can itself
    # have hypot one ULP below one.  Keep a handful of ULPs of separation so a
    # rounded automorphism image cannot equal that stored singularity.
    limit = 1.0 - 8.0 * np.finfo(float).eps
    if radius < limit:
        return value
    components = (
        np.asarray([value.real, value.imag], dtype=np.float64) * (limit / radius)
    )
    for _ in range(8):
        components = np.nextafter(components, np.zeros(2, dtype=np.float64))
        if float(np.hypot(components[0], components[1])) < limit:
            return complex(float(components[0]), float(components[1]))
    components *= 1.0 - 8.0 * np.finfo(float).eps
    return complex(float(components[0]), float(components[1]))


def _strict_open_complex_array(z: np.ndarray) -> np.ndarray:
    values = np.asarray(z, dtype=np.complex128)
    output = np.array(values, copy=True)
    radius = np.hypot(output.real, output.imag)
    limit = 1.0 - 8.0 * np.finfo(float).eps
    bad = radius >= limit
    if not np.any(bad):
        return output
    real = output.real.copy()
    imag = output.imag.copy()
    real[bad] *= limit / radius[bad]
    imag[bad] *= limit / radius[bad]
    for _ in range(8):
        if not np.any(bad):
            break
        real[bad] = np.nextafter(real[bad], 0.0)
        imag[bad] = np.nextafter(imag[bad], 0.0)
        radius = np.hypot(real, imag)
        bad = radius >= limit
    if np.any(bad):
        real[bad] *= 1.0 - 8.0 * np.finfo(float).eps
        imag[bad] *= 1.0 - 8.0 * np.finfo(float).eps
    output.real = real
    output.imag = imag
    return output


@dataclass(frozen=True)
class SCFitDiagnostics:
    """Numerical checks recorded with an offline SC parameter solution."""

    optimizer_success: bool
    optimizer_status: int
    optimizer_nfev: int
    parameter_residual_inf: float
    vertex_reconstruction_inf: float
    closure_error: float
    minimum_prevertex_gap: float
    maximum_gap_ratio: float
    derivative_condition_estimate: float


class SCDiskMap:
    """Numerical Schwarz--Christoffel disk map for a simple polygon.

    Construct maps with :meth:`fit`, then persist their offline parameters with
    :meth:`save`.  Direct construction is intentionally private-ish because a
    mismatched tuple of angles and prevertices is unsafe.
    """

    FORMAT_VERSION = 1
    _MAX_INTERIOR_QUADRATURE_ORDER = 512
    _MAX_FACTOR_ELEMENTS = 4_000_000

    def __init__(
        self,
        *,
        vertices: ArrayLike,
        alpha: ArrayLike,
        prevertices: ArrayLike,
        A: complex,
        C: complex,
        normalization: complex = 0.0j,
        q0: ArrayLike | None = None,
        quadrature_order: int = 64,
        diagnostics: SCFitDiagnostics | None = None,
    ) -> None:
        self.vertices = _clean_polygon(vertices)
        self.alpha = np.asarray(alpha, dtype=np.float64)
        self.prevertices = np.asarray(prevertices, dtype=np.complex128)
        n = len(self.vertices)
        if self.alpha.shape != (n,) or self.prevertices.shape != (n,):
            raise ValueError("alpha and prevertices must have one entry per vertex")
        if not np.all(np.isfinite(self.alpha)) or not np.all(np.isfinite(self.prevertices)):
            raise ValueError("SC parameters must be finite")
        if np.max(np.abs(np.abs(self.prevertices) - 1.0)) > 2e-10:
            raise ValueError("prevertices must lie on the unit circle")
        self.exponents = self.alpha - 1.0
        if np.any(self.exponents <= -1.0) or np.any(self.exponents >= 1.0):
            raise ValueError("SC exponents must lie strictly between -1 and 1")
        self.A = complex(A)
        self.C = complex(C)
        self.normalization = complex(normalization)
        if not all(
            np.isfinite(value)
            for value in (
                self.A.real,
                self.A.imag,
                self.C.real,
                self.C.imag,
                self.normalization.real,
                self.normalization.imag,
            )
        ):
            raise ValueError("A, C, and normalization must be finite")
        if abs(self.C) <= np.finfo(float).tiny:
            raise ValueError("C must be nonzero")
        if abs(self.normalization) >= 1.0:
            raise ValueError("normalization parameter must lie in the open disk")
        self.q0 = polylabel_point(self.vertices) if q0 is None else np.asarray(q0, dtype=np.float64)
        if self.q0.shape != (2,) or not np.all(np.isfinite(self.q0)):
            raise ValueError("q0 must be finite with shape (2,)")
        self.quadrature_order = int(quadrature_order)
        if self.quadrature_order < 16:
            raise ValueError("quadrature_order must be at least 16")
        if self.quadrature_order > self._MAX_INTERIOR_QUADRATURE_ORDER // 2:
            raise ValueError(
                "quadrature_order must be at most 256 so convergence can be "
                "checked against a higher-order rule"
            )
        self.diagnostics = diagnostics
        self._legendre_cache: dict[int, tuple[FloatArray, FloatArray]] = {}

    @property
    def n_vertices(self) -> int:
        return len(self.vertices)

    @property
    def beta(self) -> FloatArray:
        """Interior angles in radians."""

        return self.alpha * np.pi

    @property
    def prevertex_angles(self) -> FloatArray:
        return np.mod(np.angle(self.prevertices), 2.0 * np.pi)

    @staticmethod
    def _anchors(
        n: int, anchors: tuple[int, int, int] | None = None
    ) -> tuple[int, int, int, int]:
        values = (0, n // 3, (2 * n) // 3) if anchors is None else tuple(anchors)
        if len(values) != 3 or values[0] != 0 or not (0 < values[1] < values[2] < n):
            raise ValueError("SC anchors must satisfy 0 = a0 < a1 < a2 < n")
        return values[0], values[1], values[2], n

    @staticmethod
    def _perimeter_anchors(edge_lengths: FloatArray) -> tuple[int, int, int]:
        """Choose three existing vertices approximately one-third apart.

        Vertex-count thirds are a poor disk normalization for mixed boundaries
        containing one long line and many short curve samples.  The Möbius
        freedom permits any ordered triple; perimeter thirds avoid introducing
        artificial crowding while leaving every polygon vertex untouched.
        """

        lengths = np.asarray(edge_lengths, dtype=np.float64)
        n = len(lengths)
        cumulative = np.r_[0.0, np.cumsum(lengths)]
        total = float(cumulative[-1])
        first = int(np.argmin(np.abs(cumulative[1 : n - 1] - total / 3.0))) + 1
        if first >= n - 1:
            first = n - 2
        second_candidates = np.arange(first + 1, n)
        second = int(
            second_candidates[
                np.argmin(np.abs(cumulative[second_candidates] - 2.0 * total / 3.0))
            ]
        )
        return 0, first, second

    @classmethod
    def _decode_prevertices(
        cls,
        x: FloatArray,
        n: int,
        anchors: tuple[int, int, int] | None = None,
    ) -> tuple[np.ndarray, FloatArray]:
        """Decode positive gaps while fixing three disk-automorphism DOFs."""

        anchor_bounds = cls._anchors(n, anchors)
        gaps = np.empty(n, dtype=np.float64)
        cursor = 0
        for block in range(3):
            start, stop = anchor_bounds[block], anchor_bounds[block + 1]
            count = stop - start
            logits = np.r_[x[cursor : cursor + count - 1], 0.0]
            cursor += count - 1
            logits -= np.max(logits)
            weights = np.exp(logits)
            gaps[start:stop] = (2.0 * np.pi / 3.0) * weights / weights.sum()
        if cursor != n - 3:
            raise AssertionError("internal prevertex parameter count mismatch")
        theta = np.r_[0.0, np.cumsum(gaps[:-1])]
        return np.exp(1j * theta), gaps

    @classmethod
    def _decode_prevertices_with_jacobian(
        cls,
        x: FloatArray,
        n: int,
        anchors: tuple[int, int, int] | None = None,
    ) -> tuple[np.ndarray, FloatArray, FloatArray]:
        """Decode prevertices and the analytic ``d theta / d x`` matrix.

        Each of the three block sums is fixed at ``2*pi/3``.  The omitted
        final logit in every block removes the softmax translation nullspace.
        This leaves exactly ``n-3`` real disk-prevertex parameters.
        """

        anchor_bounds = cls._anchors(n, anchors)
        gaps = np.empty(n, dtype=np.float64)
        gap_jac = np.zeros((n, n - 3), dtype=np.float64)
        cursor = 0
        for block in range(3):
            start, stop = anchor_bounds[block], anchor_bounds[block + 1]
            count = stop - start
            logits = np.r_[x[cursor : cursor + count - 1], 0.0]
            logits -= np.max(logits)
            weights = np.exp(logits)
            probabilities = weights / weights.sum()
            block_gaps = (2.0 * np.pi / 3.0) * probabilities
            gaps[start:stop] = block_gaps
            if count > 1:
                local = -np.outer(block_gaps, probabilities[:-1])
                local[np.arange(count - 1), np.arange(count - 1)] += block_gaps[:-1]
                gap_jac[start:stop, cursor : cursor + count - 1] = local
            cursor += count - 1
        if cursor != n - 3:
            raise AssertionError("internal prevertex parameter count mismatch")
        theta = np.r_[0.0, np.cumsum(gaps[:-1])]
        theta_jac = np.zeros((n, n - 3), dtype=np.float64)
        if n > 1 and n > 3:
            theta_jac[1:] = np.cumsum(gap_jac[:-1], axis=0)
        return np.exp(1j * theta), gaps, theta_jac

    @staticmethod
    def _factor(z: complex | np.ndarray, prevertices: np.ndarray, exponents: FloatArray) -> complex | np.ndarray:
        """Principal analytic branch of the SC derivative factor in the disk.

        For ``|z| < 1``, every ``1-z/zeta_k`` has positive real part, so the
        principal logarithm is a single consistent analytic branch.
        """

        values = np.asarray(z, dtype=np.complex128)
        flat = values.reshape(-1)
        separation = np.min(
            np.abs(prevertices[None, :] - flat[:, None]), axis=1
        )
        near = separation < 1e-7
        result = np.empty(len(flat), dtype=np.complex128)
        if np.any(~near):
            regular = flat[~near]
            logs = np.log1p(-regular[:, None] / prevertices[None, :])
            result[~near] = np.exp(logs @ exponents)
        if np.any(near):
            # ``1-z/zeta`` can round to zero when z is one ULP inside and
            # aligned with zeta.  Subtract the stored complex numbers in
            # extended precision before taking the logarithm.
            local = np.asarray(flat[near], dtype=np.clongdouble)
            pre = np.asarray(prevertices, dtype=np.clongdouble)
            ratios = (pre[None, :] - local[:, None]) / pre[None, :]
            logs = np.log(ratios)
            extended = np.exp(
                np.sum(
                    logs
                    * np.asarray(exponents, dtype=np.longdouble)[None, :],
                    axis=1,
                )
            )
            result[near] = np.asarray(extended, dtype=np.complex128)
        out = result.reshape(values.shape)
        return complex(out) if values.ndim == 0 else out

    @classmethod
    def _edge_quadrature_data(cls, exponents: FloatArray, order: int) -> list[tuple[FloatArray, FloatArray, float]]:
        data = []
        n = len(exponents)
        for j in range(n):
            p_start, p_end = float(exponents[j]), float(exponents[(j + 1) % n])
            nodes, weights = roots_jacobi(order, p_end, p_start)
            t = 0.5 * (nodes + 1.0)
            scale = 2.0 ** (-(p_start + p_end + 1.0))
            data.append((t.astype(np.float64), weights.astype(np.float64), scale))
        return data

    @staticmethod
    def _edge_smooth_factor(
        t: FloatArray,
        z0: complex,
        z1: complex,
        j: int,
        jp: int,
        prevertices: np.ndarray,
        exponents: FloatArray,
        *,
        extended_precision: bool = False,
    ) -> np.ndarray:
        """SC edge factor after exact removal of both endpoint powers."""

        if extended_precision:
            # Crowded prevertices may differ by only 1e-11 while their real
            # and imaginary components are O(1).  Carry subtraction, path
            # interpolation and logarithms in the platform's extended type so
            # quadrature refinement measures integration error rather than
            # complex128 cancellation noise.
            pre = np.asarray(prevertices, dtype=np.clongdouble)
            local_t = np.asarray(t, dtype=np.longdouble)
            start, end = np.clongdouble(z0), np.clongdouble(z1)
            z = start + local_t * (end - start)
            ratios = (pre[None, :] - z[:, None]) / pre[None, :]
            ratios[:, j] = (start - end) / start
            ratios[:, jp] = (end - start) / end
            logs = np.log(ratios)
            return np.exp(
                np.sum(logs * np.asarray(exponents, dtype=np.longdouble)[None, :], axis=1)
            )
        z = z0 + t * (z1 - z0)
        ratios = 1.0 - z[:, None] / prevertices[None, :]
        ratios[:, j] = (z0 - z1) / z0
        ratios[:, jp] = (z1 - z0) / z1
        logs = np.log(ratios)
        return np.exp(logs @ exponents)

    @classmethod
    def _composite_edge_data(
        cls,
        prevertices: np.ndarray,
        exponents: FloatArray,
        edge_index: int,
        reference_order: int,
        *,
        proximity_threshold: float = 0.08,
    ) -> tuple[FloatArray, FloatArray] | None:
        """Build a strict endpoint-graded rule for a crowded edge.

        Besides the two algebraic endpoint singularities already handled by
        Gauss--Jacobi, a neighbouring prevertex can sit extremely close to an
        endpoint in chord parameter space.  A single global rule then
        converges impractically slowly.  Dyadic panels put every such exterior
        branch point a fixed number of panel widths away.  The first and last
        panels retain exact Jacobi endpoint weights; interior panels use
        Gauss--Legendre.
        """

        n = len(prevertices)
        j = int(edge_index)
        jp = (j + 1) % n
        z0, z1 = prevertices[j], prevertices[jp]
        delta = z1 - z0
        mask = np.ones(n, dtype=bool)
        mask[[j, jp]] = False
        chord_parameters = (prevertices[mask] - z0) / delta
        rho_start = float(np.min(np.abs(chord_parameters)))
        rho_end = float(np.min(np.abs(chord_parameters - 1.0)))
        if min(rho_start, rho_end) >= proximity_threshold:
            return None

        def levels(distance: float) -> int:
            if distance >= proximity_threshold:
                return 2
            return int(
                np.clip(
                    np.ceil(-np.log2(max(distance, np.finfo(float).eps))) + 4,
                    2,
                    52,
                )
            )

        start_levels, end_levels = levels(rho_start), levels(rho_end)
        boundaries = {0.0, 1.0}
        boundaries.update(2.0 ** (-level) for level in range(start_levels, 0, -1))
        boundaries.update(
            1.0 - 2.0 ** (-level) for level in range(1, end_levels + 1)
        )
        panel_boundaries = np.asarray(sorted(boundaries), dtype=np.float64)
        # Grading supplies most of the convergence.  Increasing this local
        # order together with the reference rule provides an independent,
        # strict convergence sequence for _checked_edge_integrals.
        order = max(16, min(160, int(np.ceil(reference_order / 4))))
        p0, p1 = float(exponents[j]), float(exponents[jp])
        all_t: list[np.ndarray] = []
        all_weights: list[np.ndarray] = []
        for panel, (left, right) in enumerate(
            zip(panel_boundaries[:-1], panel_boundaries[1:])
        ):
            width = float(right - left)
            if width <= 0.0:
                continue
            if panel == 0:
                nodes, weights = roots_jacobi(order, 0.0, p0)
                u = 0.5 * (nodes + 1.0)
                t = width * u
                base = (
                    width ** (p0 + 1.0)
                    * 2.0 ** (-(p0 + 1.0))
                    * weights
                    * np.power(1.0 - t, p1)
                )
            elif panel == len(panel_boundaries) - 2:
                nodes, weights = roots_jacobi(order, p1, 0.0)
                u = 0.5 * (nodes + 1.0)
                t = left + width * u
                base = (
                    width ** (p1 + 1.0)
                    * 2.0 ** (-(p1 + 1.0))
                    * weights
                    * np.power(t, p0)
                )
            else:
                nodes, weights = roots_legendre(order)
                t = left + 0.5 * width * (nodes + 1.0)
                base = (
                    0.5
                    * width
                    * weights
                    * np.power(t, p0)
                    * np.power(1.0 - t, p1)
                )
            all_t.append(np.asarray(t, dtype=np.float64))
            all_weights.append(np.asarray(base, dtype=np.float64))
        return np.concatenate(all_t), np.concatenate(all_weights)

    @classmethod
    def _edge_integrals(
        cls,
        prevertices: np.ndarray,
        exponents: FloatArray,
        quad_data: list[tuple[FloatArray, FloatArray, float]],
    ) -> np.ndarray:
        """Integrate between adjacent boundary prevertices along disk chords.

        Generalised Gauss--Jacobi quadrature removes both algebraic endpoint
        singularities exactly from the quadrature weight.  Chords stay inside
        the disk, where the principal logarithm has a stable common branch.
        """

        n = len(prevertices)
        result = np.empty(n, dtype=np.complex128)
        for j, (t, weights, scale) in enumerate(quad_data):
            jp = (j + 1) % n
            z0, z1 = prevertices[j], prevertices[jp]
            composite = cls._composite_edge_data(
                prevertices, exponents, j, len(t)
            )
            if composite is None:
                local_t, local_weights = t, scale * weights
            else:
                local_t, local_weights = composite
            smooth = cls._edge_smooth_factor(
                local_t,
                z0,
                z1,
                j,
                jp,
                prevertices,
                exponents,
                extended_precision=composite is not None,
            )
            result[j] = complex(
                np.clongdouble(z1 - z0)
                * np.sum(np.asarray(local_weights, dtype=np.longdouble) * smooth)
            )
        return result

    @classmethod
    def _edge_integrals_with_angle_jacobian(
        cls,
        prevertices: np.ndarray,
        exponents: FloatArray,
        quad_data: list[tuple[FloatArray, FloatArray, float]],
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return edge integrals and their analytic prevertex-angle Jacobian.

        For ``f(z)=prod_k(1-z/zeta_k)**exponents[k]`` and a chord point
        ``z(t)``, differentiation with respect to ``theta_k`` gives

        ``d log(f)/d theta_k = -z' sum_r e_r/(zeta_r-z)
                                + i e_k z/(zeta_k-z)``.

        Evaluating all angle derivatives alongside an integral therefore has
        the same asymptotic cost as one residual evaluation.  In particular,
        it avoids the ``n-3`` extra quadrature sweeps that finite-difference
        Jacobians require at every nonlinear iteration.
        """

        n = len(prevertices)
        integrals = np.empty(n, dtype=np.complex128)
        angle_jac = np.empty((n, n), dtype=np.complex128)
        for j, (t, weights, scale) in enumerate(quad_data):
            jp = (j + 1) % n
            z0, z1 = prevertices[j], prevertices[jp]
            delta = z1 - z0
            composite = cls._composite_edge_data(
                prevertices, exponents, j, len(t)
            )
            if composite is None:
                local_t, local_weights = t, scale * weights
            else:
                local_t, local_weights = composite
            smooth = cls._edge_smooth_factor(
                local_t,
                z0,
                z1,
                j,
                jp,
                prevertices,
                exponents,
                extended_precision=composite is not None,
            )
            work_complex = np.clongdouble if composite is not None else np.complex128
            work_float = np.longdouble if composite is not None else np.float64
            pre_work = np.asarray(prevertices, dtype=work_complex)
            t_work = np.asarray(local_t, dtype=work_float)
            z0_work, z1_work = work_complex(z0), work_complex(z1)
            delta_work = z1_work - z0_work
            z = z0_work + t_work * delta_work
            weighted = np.asarray(local_weights, dtype=work_float) * smooth
            integral_without_delta = np.sum(weighted)
            integrals[j] = complex(delta_work * integral_without_delta)

            denominators = pre_work[None, :] - z[:, None]
            # Contribution caused by moving zeta_k while the chord point is
            # held fixed.  Shape is (quadrature_order, n).
            fixed_z = (
                1j
                * z[:, None]
                * np.asarray(exponents, dtype=work_float)[None, :]
                / denominators
            )
            derivative = delta_work * np.sum(weighted[:, None] * fixed_z, axis=0)

            # Only the two chord endpoints move z(t).  Their path-motion
            # contribution is separated so the dense calculation above does
            # not materialise another order-by-n matrix.
            logarithmic_z_derivative = np.sum(
                np.asarray(exponents, dtype=work_float)[None, :] / denominators,
                axis=1,
            )
            zprime_start = (1.0 - t_work) * 1j * z0_work
            zprime_end = t_work * 1j * z1_work
            derivative[j] += delta_work * np.sum(
                weighted * (-zprime_start * logarithmic_z_derivative)
            )
            derivative[jp] += delta_work * np.sum(
                weighted * (-zprime_end * logarithmic_z_derivative)
            )

            # Differentiate the leading chord vector zeta_{j+1}-zeta_j.
            derivative[j] -= 1j * z0_work * integral_without_delta
            derivative[jp] += 1j * z1_work * integral_without_delta
            angle_jac[j] = np.asarray(derivative, dtype=np.complex128)
        return integrals, angle_jac

    @classmethod
    def _checked_edge_integrals(
        cls,
        prevertices: np.ndarray,
        exponents: FloatArray,
        *,
        initial_order: int,
        maximum_order: int = 512,
        relative_tolerance: float = 2e-9,
    ) -> tuple[np.ndarray, int]:
        """Adapt edge rules and fail instead of accepting an unconverged rule."""

        order = min(int(initial_order), int(maximum_order) // 2)
        previous = cls._edge_integrals(
            prevertices, exponents, cls._edge_quadrature_data(exponents, order)
        )
        while order < maximum_order:
            order = min(order * 2, int(maximum_order))
            current = cls._edge_integrals(
                prevertices, exponents, cls._edge_quadrature_data(exponents, order)
            )
            denominator = np.maximum(np.abs(current), 1e-14)
            if float(np.max(np.abs(current - previous) / denominator)) <= relative_tolerance:
                return current, order
            previous = current
        raise SCMappingError(
            f"SC boundary quadrature did not converge by order {maximum_order}"
        )

    @classmethod
    def _radial_boundary_integral(
        cls,
        endpoint: complex,
        endpoint_index: int,
        prevertices: np.ndarray,
        exponents: FloatArray,
        order: int,
    ) -> complex:
        p = float(exponents[endpoint_index])
        composite = cls._composite_radial_boundary_data(
            endpoint,
            endpoint_index,
            prevertices,
            exponents,
            order,
        )
        if composite is None:
            nodes, weights = roots_jacobi(order, p, 0.0)
            t = 0.5 * (nodes + 1.0)
            base_weights = 2.0 ** (-(p + 1.0)) * weights
            z = endpoint * t
            ratios = 1.0 - z[:, None] / prevertices[None, :]
            ratios[:, endpoint_index] = 1.0
            logs = np.log(ratios)
            smooth = np.exp(logs @ exponents)
            return complex(endpoint * np.dot(base_weights, smooth))

        t, base_weights = composite
        pre = np.asarray(prevertices, dtype=np.clongdouble)
        end = np.clongdouble(endpoint)
        local_t = np.asarray(t, dtype=np.longdouble)
        z = end * local_t
        ratios = (pre[None, :] - z[:, None]) / pre[None, :]
        ratios[:, endpoint_index] = 1.0
        logs = np.log(ratios)
        smooth = np.exp(
            np.sum(
                logs * np.asarray(exponents, dtype=np.longdouble)[None, :],
                axis=1,
            )
        )
        return complex(
            end
            * np.sum(np.asarray(base_weights, dtype=np.longdouble) * smooth)
        )

    @classmethod
    def _composite_radial_boundary_data(
        cls,
        endpoint: complex,
        endpoint_index: int,
        prevertices: np.ndarray,
        exponents: FloatArray,
        reference_order: int,
        *,
        proximity_threshold: float = 0.08,
    ) -> tuple[FloatArray, FloatArray] | None:
        """Grade a boundary-radius integral near neighbouring prevertices."""

        mask = np.ones(len(prevertices), dtype=bool)
        mask[int(endpoint_index)] = False
        singular_parameters = prevertices[mask] / endpoint
        proximity = float(np.min(np.abs(singular_parameters - 1.0)))
        if proximity >= proximity_threshold:
            return None
        levels = int(
            np.clip(
                np.ceil(-np.log2(max(proximity, np.finfo(float).eps))) + 4,
                2,
                52,
            )
        )
        boundaries = np.asarray(
            [0.0]
            + [1.0 - 2.0 ** (-level) for level in range(1, levels + 1)]
            + [1.0],
            dtype=np.float64,
        )
        order = max(16, min(160, int(np.ceil(reference_order / 4))))
        exponent = float(exponents[endpoint_index])
        all_t: list[np.ndarray] = []
        all_weights: list[np.ndarray] = []
        for panel, (left, right) in enumerate(zip(boundaries[:-1], boundaries[1:])):
            width = float(right - left)
            if panel == len(boundaries) - 2:
                nodes, weights = roots_jacobi(order, exponent, 0.0)
                u = 0.5 * (nodes + 1.0)
                t = left + width * u
                base = (
                    width ** (exponent + 1.0)
                    * 2.0 ** (-(exponent + 1.0))
                    * weights
                )
            else:
                nodes, weights = roots_legendre(order)
                t = left + 0.5 * width * (nodes + 1.0)
                base = (
                    0.5
                    * width
                    * weights
                    * np.power(1.0 - t, exponent)
                )
            all_t.append(np.asarray(t, dtype=np.float64))
            all_weights.append(np.asarray(base, dtype=np.float64))
        return np.concatenate(all_t), np.concatenate(all_weights)

    @classmethod
    def _checked_radial_boundary_integral(
        cls,
        endpoint: complex,
        endpoint_index: int,
        prevertices: np.ndarray,
        exponents: FloatArray,
        *,
        initial_order: int,
        maximum_order: int = 512,
        relative_tolerance: float = 2e-11,
    ) -> tuple[complex, int]:
        order = min(int(initial_order), int(maximum_order) // 2)
        previous = cls._radial_boundary_integral(
            endpoint, endpoint_index, prevertices, exponents, order
        )
        while order < maximum_order:
            order = min(order * 2, int(maximum_order))
            current = cls._radial_boundary_integral(
                endpoint, endpoint_index, prevertices, exponents, order
            )
            if abs(current - previous) <= relative_tolerance * max(abs(current), 1e-14):
                return current, order
            previous = current
        raise SCMappingError(
            f"SC radial boundary quadrature did not converge by order {maximum_order}"
        )

    @classmethod
    def fit(
        cls,
        vertices: ArrayLike,
        *,
        quadrature_order: int = 64,
        max_nfev: int = 1200,
        parameter_tolerance: float = 2e-7,
        reconstruction_tolerance: float = 2e-5,
        minimum_gap: float = 1e-12,
        maximum_gap_ratio: float = 1e12,
        maximum_condition: float = 1e11,
        max_vertices: int = 3200,
        polylabel_tolerance: float | None = None,
        inverse_tolerance: float = 2e-8,
    ) -> "SCDiskMap":
        """Solve the disk SC parameter problem offline.

        Three well-separated prevertices are fixed at cube roots of unity to
        remove disk automorphism freedom.  Remaining positive angular gaps are
        represented by block softmaxes.  A nonlinear least-squares solve makes
        the SC chord-integral lengths proportional to polygon side lengths.

        ``SCCrowdingError`` is raised for a large vertex count, collapsed gap,
        excessive gap ratio, or ill-conditioned optimizer Jacobian.  These are
        deliberate double-precision limits, not recoverable online warnings.
        """

        p = _clean_polygon(vertices)
        n = len(p)
        if n > int(max_vertices):
            raise SCCrowdingError(
                f"{n} vertices exceed the configured SC limit {max_vertices}; "
                "simplify/resample the boundary or use a multiprecision SC solver"
            )
        if quadrature_order < 24:
            raise ValueError("quadrature_order must be at least 24 while fitting")
        beta = polygon_interior_angles(p)
        alpha = beta / np.pi
        exponents = alpha - 1.0
        target_edges = np.roll(p, -1, axis=0) - p
        target_lengths = np.linalg.norm(target_edges, axis=1)
        anchors = cls._perimeter_anchors(target_lengths)
        target_log = np.log(target_lengths)
        target_log -= target_log.mean()
        # Strongly reflex polygons can already have gaps around 1e-4 with only
        # eight vertices.  A 96-point Jacobi rule is still cheap offline and
        # avoids making the nonlinear residual itself quadrature-noise limited.
        solve_order = max(int(quadrature_order), 96)
        quad_data = cls._edge_quadrature_data(exponents, solve_order)

        cache_x: FloatArray | None = None
        cache_residual: FloatArray | None = None
        cache_jacobian: FloatArray | None = None

        def residual_and_jacobian(x: FloatArray) -> tuple[FloatArray, FloatArray]:
            nonlocal cache_x, cache_residual, cache_jacobian
            if cache_x is not None and np.array_equal(x, cache_x):
                assert cache_residual is not None and cache_jacobian is not None
                return cache_residual, cache_jacobian
            pre, _, theta_jac = cls._decode_prevertices_with_jacobian(
                x, n, anchors
            )
            integrals, angle_derivatives = cls._edge_integrals_with_angle_jacobian(
                pre, exponents, quad_data
            )
            lengths = np.maximum(np.abs(integrals), np.finfo(float).tiny)
            current = np.log(lengths)
            current -= current.mean()
            values = np.asarray(current - target_log, dtype=np.float64)

            # d log|I| = Re(dI/I).  The residual removes its edge-wise mean,
            # exactly as the value calculation removes the unknown SC scale.
            log_length_angle_jac = np.real(angle_derivatives / integrals[:, None])
            log_length_angle_jac -= log_length_angle_jac.mean(axis=0, keepdims=True)
            jacobian = np.asarray(log_length_angle_jac @ theta_jac, dtype=np.float64)
            cache_x = np.array(x, copy=True)
            cache_residual = values
            cache_jacobian = jacobian
            return values, jacobian

        def residual(x: FloatArray) -> FloatArray:
            return residual_and_jacobian(x)[0]

        def residual_jacobian(x: FloatArray) -> FloatArray:
            return residual_and_jacobian(x)[1]

        x0 = np.zeros(n - 3, dtype=np.float64)
        if n == 3:
            # Fixing three boundary prevertices removes all disk-automorphism
            # degrees of freedom, so a triangle has no nonlinear parameters.
            prevertices, gaps = cls._decode_prevertices(x0, n, anchors)
            optimizer_success, optimizer_status, optimizer_nfev = True, 0, 1
            jacobian_rank, condition = 0, 1.0
        else:
            def solve(start: FloatArray) -> Any:
                return least_squares(
                    residual,
                    start,
                    jac=residual_jacobian,
                    method="trf",
                    bounds=(-24.0, 24.0),
                    xtol=1e-12,
                    ftol=1e-12,
                    gtol=1e-12,
                    max_nfev=int(max_nfev),
                    x_scale="jac",
                )

            result = solve(x0)
            best_error = float(np.max(np.abs(result.fun)))
            if not result.success or not np.isfinite(best_error) or best_error > parameter_tolerance:
                # Deterministic fallbacks derived from target side lengths.
                # They are only attempted after the symmetric start fails, so
                # ordinary and high-resolution smooth boundaries pay no
                # multi-start overhead.
                side_start = np.empty(n - 3, dtype=np.float64)
                cursor = 0
                for block in range(3):
                    start_index, stop_index = cls._anchors(n, anchors)[block : block + 2]
                    block_lengths = target_lengths[start_index:stop_index]
                    count = len(block_lengths)
                    if count > 1:
                        logits = np.log(block_lengths[:-1] / block_lengths[-1])
                        side_start[cursor : cursor + count - 1] = np.clip(logits, -12.0, 12.0)
                    cursor += count - 1
                candidates = [0.5 * side_start, side_start]
                for start in candidates:
                    candidate = solve(start)
                    candidate_error = float(np.max(np.abs(candidate.fun)))
                    if np.isfinite(candidate_error) and candidate_error < best_error:
                        result, best_error = candidate, candidate_error
            prevertices, gaps = cls._decode_prevertices(result.x, n, anchors)
            optimizer_success = bool(result.success)
            optimizer_status = int(result.status)
            optimizer_nfev = int(result.nfev)
            singular_values = np.linalg.svd(result.jac, compute_uv=False)
            threshold = np.finfo(float).eps * max(result.jac.shape) * max(1.0, singular_values[0])
            positive = singular_values[singular_values > threshold]
            jacobian_rank = int(len(positive))
            condition = float(positive[0] / positive[-1]) if len(positive) else float("inf")
        # An apparently good nonlinear solve can otherwise be an artefact of
        # an under-resolved endpoint-singular quadrature.  Recheck with
        # successively doubled rules and reject failure at the maximum order.
        edge_integrals, verified_edge_order = cls._checked_edge_integrals(
            prevertices,
            exponents,
            initial_order=solve_order,
        )
        verified_lengths = np.maximum(np.abs(edge_integrals), np.finfo(float).tiny)
        verified_log = np.log(verified_lengths)
        verified_log -= verified_log.mean()
        param_error = float(np.max(np.abs(verified_log - target_log)))
        min_gap = float(gaps.min())
        gap_ratio = float(gaps.max() / gaps.min())

        if not optimizer_success or not np.isfinite(param_error) or param_error > parameter_tolerance:
            raise SCParameterSolveError(
                f"SC prevertex solve failed: success={optimizer_success}, status={optimizer_status}, "
                f"residual_inf={param_error:.3e}, nfev={optimizer_nfev}"
            )
        if jacobian_rank != n - 3:
            raise SCParameterSolveError(
                f"SC parameter Jacobian is rank deficient: rank={jacobian_rank}, expected={n - 3}"
            )
        if min_gap < minimum_gap or gap_ratio > maximum_gap_ratio or condition > maximum_condition:
            raise SCCrowdingError(
                "SC prevertices are numerically crowded: "
                f"min_gap={min_gap:.3e}, gap_ratio={gap_ratio:.3e}, condition={condition:.3e}"
            )

        polygon_edges = target_edges[:, 0] + 1j * target_edges[:, 1]
        C = complex(np.vdot(edge_integrals, polygon_edges) / np.vdot(edge_integrals, edge_integrals))
        g0, _ = cls._checked_radial_boundary_integral(
            prevertices[0],
            0,
            prevertices,
            exponents,
            initial_order=max(solve_order, verified_edge_order // 2),
        )
        g_vertices = np.empty(n, dtype=np.complex128)
        g_vertices[0] = g0
        if n > 1:
            g_vertices[1:] = g0 + np.cumsum(edge_integrals[:-1])
        polygon_complex = p[:, 0] + 1j * p[:, 1]
        A = complex(np.mean(polygon_complex - C * g_vertices))
        diameter = max(float(np.linalg.norm(np.ptp(p, axis=0))), 1.0)
        reconstruction = float(np.max(np.abs(A + C * g_vertices - polygon_complex)) / diameter)
        closure = float(abs(C * np.sum(edge_integrals)) / diameter)
        if reconstruction > reconstruction_tolerance or closure > 5.0 * reconstruction_tolerance:
            raise SCParameterSolveError(
                "SC boundary reconstruction check failed: "
                f"vertex_error={reconstruction:.3e}, closure_error={closure:.3e}"
            )

        provisional = cls(
            vertices=p,
            alpha=alpha,
            prevertices=prevertices,
            A=A,
            C=C,
            normalization=0.0j,
            q0=polylabel_point(p, tolerance=polylabel_tolerance),
            quadrature_order=int(quadrature_order),
        )
        normalization = provisional._inverse_raw(provisional.q0, tolerance=inverse_tolerance)
        if abs(normalization) >= 1.0 - 1e-9:
            raise SCParameterSolveError("polylabel inverse landed on the disk boundary")
        diagnostics = SCFitDiagnostics(
            optimizer_success=optimizer_success,
            optimizer_status=optimizer_status,
            optimizer_nfev=optimizer_nfev,
            parameter_residual_inf=param_error,
            vertex_reconstruction_inf=reconstruction,
            closure_error=closure,
            minimum_prevertex_gap=min_gap,
            maximum_gap_ratio=gap_ratio,
            derivative_condition_estimate=condition,
        )
        return cls(
            vertices=p,
            alpha=alpha,
            prevertices=prevertices,
            A=A,
            C=C,
            normalization=normalization,
            q0=provisional.q0,
            quadrature_order=int(quadrature_order),
            diagnostics=diagnostics,
        )

    def _legendre(self, order: int) -> tuple[FloatArray, FloatArray]:
        if order not in self._legendre_cache:
            x, w = roots_legendre(order)
            self._legendre_cache[order] = (
                np.asarray(0.5 * (x + 1.0), dtype=np.float64),
                np.asarray(0.5 * w, dtype=np.float64),
            )
        return self._legendre_cache[order]

    def _integral_to_fixed(self, z: complex, order: int) -> complex:
        if z == 0.0j:
            return 0.0j
        t, weights = self._legendre(order)
        return complex(z * np.dot(weights, self._factor(t * z, self.prevertices, self.exponents)))

    def _automatic_batch_size(self, order: int) -> int:
        denominator = max(1, int(order) * self.n_vertices)
        return max(1, self._MAX_FACTOR_ELEMENTS // denominator)

    def _integral_to_fixed_many(
        self,
        z: np.ndarray,
        order: int,
        *,
        batch_size: int | None = None,
    ) -> np.ndarray:
        """Vectorised fixed-order radial integrals with bounded memory."""

        points = np.asarray(z, dtype=np.complex128).reshape(-1)
        result = np.empty(len(points), dtype=np.complex128)
        t, weights = self._legendre(order)
        size = self._automatic_batch_size(order) if batch_size is None else int(batch_size)
        if size <= 0:
            raise ValueError("batch_size must be positive")
        for start in range(0, len(points), size):
            stop = min(start + size, len(points))
            local = points[start:stop]
            samples = local[:, None] * t[None, :]
            factors = self._factor(samples, self.prevertices, self.exponents)
            result[start:stop] = local * (factors @ weights)
        return result

    def _prevertex_proximity(self, z: complex) -> float:
        """Distance, in radial-parameter space, to the nearest singularity."""

        if z == 0.0j:
            return float("inf")
        return float(np.min(np.abs(self.prevertices / z - 1.0)))

    def _tail_to_prevertex_fixed(
        self, z: complex, endpoint_index: int, order: int
    ) -> complex:
        """Integrate from an interior point to one prevertex.

        The sole endpoint singularity is removed analytically from the
        Gauss--Jacobi weight.  This is the stable complement of the exact
        radial boundary integral when ``z`` is extremely close to a vertex.
        """

        endpoint = self.prevertices[endpoint_index]
        exponent = float(self.exponents[endpoint_index])
        composite = self._composite_tail_data(z, endpoint_index, order)
        if composite is None:
            nodes, weights = roots_jacobi(int(order), exponent, 0.0)
            t = np.asarray(0.5 * (nodes + 1.0), dtype=np.longdouble)
            base_weights = (
                np.longdouble(2.0) ** (-(exponent + 1.0))
                * np.asarray(weights, dtype=np.longdouble)
            )
        else:
            local_t, local_weights = composite
            t = np.asarray(local_t, dtype=np.longdouble)
            base_weights = np.asarray(local_weights, dtype=np.longdouble)
        pre = np.asarray(self.prevertices, dtype=np.clongdouble)
        start, end = np.clongdouble(z), np.clongdouble(endpoint)
        delta = end - start
        path = start + t * delta
        ratios = (pre[None, :] - path[:, None]) / pre[None, :]
        # 1-path/endpoint = (1-z/endpoint)*(1-t).  Remove (1-t)^p,
        # leaving the constant first factor on its principal branch.
        ratios[:, endpoint_index] = delta / end
        logs = np.log(ratios)
        smooth = np.exp(
            np.sum(
                logs
                * np.asarray(self.exponents, dtype=np.longdouble)[None, :],
                axis=1,
            )
        )
        return complex(delta * np.sum(base_weights * smooth))

    def _composite_tail_data(
        self,
        z: complex,
        endpoint_index: int,
        reference_order: int,
        *,
        proximity_threshold: float = 0.08,
    ) -> tuple[FloatArray, FloatArray] | None:
        """Grade a short-tail integral around neighbouring prevertices."""

        endpoint = self.prevertices[endpoint_index]
        delta = endpoint - z
        mask = np.ones(self.n_vertices, dtype=bool)
        mask[int(endpoint_index)] = False
        singular_parameters = (self.prevertices[mask] - z) / delta
        rho_start = float(np.min(np.abs(singular_parameters)))
        rho_end = float(np.min(np.abs(singular_parameters - 1.0)))
        if min(rho_start, rho_end) >= proximity_threshold:
            return None

        def levels(distance: float) -> int:
            if distance >= proximity_threshold:
                return 2
            return int(
                np.clip(
                    np.ceil(-np.log2(max(distance, np.finfo(float).eps))) + 4,
                    2,
                    52,
                )
            )

        start_levels, end_levels = levels(rho_start), levels(rho_end)
        boundaries = {0.0, 1.0}
        boundaries.update(2.0 ** (-level) for level in range(start_levels, 0, -1))
        boundaries.update(
            1.0 - 2.0 ** (-level) for level in range(1, end_levels + 1)
        )
        panel_boundaries = np.asarray(sorted(boundaries), dtype=np.float64)
        # Unlike fit-time edge checks, the online convergence sequence starts
        # at order 24.  Halving the reference order gives distinct 12/24/48/96
        # panel rules instead of accidentally comparing identical minima.
        order = max(12, min(192, int(np.ceil(reference_order / 2))))
        exponent = float(self.exponents[endpoint_index])
        all_t: list[np.ndarray] = []
        all_weights: list[np.ndarray] = []
        for panel, (left, right) in enumerate(
            zip(panel_boundaries[:-1], panel_boundaries[1:])
        ):
            width = float(right - left)
            if panel == len(panel_boundaries) - 2:
                nodes, weights = roots_jacobi(order, exponent, 0.0)
                u = 0.5 * (nodes + 1.0)
                t = left + width * u
                base = (
                    width ** (exponent + 1.0)
                    * 2.0 ** (-(exponent + 1.0))
                    * weights
                )
            else:
                nodes, weights = roots_legendre(order)
                t = left + 0.5 * width * (nodes + 1.0)
                base = (
                    0.5
                    * width
                    * weights
                    * np.power(1.0 - t, exponent)
                )
            all_t.append(np.asarray(t, dtype=np.float64))
            all_weights.append(np.asarray(base, dtype=np.float64))
        return np.concatenate(all_t), np.concatenate(all_weights)

    def _integral_to_near_prevertex(self, z: complex) -> complex:
        distances = np.abs(self.prevertices / z - 1.0)
        endpoint_index = int(np.argmin(distances))
        # The fitted SC correspondence supplies this boundary antiderivative
        # exactly: vertex_j = A + C*G(zeta_j).  Re-integrating the full radius
        # online is both wasteful and poorly conditioned when another
        # prevertex is only a few ULPs away.  Fit/load already validate this
        # correspondence with independently converged boundary quadrature.
        vertex = complex(
            float(self.vertices[endpoint_index, 0]),
            float(self.vertices[endpoint_index, 1]),
        )
        boundary = (vertex - self.A) / self.C
        previous = self._tail_to_prevertex_fixed(z, endpoint_index, 24)
        for order in (48, 96, 192):
            current = self._tail_to_prevertex_fixed(z, endpoint_index, order)
            if abs(current - previous) <= 2e-13 * max(1.0, abs(boundary - current)):
                return boundary - current
            previous = current
        raise SCMappingError(
            "near-prevertex SC quadrature did not converge by order 192"
        )

    def _integral_to_composite_fixed(self, z: complex, order: int, levels: int) -> complex:
        """Geometrically graded Gauss--Legendre radial quadrature.

        A prevertex nearly aligned with ``z`` creates a branch point just past
        ``t=1`` in ``f(t*z)``.  Dyadic panels clustered at that endpoint keep
        the branch point a fixed number of panel widths away even for
        ``1-|z|`` near machine precision.
        """

        if z == 0.0j:
            return 0.0j
        nodes, weights = roots_legendre(int(order))
        left = [0.0]
        right = []
        for level in range(1, int(levels) + 1):
            boundary = 1.0 - 2.0 ** (-level)
            right.append(boundary)
            left.append(boundary)
        right.append(1.0)
        left_array = np.asarray(left, dtype=np.float64)
        right_array = np.asarray(right, dtype=np.float64)
        centers = 0.5 * (left_array + right_array)
        half_widths = 0.5 * (right_array - left_array)
        parameters = centers[:, None] + half_widths[:, None] * nodes[None, :]
        factors = self._factor(parameters * z, self.prevertices, self.exponents)
        panel_values = half_widths * (factors @ weights)
        return complex(z * np.sum(panel_values))

    def _integral_to_clustered(self, z: complex) -> complex:
        """Strict near-prevertex integral with an endpoint-graded mesh."""

        proximity = self._prevertex_proximity(z)
        if not np.isfinite(proximity):
            return 0.0j
        # Four additional dyadic levels put the closest exterior branch point
        # at least about sixteen final-panel widths away.  56 levels cover all
        # distinct double-precision radii below one without zero-width panels.
        levels = int(
            np.clip(np.ceil(-np.log2(max(proximity, np.finfo(float).eps))) + 4, 2, 52)
        )
        previous = self._integral_to_composite_fixed(z, 24, levels)
        for order in (48, 96, 192):
            current = self._integral_to_composite_fixed(z, order, levels)
            if abs(current - previous) <= 2e-13 * max(1.0, abs(current)):
                return current
            previous = current
        raise SCMappingError(
            "clustered SC quadrature did not converge by 192 points per panel"
        )

    def _integral_to(self, z: complex) -> complex:
        """Adaptive-order Gauss--Legendre integral along the radial segment."""

        if self._prevertex_proximity(z) < 0.05:
            return self._integral_to_near_prevertex(z)
        order = self.quadrature_order
        previous = self._integral_to_fixed(z, order)
        target = 2e-13 * max(1.0, abs(previous))
        while order < self._MAX_INTERIOR_QUADRATURE_ORDER:
            order *= 2
            current = self._integral_to_fixed(z, order)
            if abs(current - previous) <= target:
                return current
            previous = current
        # A global rule can still be slow for a branch point just outside its
        # conservative direct-switch threshold.  The independently checked
        # graded rule is the strict fallback; it raises if its own maximum
        # order does not converge.
        return self._integral_to_clustered(z)

    def _integral_to_many(
        self,
        z: np.ndarray,
        *,
        batch_size: int | None = None,
    ) -> np.ndarray:
        """Adaptive radial integration for a flat complex point array."""

        points = np.asarray(z, dtype=np.complex128).reshape(-1)
        order = self.quadrature_order
        previous = self._integral_to_fixed_many(points, order, batch_size=batch_size)
        unresolved = np.arange(len(points), dtype=np.int64)
        output = np.empty(len(points), dtype=np.complex128)
        while order < self._MAX_INTERIOR_QUADRATURE_ORDER and len(unresolved):
            order *= 2
            current = self._integral_to_fixed_many(
                points[unresolved], order, batch_size=batch_size
            )
            tolerance = 2e-13 * np.maximum(1.0, np.abs(previous[unresolved]))
            converged = np.abs(current - previous[unresolved]) <= tolerance
            output[unresolved[converged]] = current[converged]
            previous[unresolved] = current
            unresolved = unresolved[~converged]
        if len(unresolved):
            # Preserve strict scalar semantics for the small tail of points
            # close to prevertices.  This path is rare for area-uniform bulk
            # validation samples and avoids allocating a ragged panel tensor.
            for index in unresolved:
                point = complex(points[index])
                if self._prevertex_proximity(point) < 0.05:
                    output[index] = self._integral_to_near_prevertex(point)
                else:
                    output[index] = self._integral_to_clustered(point)
        return output

    def _raw_map_complex(self, z: complex) -> complex:
        if abs(z) >= 1.0:
            raise ValueError("SC map is defined on the open unit disk")
        return self.A + self.C * self._integral_to(z)

    def _raw_derivative(self, z: complex) -> complex:
        if abs(z) >= 1.0:
            raise ValueError("SC derivative is defined on the open unit disk")
        return complex(self.C * self._factor(z, self.prevertices, self.exponents))

    def _automorphism(self, z: complex) -> complex:
        a = self.normalization
        value = (z + a) / (1.0 + np.conj(a) * z)
        # For |z|,|a|<1 the exact value is strictly inside.  Very large
        # unconstrained variables make B(d) one ULP from the circle and the
        # quotient can round to modulus exactly one.  Preserve the exact open
        # disk invariant with the smallest representable inward correction.
        return _strict_open_complex(value)

    def _automorphism_many(self, z: np.ndarray) -> np.ndarray:
        a = self.normalization
        values = (z + a) / (1.0 + np.conj(a) * z)
        return _strict_open_complex_array(values)

    def _automorphism_derivative(self, z: complex) -> complex:
        a = self.normalization
        return complex((1.0 - abs(a) ** 2) / (1.0 + np.conj(a) * z) ** 2)

    def map_complex(self, z: complex | ArrayLike) -> complex:
        """Evaluate the normalized SC map and return a complex coordinate."""

        disk = _to_complex(z)
        if abs(disk) >= 1.0:
            raise ValueError("z must lie in the open unit disk")
        return self._raw_map_complex(self._automorphism(disk))

    def evaluate(self, z: complex | ArrayLike) -> FloatArray:
        """Evaluate ``Psi(z)`` and return ``[x, y]``."""

        return _as_xy(self.map_complex(z))

    @staticmethod
    def _complex_points(z: ArrayLike) -> np.ndarray:
        values = np.asarray(z)
        if np.iscomplexobj(values):
            if values.ndim != 1:
                raise ValueError("complex disk points must have shape (n,)")
            points = np.asarray(values, dtype=np.complex128)
        else:
            real = np.asarray(z, dtype=np.float64)
            if real.ndim != 2 or real.shape[1] != 2:
                raise ValueError("disk points must have shape (n, 2) or complex shape (n,)")
            points = real[:, 0] + 1j * real[:, 1]
        if not np.all(np.isfinite(points)):
            raise ValueError("disk points must be finite")
        if np.any(np.abs(points) >= 1.0):
            raise ValueError("all z points must lie in the open unit disk")
        return np.ascontiguousarray(points, dtype=np.complex128)

    def evaluate_many(
        self,
        z: ArrayLike,
        *,
        batch_size: int | None = None,
    ) -> FloatArray:
        """Evaluate many disk points as an ``(n, 2)`` array.

        Radial quadrature and SC factors are evaluated in bounded-memory
        batches.  Converged points leave the adaptive loop immediately, which
        makes million-sample validation practical without weakening the
        scalar integration tolerance.
        """

        points = self._complex_points(z)
        transformed = self._automorphism_many(points)
        integrals = self._integral_to_many(transformed, batch_size=batch_size)
        mapped = self.A + self.C * integrals
        return np.column_stack((mapped.real, mapped.imag)).astype(np.float64, copy=False)

    __call__ = evaluate

    def derivative(self, z: complex | ArrayLike) -> complex:
        """Analytic complex derivative ``Psi'(z)``."""

        disk = _to_complex(z)
        if abs(disk) >= 1.0:
            raise ValueError("z must lie in the open unit disk")
        transformed = self._automorphism(disk)
        return self._raw_derivative(transformed) * self._automorphism_derivative(disk)

    complex_derivative = derivative

    def jacobian(self, z: complex | ArrayLike) -> FloatArray:
        """Analytic 2x2 real Jacobian of ``Psi``."""

        fp = self.derivative(z)
        return np.asarray([[fp.real, -fp.imag], [fp.imag, fp.real]], dtype=np.float64)

    def jacobian_many(
        self,
        z: ArrayLike,
        *,
        batch_size: int | None = None,
    ) -> NDArray[np.float64]:
        """Return analytic real Jacobians with shape ``(n, 2, 2)``."""

        points = self._complex_points(z)
        size = (
            max(1, self._MAX_FACTOR_ELEMENTS // max(1, self.n_vertices))
            if batch_size is None
            else int(batch_size)
        )
        if size <= 0:
            raise ValueError("batch_size must be positive")
        derivatives = np.empty(len(points), dtype=np.complex128)
        a = self.normalization
        for start in range(0, len(points), size):
            stop = min(start + size, len(points))
            local = points[start:stop]
            transformed = self._automorphism_many(local)
            auto_derivative = (1.0 - abs(a) ** 2) / (
                1.0 + np.conj(a) * local
            ) ** 2
            derivatives[start:stop] = (
                self.C
                * self._factor(transformed, self.prevertices, self.exponents)
                * auto_derivative
            )
        output = np.empty((len(points), 2, 2), dtype=np.float64)
        output[:, 0, 0] = derivatives.real
        output[:, 0, 1] = -derivatives.imag
        output[:, 1, 0] = derivatives.imag
        output[:, 1, 1] = derivatives.real
        return output

    def map_unconstrained(self, d: ArrayLike) -> FloatArray:
        """Evaluate ``Psi(B(d))`` for the two optimizer variables."""

        return self.evaluate(B(d))

    def jacobian_unconstrained(self, d: ArrayLike) -> FloatArray:
        """Analytic Jacobian ``J_Psi(B(d)) J_B(d)``."""

        disk, jb = B_with_jacobian(d)
        return self.jacobian(disk) @ jb

    def map_unconstrained_with_jacobian(self, d: ArrayLike) -> tuple[FloatArray, FloatArray]:
        disk, jb = B_with_jacobian(d)
        return self.evaluate(disk), self.jacobian(disk) @ jb

    def _inverse_raw(self, point: ArrayLike, *, tolerance: float = 2e-8) -> complex:
        q = np.asarray(point, dtype=np.float64)
        if q.shape != (2,) or not np.all(np.isfinite(q)):
            raise ValueError("point must be finite with shape (2,)")
        target = complex(float(q[0]), float(q[1]))
        polygon = Polygon(self.vertices)
        if not polygon.covers(Point(q)):
            raise ValueError("inverse target is outside the polygon")
        scale = max(float(np.linalg.norm(np.ptp(self.vertices, axis=0))), 1.0)

        center_value = self._raw_map_complex(0.0j)
        linear = (target - center_value) / self._raw_derivative(0.0j)
        if abs(linear) > 0.8:
            linear *= 0.8 / abs(linear)

        def disk_to_u(z: complex) -> FloatArray:
            radius2 = min(abs(z) ** 2, 1.0 - 1e-12)
            return _as_xy(z) / np.sqrt(1.0 - radius2)

        starts = [np.zeros(2), disk_to_u(linear)]
        best: Any = None
        for start in starts:
            def fun(u: FloatArray) -> FloatArray:
                zxy = B(u)
                value = self._raw_map_complex(complex(zxy[0], zxy[1]))
                return np.asarray([value.real - target.real, value.imag - target.imag])

            def jac(u: FloatArray) -> FloatArray:
                zxy = B(u)
                z = complex(zxy[0], zxy[1])
                fp = self._raw_derivative(z)
                jf = np.asarray([[fp.real, -fp.imag], [fp.imag, fp.real]])
                return jf @ jacobian_B(u)

            result = least_squares(
                fun,
                start,
                jac=jac,
                method="trf",
                bounds=(-1e4, 1e4),
                xtol=1e-12,
                ftol=1e-12,
                gtol=1e-12,
                max_nfev=180,
            )
            if best is None or np.linalg.norm(result.fun) < np.linalg.norm(best.fun):
                best = result
        assert best is not None
        error = float(np.linalg.norm(best.fun) / scale)
        if not best.success or error > tolerance:
            raise SCParameterSolveError(
                f"SC inverse failed: success={best.success}, relative_error={error:.3e}"
            )
        zxy = B(best.x)
        return complex(float(zxy[0]), float(zxy[1]))

    def inverse_complex(self, point: ArrayLike, *, tolerance: float = 2e-8) -> complex:
        """Numerically invert the normalized SC map for an interior point."""

        raw = self._inverse_raw(point, tolerance=tolerance)
        a = self.normalization
        disk = (raw - a) / (1.0 - np.conj(a) * raw)
        if abs(disk) >= 1.0 + 1e-9:
            raise SCParameterSolveError("normalized SC inverse left the unit disk")
        return complex(disk)

    def inverse(self, point: ArrayLike, *, tolerance: float = 2e-8) -> FloatArray:
        return _as_xy(self.inverse_complex(point, tolerance=tolerance))

    def save(self, path: str | Path) -> None:
        """Save offline SC parameters to a non-pickle ``.npz`` archive."""

        d = self.diagnostics
        diag = np.asarray(
            [
                np.nan if d is None else float(d.optimizer_success),
                np.nan if d is None else float(d.optimizer_status),
                np.nan if d is None else float(d.optimizer_nfev),
                np.nan if d is None else d.parameter_residual_inf,
                np.nan if d is None else d.vertex_reconstruction_inf,
                np.nan if d is None else d.closure_error,
                np.nan if d is None else d.minimum_prevertex_gap,
                np.nan if d is None else d.maximum_gap_ratio,
                np.nan if d is None else d.derivative_condition_estimate,
            ],
            dtype=np.float64,
        )
        with Path(path).open("wb") as stream:
            np.savez_compressed(
                stream,
                format_version=np.asarray(self.FORMAT_VERSION, dtype=np.int64),
                vertices=self.vertices,
                alpha=self.alpha,
                prevertices=self.prevertices,
                A=np.asarray(self.A, dtype=np.complex128),
                C=np.asarray(self.C, dtype=np.complex128),
                normalization=np.asarray(self.normalization, dtype=np.complex128),
                q0=self.q0,
                quadrature_order=np.asarray(self.quadrature_order, dtype=np.int64),
                diagnostics=diag,
            )

    def _validate_loaded_parameters(self) -> None:
        """Perform geometric and analytic consistency checks after loading."""

        expected_alpha = polygon_interior_angles(self.vertices) / np.pi
        if not np.allclose(self.alpha, expected_alpha, rtol=0.0, atol=5e-11):
            raise SCMappingError("stored alpha values do not match polygon interior angles")

        angles = np.mod(np.angle(self.prevertices), 2.0 * np.pi)
        angle0_error = min(abs(float(angles[0])), abs(float(angles[0]) - 2.0 * np.pi))
        if angle0_error > 2e-10:
            raise SCMappingError("the first stored prevertex is not anchored at angle zero")
        gaps = np.mod(np.roll(angles, -1) - angles, 2.0 * np.pi)
        if np.any(gaps <= 1e-12) or not np.isclose(
            gaps.sum(), 2.0 * np.pi, rtol=0.0, atol=2e-9
        ):
            raise SCMappingError("stored prevertices are not in strict cyclic order")
        expected_anchors = np.exp(2j * np.pi * np.arange(3) / 3.0)
        anchor_indices = tuple(
            int(np.argmin(np.abs(self.prevertices - expected)))
            for expected in expected_anchors
        )
        anchor_errors = np.asarray(
            [
                abs(self.prevertices[index] - expected)
                for index, expected in zip(anchor_indices, expected_anchors)
            ]
        )
        if (
            np.max(anchor_errors) > 2e-9
            or anchor_indices[0] != 0
            or not (anchor_indices[0] < anchor_indices[1] < anchor_indices[2])
        ):
            raise SCMappingError("stored prevertices violate the three-point normalization")

        polygon = Polygon(self.vertices)
        if not polygon.contains(Point(self.q0)):
            raise SCMappingError("stored q0 is not strictly inside the polygon")

        initial_order = max(96, self.quadrature_order)
        edge_integrals, verified_order = self._checked_edge_integrals(
            self.prevertices,
            self.exponents,
            initial_order=initial_order,
        )
        g0, _ = self._checked_radial_boundary_integral(
            self.prevertices[0],
            0,
            self.prevertices,
            self.exponents,
            initial_order=max(initial_order, verified_order // 2),
        )
        g_vertices = np.empty(self.n_vertices, dtype=np.complex128)
        g_vertices[0] = g0
        if self.n_vertices > 1:
            g_vertices[1:] = g0 + np.cumsum(edge_integrals[:-1])
        polygon_complex = self.vertices[:, 0] + 1j * self.vertices[:, 1]
        diameter = max(float(np.linalg.norm(np.ptp(self.vertices, axis=0))), 1.0)
        reconstruction = float(
            np.max(np.abs(self.A + self.C * g_vertices - polygon_complex)) / diameter
        )
        closure = float(abs(self.C * np.sum(edge_integrals)) / diameter)
        if reconstruction > 5e-5 or closure > 1e-4:
            raise SCMappingError(
                "stored A/C/prevertices fail boundary reconstruction: "
                f"vertex_error={reconstruction:.3e}, closure_error={closure:.3e}"
            )

        try:
            center = self._raw_map_complex(self.normalization)
        except (ValueError, SCMappingError) as exc:
            raise SCMappingError("stored normalization cannot be evaluated") from exc
        q0_complex = complex(float(self.q0[0]), float(self.q0[1]))
        if abs(center - q0_complex) / diameter > 1e-6:
            raise SCMappingError("stored normalization does not satisfy Psi(0) = q0")

        if self.diagnostics is not None:
            d = self.diagnostics
            target_lengths = np.linalg.norm(
                np.roll(self.vertices, -1, axis=0) - self.vertices, axis=1
            )
            target_log = np.log(target_lengths)
            target_log -= target_log.mean()
            current_log = np.log(np.maximum(np.abs(edge_integrals), np.finfo(float).tiny))
            current_log -= current_log.mean()
            parameter_residual = float(np.max(np.abs(current_log - target_log)))
            computed = np.asarray(
                [parameter_residual, reconstruction, closure, gaps.min(), gaps.max() / gaps.min()]
            )
            stored = np.asarray(
                [
                    d.parameter_residual_inf,
                    d.vertex_reconstruction_inf,
                    d.closure_error,
                    d.minimum_prevertex_gap,
                    d.maximum_gap_ratio,
                ]
            )
            if not np.allclose(stored, computed, rtol=2e-5, atol=2e-11):
                raise SCMappingError("stored diagnostics are inconsistent with SC parameters")

    @classmethod
    def load(cls, path: str | Path) -> "SCDiskMap":
        """Load and validate parameters written by :meth:`save`."""

        required = {
            "format_version",
            "vertices",
            "alpha",
            "prevertices",
            "A",
            "C",
            "normalization",
            "q0",
            "quadrature_order",
            "diagnostics",
        }

        def scalar(array: np.ndarray, name: str) -> Any:
            value = np.asarray(array)
            if value.shape != ():
                raise SCMappingError(f"stored {name} must be a scalar")
            return value.item()

        try:
            with np.load(Path(path), allow_pickle=False) as data:
                missing = required.difference(data.files)
                if missing:
                    raise SCMappingError(
                        "SC parameter archive is missing: " + ", ".join(sorted(missing))
                    )
                version = int(scalar(data["format_version"], "format_version"))
                if version != cls.FORMAT_VERSION:
                    raise SCMappingError(f"unsupported SC parameter format version {version}")
                diag_values = np.asarray(data["diagnostics"], dtype=np.float64)
                if diag_values.shape != (9,):
                    raise SCMappingError("stored diagnostics must have shape (9,)")
                diagnostics = None
                if np.all(np.isnan(diag_values)):
                    diagnostics = None
                elif np.all(np.isfinite(diag_values)):
                    if diag_values[0] not in (0.0, 1.0):
                        raise SCMappingError("stored optimizer_success is invalid")
                    if (
                        diag_values[2] < 0.0
                        or not float(diag_values[1]).is_integer()
                        or not float(diag_values[2]).is_integer()
                        or np.any(diag_values[3:] < 0.0)
                    ):
                        raise SCMappingError("stored diagnostics contain invalid values")
                    diagnostics = SCFitDiagnostics(
                        optimizer_success=bool(diag_values[0]),
                        optimizer_status=int(diag_values[1]),
                        optimizer_nfev=int(diag_values[2]),
                        parameter_residual_inf=float(diag_values[3]),
                        vertex_reconstruction_inf=float(diag_values[4]),
                        closure_error=float(diag_values[5]),
                        minimum_prevertex_gap=float(diag_values[6]),
                        maximum_gap_ratio=float(diag_values[7]),
                        derivative_condition_estimate=float(diag_values[8]),
                    )
                else:
                    raise SCMappingError("stored diagnostics mix finite and non-finite values")

                mapping = cls(
                    vertices=np.asarray(data["vertices"], dtype=np.float64),
                    alpha=np.asarray(data["alpha"], dtype=np.float64),
                    prevertices=np.asarray(data["prevertices"], dtype=np.complex128),
                    A=complex(scalar(data["A"], "A")),
                    C=complex(scalar(data["C"], "C")),
                    normalization=complex(scalar(data["normalization"], "normalization")),
                    q0=np.asarray(data["q0"], dtype=np.float64),
                    quadrature_order=int(
                        scalar(data["quadrature_order"], "quadrature_order")
                    ),
                    diagnostics=diagnostics,
                )
        except SCMappingError:
            raise
        except (KeyError, OSError, TypeError, ValueError, OverflowError) as exc:
            raise SCMappingError(f"invalid SC parameter archive: {exc}") from exc

        mapping._validate_loaded_parameters()
        return mapping


__all__ = [
    "B",
    "B_with_jacobian",
    "SCCrowdingError",
    "SCDiskMap",
    "SCFitDiagnostics",
    "SCMappingError",
    "SCParameterSolveError",
    "jacobian_B",
    "polygon_interior_angles",
    "polylabel_point",
    "unconstrained_to_disk",
    "unconstrained_to_disk_jacobian",
]
