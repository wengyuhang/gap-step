"""TOGT-compatible MINCO minimum-snap trajectory parameterization.

The original TOGT implementation uses ``MincoSnap`` (``s=4``): every
trajectory piece is a degree-seven polynomial and the integral of squared
snap is minimized.  This module mirrors that construction.  In particular,
it is *not* a collection of independent quintic/Hermite interpolants.

Coefficients are stored in ascending-power order::

    p_i(t) = c[i, 0] + c[i, 1] t + ... + c[i, 7] t**7,
    0 <= t <= T_i.

The start and finish are constrained through position, velocity,
acceleration and jerk (PVAJ).  Intermediate variables constrain positions;
the MINCO linear system supplies the remaining derivatives and enforces the
optimality/continuity conditions.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import factorial
from typing import Callable, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray


_DIM = 3
_DEGREE = 7
_NUM_COEFF = _DEGREE + 1
_MAX_CONSTRAINED_DERIVATIVE = 3  # PVAJ


def _zero3() -> NDArray[np.float64]:
    return np.zeros(_DIM, dtype=float)


def _vector3(value: ArrayLike, name: str) -> NDArray[np.generic]:
    array = np.asarray(value)
    if array.shape != (_DIM,):
        raise ValueError(f"{name} must have shape (3,), got {array.shape}")
    if not np.all(np.isfinite(np.real(array))) or not np.all(
        np.isfinite(np.imag(array))
    ):
        raise ValueError(f"{name} must contain finite values")
    return array.copy()


@dataclass(frozen=True)
class BoundaryState:
    """Position and its first three derivatives at a trajectory endpoint."""

    position: ArrayLike
    velocity: ArrayLike = field(default_factory=_zero3)
    acceleration: ArrayLike = field(default_factory=_zero3)
    jerk: ArrayLike = field(default_factory=_zero3)

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", _vector3(self.position, "position"))
        object.__setattr__(self, "velocity", _vector3(self.velocity, "velocity"))
        object.__setattr__(
            self, "acceleration", _vector3(self.acceleration, "acceleration")
        )
        object.__setattr__(self, "jerk", _vector3(self.jerk, "jerk"))

    @property
    def matrix(self) -> NDArray[np.generic]:
        """Return a ``(4, 3)`` PVAJ array."""

        return np.stack(
            (self.position, self.velocity, self.acceleration, self.jerk), axis=0
        )

    @classmethod
    def from_array(cls, state: ArrayLike) -> "BoundaryState":
        """Create a state from position/PVA/PVAJ or C++ ``(3,4)`` data."""

        array = np.asarray(state)
        if array.shape == (_DIM,):
            return cls(array)
        if array.shape == (_DIM, 4):
            array = array.T
        if array.ndim == 2 and array.shape[1] == _DIM and 1 <= array.shape[0] <= 4:
            padded = np.zeros((4, _DIM), dtype=np.result_type(array.dtype, float))
            padded[: array.shape[0]] = array
            array = padded
        if array.shape != (4, _DIM):
            raise ValueError(
                "boundary state must have shape (3,), (1..4, 3), or (3, 4); "
                f"got {array.shape}"
            )
        return cls(array[0], array[1], array[2], array[3])


# A descriptive alias used in some trajectory literature.
PVAJState = BoundaryState


@dataclass(frozen=True)
class TrajectorySamples:
    """Vectorized samples of a MINCO trajectory."""

    time: NDArray[np.float64]
    position: NDArray[np.generic]
    velocity: NDArray[np.generic]
    acceleration: NDArray[np.generic]
    jerk: NDArray[np.generic]
    snap: NDArray[np.generic]
    crackle: NDArray[np.generic]

    @property
    def p(self) -> NDArray[np.generic]:
        return self.position

    @property
    def v(self) -> NDArray[np.generic]:
        return self.velocity

    @property
    def a(self) -> NDArray[np.generic]:
        return self.acceleration

    @property
    def j(self) -> NDArray[np.generic]:
        return self.jerk

    @property
    def s(self) -> NDArray[np.generic]:
        return self.snap


def _coerce_state(
    state: BoundaryState | Mapping[str, ArrayLike] | ArrayLike,
) -> BoundaryState:
    if isinstance(state, BoundaryState):
        return state
    if isinstance(state, Mapping):
        position = state.get("position", state.get("p"))
        if position is None:
            raise ValueError("boundary-state mapping requires 'position'")
        return BoundaryState(
            position=position,
            velocity=state.get("velocity", state.get("v", _zero3())),
            acceleration=state.get("acceleration", state.get("a", _zero3())),
            jerk=state.get("jerk", state.get("j", _zero3())),
        )
    return BoundaryState.from_array(state)


def _coerce_waypoints(points: ArrayLike, expected: int) -> NDArray[np.generic]:
    array = np.asarray(points)
    if array.size == 0:
        array = np.empty((0, _DIM), dtype=np.result_type(array.dtype, float))
    elif array.ndim == 1 and expected == 1 and array.shape == (_DIM,):
        array = array.reshape(1, _DIM)
    elif array.ndim == 2 and array.shape == (expected, _DIM):
        # Canonical Python layout.  Check it first because the three-point
        # case is square and therefore otherwise ambiguous.
        pass
    elif array.ndim == 2 and array.shape == (_DIM, expected):
        # Match the 3 x (N-1) layout of the C++ implementation.
        array = array.T
    if array.shape != (expected, _DIM):
        raise ValueError(
            f"intermediate_points must have shape ({expected}, 3), "
            f"got {array.shape}"
        )
    if not np.all(np.isfinite(np.real(array))) or not np.all(
        np.isfinite(np.imag(array))
    ):
        raise ValueError("intermediate_points must contain finite values")
    return array.copy()


def derivative_basis(
    t: ArrayLike, derivative: int = 0, degree: int = _DEGREE
) -> NDArray[np.generic]:
    """Polynomial derivative basis in ascending-power order.

    The returned shape is ``np.shape(t) + (degree + 1,)``.  The function
    intentionally preserves complex values so it can be used by the
    complex-step gradient path.
    """

    if derivative < 0 or derivative > degree:
        raise ValueError(f"derivative must be between 0 and {degree}")
    time = np.asarray(t)
    dtype = np.result_type(time.dtype, float)
    basis = np.zeros(time.shape + (degree + 1,), dtype=dtype)
    for power in range(derivative, degree + 1):
        scale = factorial(power) / factorial(power - derivative)
        basis[..., power] = scale * time ** (power - derivative)
    return basis


def _require_torch():
    """Import PyTorch only when the reverse-mode backend is requested."""

    try:
        import torch
    except ImportError as exc:  # pragma: no cover - exercised without the extra
        raise ImportError(
            "the autodiff MINCO gradient backend requires PyTorch; install "
            "the dependency or select backend='complex_step'"
        ) from exc
    return torch


def _torch_derivative_basis(time, derivative: int):
    """Torch counterpart of :func:`derivative_basis` for a scalar time."""

    values = []
    for power in range(_NUM_COEFF):
        if power < derivative:
            values.append(time.new_zeros(()))
        else:
            scale = factorial(power) / factorial(power - derivative)
            values.append(scale * time ** (power - derivative))
    return _require_torch().stack(values)


class MincoSnap:
    """Degree-seven, non-uniform-time MINCO trajectory used by TOGT.

    The class supports both direct construction and the two-stage style of
    the C++ solver::

        traj = MincoSnap(start, finish, points, durations)

    or::

        traj = MincoSnap()
        traj.set_conditions(start, finish, piece_num)
        traj.set_parameters(points, durations)
    """

    degree = _DEGREE
    derivative_order = 4

    def __init__(
        self,
        start_state: BoundaryState | Mapping[str, ArrayLike] | ArrayLike | None = None,
        end_state: BoundaryState | Mapping[str, ArrayLike] | ArrayLike | None = None,
        intermediate_points: ArrayLike | None = None,
        durations: ArrayLike | None = None,
    ) -> None:
        self._start_state: BoundaryState | None = None
        self._end_state: BoundaryState | None = None
        self._expected_piece_num: int | None = None
        self._durations: NDArray[np.generic] | None = None
        self._waypoints: NDArray[np.generic] | None = None
        self._coefficients: NDArray[np.generic] | None = None
        self._system_matrix: NDArray[np.generic] | None = None

        supplied = (
            start_state is not None,
            end_state is not None,
            intermediate_points is not None,
            durations is not None,
        )
        if any(supplied) and not all(supplied):
            raise ValueError(
                "direct construction requires start_state, end_state, "
                "intermediate_points, and durations"
            )
        if all(supplied):
            assert start_state is not None and end_state is not None
            assert intermediate_points is not None and durations is not None
            self.set_conditions(start_state, end_state)
            self.set_parameters(intermediate_points, durations)

    def set_conditions(
        self,
        start_state: BoundaryState | Mapping[str, ArrayLike] | ArrayLike,
        end_state: BoundaryState | Mapping[str, ArrayLike] | ArrayLike,
        piece_num: int | None = None,
    ) -> "MincoSnap":
        """Set endpoint PVAJ constraints and optionally the piece count."""

        if piece_num is not None and piece_num < 1:
            raise ValueError("piece_num must be at least one")
        self._start_state = _coerce_state(start_state)
        self._end_state = _coerce_state(end_state)
        self._expected_piece_num = piece_num
        self._durations = None
        self._waypoints = None
        self._coefficients = None
        self._system_matrix = None
        return self

    # C++ spelling retained as a small compatibility convenience.
    setConditions = set_conditions

    def set_parameters(
        self, intermediate_points: ArrayLike, durations: ArrayLike
    ) -> "MincoSnap":
        """Solve the MINCO linear system for points and segment durations."""

        if self._start_state is None or self._end_state is None:
            raise RuntimeError("call set_conditions before set_parameters")

        times = np.asarray(durations)
        if times.ndim != 1 or times.size < 1:
            raise ValueError("durations must be a non-empty one-dimensional array")
        if not np.all(np.isfinite(np.real(times))) or not np.all(
            np.isfinite(np.imag(times))
        ):
            raise ValueError("durations must contain finite values")
        # During complex-step differentiation only the real part controls the
        # physical domain; the infinitesimal imaginary part must be retained.
        if np.any(np.real(times) <= 1.0e-9):
            raise ValueError("all durations must be strictly positive")

        piece_num = int(times.size)
        if (
            self._expected_piece_num is not None
            and piece_num != self._expected_piece_num
        ):
            raise ValueError(
                f"expected {self._expected_piece_num} pieces, got {piece_num}"
            )
        points = _coerce_waypoints(intermediate_points, piece_num - 1)

        coefficients, matrix = self._solve_coefficients(points, times)
        self._durations = times.copy()
        self._waypoints = points
        self._coefficients = coefficients
        self._system_matrix = matrix
        self._expected_piece_num = piece_num
        return self

    setParameters = set_parameters

    def _solve_coefficients(
        self, points: NDArray[np.generic], times: NDArray[np.generic]
    ) -> tuple[NDArray[np.generic], NDArray[np.generic]]:
        """Assemble the same 8N square system as drolib ``MincoSnap``."""

        assert self._start_state is not None and self._end_state is not None
        piece_num = int(times.size)
        dtype = np.result_type(
            times.dtype,
            points.dtype,
            self._start_state.matrix.dtype,
            self._end_state.matrix.dtype,
            float,
        )
        matrix = np.zeros((8 * piece_num, 8 * piece_num), dtype=dtype)
        rhs = np.zeros((8 * piece_num, _DIM), dtype=dtype)

        # Head PVAJ.
        for derivative in range(4):
            matrix[derivative, :8] = derivative_basis(
                np.asarray(0, dtype=dtype), derivative
            )
            rhs[derivative] = self._start_state.matrix[derivative]

        for index in range(piece_num - 1):
            row = 8 * index + 4
            col = 8 * index
            next_col = col + 8
            duration = times[index]

            # Natural MINCO junction conditions: continuity of derivatives
            # four, five and six.  These are the Euler-Lagrange conditions for
            # the integral of squared fourth derivative.
            for offset, derivative in enumerate((4, 5, 6)):
                matrix[row + offset, col : col + 8] = derivative_basis(
                    duration, derivative
                )
                matrix[row + offset, next_col : next_col + 8] = -derivative_basis(
                    np.asarray(0, dtype=dtype), derivative
                )

            # The end of this piece is the optimized intermediate point.
            matrix[row + 3, col : col + 8] = derivative_basis(duration, 0)
            rhs[row + 3] = points[index]

            # Position through jerk are continuous into the next piece.
            for offset, derivative in enumerate(range(4), start=4):
                matrix[row + offset, col : col + 8] = derivative_basis(
                    duration, derivative
                )
                matrix[row + offset, next_col : next_col + 8] = -derivative_basis(
                    np.asarray(0, dtype=dtype), derivative
                )

        # Tail PVAJ.
        last_col = 8 * (piece_num - 1)
        last_time = times[-1]
        for derivative in range(4):
            row = 8 * piece_num - 4 + derivative
            matrix[row, last_col : last_col + 8] = derivative_basis(
                last_time, derivative
            )
            rhs[row] = self._end_state.matrix[derivative]

        try:
            flat_coefficients = np.linalg.solve(matrix, rhs)
        except np.linalg.LinAlgError as exc:
            raise ValueError(
                "MINCO system is singular; check durations and boundary data"
            ) from exc
        coefficients = flat_coefficients.reshape(piece_num, 8, _DIM)
        return coefficients, matrix

    def _torch_parameterization(self):
        """Build the MINCO solve as one float64 reverse-mode graph.

        This private bridge is shared by the minimum-snap and complete TOGT
        objectives.  PyTorch is deliberately imported lazily so trajectory
        construction/evaluation itself remains a NumPy-only operation.
        """

        self._require_solved()
        assert self._waypoints is not None and self._durations is not None
        if np.iscomplexobj(self._waypoints) or np.iscomplexobj(self._durations):
            raise ValueError("autodiff gradients require a real base trajectory")

        torch = _require_torch()
        points = torch.tensor(
            np.asarray(self._waypoints, dtype=float),
            dtype=torch.float64,
            requires_grad=True,
        )
        durations = torch.tensor(
            np.asarray(self._durations, dtype=float),
            dtype=torch.float64,
            requires_grad=True,
        )
        coefficients = self._torch_solve_coefficients(points, durations)
        return torch, points, durations, coefficients

    def _torch_solve_coefficients(self, points, durations):
        """Differentiable equivalent of :meth:`_solve_coefficients`."""

        assert self._start_state is not None and self._end_state is not None
        torch = _require_torch()
        piece_num = int(durations.numel())
        matrix = durations.new_zeros((8 * piece_num, 8 * piece_num))
        rhs = durations.new_zeros((8 * piece_num, _DIM))
        zero = durations.new_zeros(())
        start = torch.as_tensor(
            np.asarray(self._start_state.matrix, dtype=float),
            dtype=durations.dtype,
            device=durations.device,
        )
        finish = torch.as_tensor(
            np.asarray(self._end_state.matrix, dtype=float),
            dtype=durations.dtype,
            device=durations.device,
        )

        for derivative in range(4):
            matrix[derivative, :8] = _torch_derivative_basis(zero, derivative)
            rhs[derivative] = start[derivative]

        for index in range(piece_num - 1):
            row = 8 * index + 4
            col = 8 * index
            next_col = col + 8
            duration = durations[index]
            for offset, derivative in enumerate((4, 5, 6)):
                matrix[row + offset, col : col + 8] = _torch_derivative_basis(
                    duration, derivative
                )
                matrix[row + offset, next_col : next_col + 8] = (
                    -_torch_derivative_basis(zero, derivative)
                )

            matrix[row + 3, col : col + 8] = _torch_derivative_basis(duration, 0)
            rhs[row + 3] = points[index]
            for offset, derivative in enumerate(range(4), start=4):
                matrix[row + offset, col : col + 8] = _torch_derivative_basis(
                    duration, derivative
                )
                matrix[row + offset, next_col : next_col + 8] = (
                    -_torch_derivative_basis(zero, derivative)
                )

        last_col = 8 * (piece_num - 1)
        for derivative in range(4):
            row = 8 * piece_num - 4 + derivative
            matrix[row, last_col : last_col + 8] = _torch_derivative_basis(
                durations[-1], derivative
            )
            rhs[row] = finish[derivative]

        flat_coefficients = torch.linalg.solve(matrix, rhs)
        return flat_coefficients.reshape(piece_num, _NUM_COEFF, _DIM)

    @staticmethod
    def _torch_snap_energy(coefficients, durations):
        """Exact snap integral expressed in differentiable tensor algebra."""

        energy = durations.new_zeros(())
        scales = (24.0, 120.0, 360.0, 840.0)
        for segment, duration in enumerate(durations.unbind()):
            snap_coefficients = _require_torch().stack(
                [
                    scale * coefficients[segment, power]
                    for scale, power in zip(scales, range(4, 8))
                ]
            )
            for left in range(4):
                for right in range(4):
                    energy = energy + (
                        (snap_coefficients[left] * snap_coefficients[right]).sum()
                        * duration ** (left + right + 1)
                        / (left + right + 1)
                    )
        return energy

    @property
    def solved(self) -> bool:
        return self._coefficients is not None

    def _require_solved(self) -> None:
        if not self.solved:
            raise RuntimeError("MINCO coefficients have not been solved")

    @property
    def start_state(self) -> BoundaryState:
        if self._start_state is None:
            raise RuntimeError("endpoint conditions have not been set")
        return self._start_state

    @property
    def end_state(self) -> BoundaryState:
        if self._end_state is None:
            raise RuntimeError("endpoint conditions have not been set")
        return self._end_state

    @property
    def num_segments(self) -> int:
        self._require_solved()
        assert self._durations is not None
        return int(self._durations.size)

    @property
    def piece_num(self) -> int:
        return self.num_segments

    @property
    def durations(self) -> NDArray[np.generic]:
        self._require_solved()
        assert self._durations is not None
        return self._durations.copy()

    @property
    def intermediate_points(self) -> NDArray[np.generic]:
        self._require_solved()
        assert self._waypoints is not None
        return self._waypoints.copy()

    @property
    def coefficients(self) -> NDArray[np.generic]:
        """Polynomial coefficients with shape ``(N, 8, 3)``."""

        self._require_solved()
        assert self._coefficients is not None
        return self._coefficients.copy()

    def get_coeffs(self) -> NDArray[np.generic]:
        return self.coefficients

    getCoeffs = get_coeffs

    @property
    def total_time(self) -> np.generic:
        self._require_solved()
        assert self._durations is not None
        return np.sum(self._durations)

    def evaluate_segment(
        self, segment: int, local_time: ArrayLike, derivative: int = 0
    ) -> NDArray[np.generic]:
        """Evaluate one piece at local time(s)."""

        self._require_solved()
        assert self._durations is not None and self._coefficients is not None
        if segment < 0 or segment >= self.num_segments:
            raise IndexError(f"segment index {segment} is out of range")
        if derivative < 0 or derivative > self.degree:
            raise ValueError(f"derivative must be in [0, {self.degree}]")

        local = np.asarray(local_time)
        real_local = np.real(local)
        duration = np.real(self._durations[segment])
        tolerance = 1.0e-10 * max(1.0, float(duration))
        if np.any(real_local < -tolerance) or np.any(real_local > duration + tolerance):
            raise ValueError("local_time lies outside the selected segment")
        basis = derivative_basis(local, derivative)
        return np.tensordot(basis, self._coefficients[segment], axes=([-1], [0]))

    def evaluate(self, time: ArrayLike, derivative: int = 0) -> NDArray[np.generic]:
        """Evaluate global trajectory time(s), including both endpoints."""

        self._require_solved()
        assert self._durations is not None and self._coefficients is not None
        query = np.asarray(time)
        query_shape = query.shape
        flat_query = query.reshape(-1)
        cumulative = np.concatenate(
            (np.zeros(1, dtype=self._durations.dtype), np.cumsum(self._durations))
        )
        total_real = float(np.real(cumulative[-1]))
        tolerance = 1.0e-10 * max(1.0, total_real)
        if np.any(np.real(flat_query) < -tolerance) or np.any(
            np.real(flat_query) > total_real + tolerance
        ):
            raise ValueError("time lies outside [0, total_time]")

        result_dtype = np.result_type(query.dtype, self._coefficients.dtype, float)
        output = np.empty((flat_query.size, _DIM), dtype=result_dtype)
        real_cumulative = np.real(cumulative)
        for output_index, instant in enumerate(flat_query):
            piece = int(
                np.searchsorted(real_cumulative[1:], np.real(instant), side="right")
            )
            piece = min(piece, self.num_segments - 1)
            local = instant - cumulative[piece]
            output[output_index] = self.evaluate_segment(piece, local, derivative)
        reshaped = output.reshape(query_shape + (_DIM,))
        return reshaped

    __call__ = evaluate

    def sample(
        self,
        *,
        times: ArrayLike | None = None,
        num_samples: int | None = None,
        samples_per_segment: int | None = None,
    ) -> TrajectorySamples:
        """Sample position through crackle on a continuous global time grid."""

        self._require_solved()
        if sum(x is not None for x in (times, num_samples, samples_per_segment)) > 1:
            raise ValueError(
                "specify only one of times, num_samples, or samples_per_segment"
            )
        if times is not None:
            grid = np.asarray(times, dtype=float)
            if grid.ndim != 1:
                raise ValueError("times must be one-dimensional")
        elif samples_per_segment is not None:
            if samples_per_segment < 2:
                raise ValueError("samples_per_segment must be at least two")
            assert self._durations is not None
            chunks: list[NDArray[np.float64]] = []
            elapsed = 0.0
            for index, duration in enumerate(np.real(self._durations)):
                local = np.linspace(0.0, float(duration), samples_per_segment)
                if index:
                    local = local[1:]
                chunks.append(elapsed + local)
                elapsed += float(duration)
            grid = np.concatenate(chunks)
        else:
            count = 101 if num_samples is None else num_samples
            if count < 2:
                raise ValueError("num_samples must be at least two")
            grid = np.linspace(0.0, float(np.real(self.total_time)), count)

        return TrajectorySamples(
            time=grid,
            position=self.evaluate(grid, 0),
            velocity=self.evaluate(grid, 1),
            acceleration=self.evaluate(grid, 2),
            jerk=self.evaluate(grid, 3),
            snap=self.evaluate(grid, 4),
            crackle=self.evaluate(grid, 5),
        )

    def snap_energy(self) -> np.generic:
        """Return the exact integral ``integral ||p''''(t)||^2 dt``.

        Products are deliberately non-Hermitian so the expression is
        holomorphic for complex-step differentiation.  For real trajectories
        this is the usual squared Euclidean norm.
        """

        self._require_solved()
        assert self._durations is not None and self._coefficients is not None
        dtype = np.result_type(self._durations.dtype, self._coefficients.dtype)
        energy = np.zeros((), dtype=dtype)
        for segment, duration in enumerate(self._durations):
            snap_coefficients = np.empty((4, _DIM), dtype=dtype)
            for index, power in enumerate(range(4, 8)):
                snap_coefficients[index] = (
                    factorial(power)
                    / factorial(power - 4)
                    * self._coefficients[segment, power]
                )
            for left in range(4):
                for right in range(4):
                    energy = energy + (
                        np.sum(snap_coefficients[left] * snap_coefficients[right])
                        * duration ** (left + right + 1)
                        / (left + right + 1)
                    )
        return energy[()]

    energy = snap_energy

    def get_energy(self) -> np.generic:
        return self.snap_energy()

    getEnergy = get_energy

    def parameter_gradient(
        self,
        cost_function: Callable[["MincoSnap"], complex | float],
        *,
        step: float = 1.0e-30,
    ) -> tuple[float, NDArray[np.float64], NDArray[np.float64]]:
        """Differentiate a scalar trajectory cost by complex step.

        ``cost_function`` must use analytic arithmetic (the helpers in
        :mod:`dynamics` do so) and return a real scalar for a real trajectory.
        The result is ``(value, dcost/dpoints, dcost/ddurations)`` and is ready
        to flatten for SciPy L-BFGS-B.
        """

        self._require_solved()
        if step <= 0.0:
            raise ValueError("step must be positive")
        assert self._waypoints is not None and self._durations is not None
        if np.iscomplexobj(self._waypoints) or np.iscomplexobj(self._durations):
            raise ValueError("parameter_gradient expects a real base trajectory")

        value_raw = cost_function(self)
        value = float(np.real(value_raw))
        point_gradient = np.zeros_like(self._waypoints, dtype=float)
        time_gradient = np.zeros_like(self._durations, dtype=float)

        for index in np.ndindex(self._waypoints.shape):
            trial_points = self._waypoints.astype(complex)
            trial_points[index] += 1j * step
            trial = MincoSnap(
                self.start_state,
                self.end_state,
                trial_points,
                self._durations,
            )
            point_gradient[index] = np.imag(cost_function(trial)) / step

        for index in range(self.num_segments):
            trial_times = self._durations.astype(complex)
            trial_times[index] += 1j * step
            trial = MincoSnap(
                self.start_state,
                self.end_state,
                self._waypoints,
                trial_times,
            )
            time_gradient[index] = np.imag(cost_function(trial)) / step

        return value, point_gradient, time_gradient

    def energy_with_grad(
        self,
        *,
        backend: str = "autodiff",
        step: float = 1.0e-30,
    ) -> tuple[float, NDArray[np.float64], NDArray[np.float64]]:
        """Return energy and gradients by points and durations.

        ``backend='autodiff'`` (the default) differentiates one MINCO linear
        solve in float64 reverse mode.  ``backend='complex_step'`` retains the
        independent, slower perturbation implementation for validation.
        """

        normalized_backend = backend.lower().replace("-", "_")
        if normalized_backend in {"complex", "complex_step"}:
            return self.parameter_gradient(
                lambda trajectory: trajectory.snap_energy(), step=step
            )
        if normalized_backend not in {"autodiff", "torch", "reverse_mode"}:
            raise ValueError(
                "backend must be 'autodiff' or 'complex_step', "
                f"got {backend!r}"
            )

        torch, points, durations, coefficients = self._torch_parameterization()
        energy = self._torch_snap_energy(coefficients, durations)
        point_gradient, time_gradient = torch.autograd.grad(
            energy, (points, durations), allow_unused=True
        )
        if point_gradient is None:
            point_array = np.zeros_like(self.intermediate_points, dtype=float)
        else:
            point_array = point_gradient.detach().cpu().numpy()
        if time_gradient is None:  # pragma: no cover - time always participates
            time_array = np.zeros_like(self.durations, dtype=float)
        else:
            time_array = time_gradient.detach().cpu().numpy()
        return float(energy.detach().cpu()), point_array, time_array

    get_energy_with_grads = energy_with_grad


# Descriptive aliases; ``MincoSnap`` remains the reference-compatible name.
MinimumSnapTrajectory = MincoSnap
MincoTrajectory = MincoSnap


def solve_minimum_snap(
    start_state: BoundaryState | Mapping[str, ArrayLike] | ArrayLike,
    end_state: BoundaryState | Mapping[str, ArrayLike] | ArrayLike,
    intermediate_points: ArrayLike,
    durations: ArrayLike,
) -> MincoSnap:
    """Convenience constructor for a solved TOGT MINCO trajectory."""

    return MincoSnap(start_state, end_state, intermediate_points, durations)


__all__ = [
    "BoundaryState",
    "MincoSnap",
    "MincoTrajectory",
    "MinimumSnapTrajectory",
    "PVAJState",
    "TrajectorySamples",
    "derivative_basis",
    "solve_minimum_snap",
]
