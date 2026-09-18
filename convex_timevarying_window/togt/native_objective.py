"""Joint moving-window TOGT objective backed by released C++ gradients."""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np

from convex_timevarying_window.togt.native_backend import AnalyticTOGTCore
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import (
    add_traversal_time_gradients, backpropagate_to_k, durations_from_k,
    duration_jacobian_diagonal, k_from_durations, traversal_times,
)


@dataclass(frozen=True)
class NativeForward:
    k: np.ndarray
    d: tuple[np.ndarray, ...]
    durations: np.ndarray
    traversal_times: np.ndarray
    waypoints: np.ndarray
    local_points: np.ndarray
    trajectory: MincoSnap


class NativeJointTOGTObjective:
    def __init__(self, track, config):
        self.track = track
        self.config = config
        self.window_count = len(track.order)
        self.temporal_dimension = self.window_count + 1
        self.spatial_dimensions = tuple(w.aperture.dimension for w in track.windows)
        self.dimension = self.temporal_dimension + sum(self.spatial_dimensions)
        self.head = np.zeros((3, 4)); self.head[:, 0] = track.start
        self.tail = np.zeros((3, 4)); self.tail[:, 0] = track.goal
        self.core = AnalyticTOGTCore()
        self.invalid_trial_count = 0

    def split(self, x):
        values = np.asarray(x, dtype=float)
        if values.shape != (self.dimension,) or not np.all(np.isfinite(values)):
            raise ValueError(f"x must have shape ({self.dimension},) and be finite")
        spatial=values[self.temporal_dimension:]; d=[]; offset=0
        for wi in self.track.order:
            size=self.spatial_dimensions[wi]; d.append(spatial[offset:offset+size]); offset+=size
        return values[:self.temporal_dimension], tuple(d)

    def _geometry(self, x):
        k, d = self.split(x); durations = durations_from_k(k)
        crossings = traversal_times(durations, self.window_count)
        points = np.empty((self.window_count, 3)); local = np.empty((self.window_count, 2))
        jacobians = []; rates = np.empty_like(points)
        for i, wi in enumerate(self.track.order):
            point, q, jacobian, rate = self.track.windows[wi].point_and_jacobians(d[i], crossings[i])
            points[i], local[i], rates[i] = point, q, rate; jacobians.append(jacobian)
        return k, d, durations, crossings, points, local, jacobians, rates

    def value_and_gradient(self, x):
        k,d,durations,crossings,points,_,jacobians,rates = self._geometry(x)
        cost, grad_points, direct_t = self.core.value_and_gradient(self.head, self.tail,points,durations)
        grad_d = [jacobians[i].T @ grad_points[i] for i in range(self.window_count)]
        grad_crossings = np.einsum("ij,ij->i", grad_points, rates)
        grad_t = add_traversal_time_gradients(direct_t, grad_crossings)
        return cost, np.concatenate((backpropagate_to_k(k, grad_t), *grad_d))

    def scipy_value_and_gradient(self, x):
        try:
            return self.value_and_gradient(x)
        except (RuntimeError, ValueError, np.linalg.LinAlgError, FloatingPointError, OverflowError):
            self.invalid_trial_count += 1
            values = np.clip(np.asarray(x, dtype=float), -1e6, 1e6)
            scale = self.config.invalid_trial_cost
            return scale * (1 + 1e-12 * values @ values), 2e-12 * scale * values

    def initial_guess(self):
        d = tuple(self.track.windows[wi].aperture.initial_d() for wi in self.track.order)
        durations = np.ones(self.temporal_dimension)
        for _ in range(2):
            crossings = traversal_times(durations, self.window_count)
            points = [self.track.start]
            points += [self.track.windows[wi].to_point(d[i], crossings[i]) for i,wi in enumerate(self.track.order)]
            points.append(self.track.goal)
            lengths = np.linalg.norm(np.diff(np.asarray(points), axis=0), axis=1)
            durations = np.maximum(lengths / self.config.initial_speed, self.config.minimum_initial_duration)
        return np.concatenate((k_from_durations(durations), *d))

    def forward(self, x):
        k,d,durations,crossings,points,local,_,_ = self._geometry(x)
        trajectory = MincoSnap(BoundaryState(self.track.start), BoundaryState(self.track.goal), points, durations)
        return NativeForward(k.copy(),tuple(q.copy() for q in d),durations,crossings,points,local,trajectory)


class DynamicRightTailObjective:
    """A moving-window local TOGT problem with a free right-end PVAJ state.

    Unlike :class:`NativeJointTOGTObjective`, every gate is evaluated at its
    *absolute* predicted crossing time.  The last gate is the terminal point
    of MINCO and its velocity/acceleration/jerk are decision variables.  This
    makes it suitable for receding-horizon execution: the next solve receives
    the PVAJ actually produced by the first segment, rather than an invented
    rest state.
    """

    _TAIL_SCALE = np.array([1.0, 8.0, 20.0, 60.0])

    def __init__(self, windows, head: BoundaryState, absolute_start_time: float,
                 config, tail_seed: np.ndarray | None = None,
                 tail_prior_weight: float = 0.1):
        if not windows:
            raise ValueError("a rolling horizon needs at least one window")
        self.windows = tuple(windows)
        self.head_state = head
        self.head = head.matrix.T.copy()
        self.absolute_start_time = float(absolute_start_time)
        self.config = config
        self.window_count = len(self.windows)
        self.temporal_dimension = self.window_count
        self.spatial_dimensions = tuple(w.aperture.dimension for w in self.windows)
        self.dimension = self.temporal_dimension + sum(self.spatial_dimensions) + 9
        self.core = AnalyticTOGTCore()
        self.invalid_trial_count = 0
        self.tail_prior_weight = float(tail_prior_weight)
        self.tail_seed = (np.zeros((3, 4), dtype=float) if tail_seed is None
                          else np.asarray(tail_seed, dtype=float).copy())
        if self.tail_seed.shape != (3, 4):
            raise ValueError("tail_seed must have shape (3, 4)")

    def split(self, x):
        values = np.asarray(x, dtype=float)
        if values.shape != (self.dimension,) or not np.all(np.isfinite(values)):
            raise ValueError(f"x must have shape ({self.dimension},) and be finite")
        offset = self.temporal_dimension
        d = []
        for size in self.spatial_dimensions:
            d.append(values[offset:offset + size]); offset += size
        return values[:self.temporal_dimension], tuple(d), values[offset:].reshape(3, 3)

    def _geometry(self, x):
        k, d, vaj = self.split(x)
        durations = durations_from_k(k)
        crossings = self.absolute_start_time + np.cumsum(durations)
        points = np.empty((self.window_count, 3)); local = np.empty((self.window_count, 2))
        rates = np.empty_like(points); jacobians = []
        for i, window in enumerate(self.windows):
            point, q, jacobian, rate = window.point_and_jacobians(d[i], crossings[i])
            points[i], local[i], rates[i] = point, q, rate
            jacobians.append(jacobian)
        tail = np.empty((3, 4), dtype=float)
        tail[:, 0] = points[-1]
        tail[:, 1:] = vaj
        return k, d, durations, crossings, points, local, jacobians, rates, tail

    def value_and_gradient(self, x):
        k, d, durations, _, points, _, jacobians, rates, tail = self._geometry(x)
        # The last location is supplied as the terminal PVAJ position, so it
        # is deliberately omitted from the C++ intermediate-point array.
        cost, grad_inner, grad_duration, grad_tail = self.core.value_and_gradient_with_tail(
            self.head, tail, points[:-1], durations)
        grad_point = np.empty_like(points)
        if self.window_count > 1:
            grad_point[:-1] = grad_inner
        grad_point[-1] = grad_tail[:, 0]
        grad_d = [jacobians[i].T @ grad_point[i] for i in range(self.window_count)]
        # t_i = t_abs + sum_{j<=i} T_j.  The reverse cumulative sum is the
        # exact chain rule and includes the terminal window crossing.
        grad_crossing = np.einsum("ij,ij->i", grad_point, rates)
        grad_duration = grad_duration + np.cumsum(grad_crossing[::-1])[::-1]
        grad_vaj = grad_tail[:, 1:].copy()

        if self.tail_prior_weight:
            delta = tail - self.tail_seed
            scaled = delta / self._TAIL_SCALE[None, :]
            cost += self.tail_prior_weight * float(np.sum(scaled * scaled))
            prior = 2.0 * self.tail_prior_weight * delta / self._TAIL_SCALE[None, :]**2
            grad_d[-1] += jacobians[-1].T @ prior[:, 0]
            grad_duration += np.cumsum(np.array([float(prior[:, 0] @ rates[-1])]))[0]
            grad_vaj += prior[:, 1:]

        grad_k = duration_jacobian_diagonal(k) * grad_duration
        return cost, np.concatenate((grad_k, *grad_d, grad_vaj.reshape(-1)))

    def scipy_value_and_gradient(self, x):
        try:
            return self.value_and_gradient(x)
        except (RuntimeError, ValueError, np.linalg.LinAlgError, FloatingPointError, OverflowError):
            self.invalid_trial_count += 1
            values = np.clip(np.asarray(x, dtype=float), -1e6, 1e6)
            scale = self.config.invalid_trial_cost
            return scale * (1 + 1e-12 * values @ values), 2e-12 * scale * values

    def initial_guess(self):
        d = tuple(w.aperture.initial_d() for w in self.windows)
        durations = np.ones(self.window_count, dtype=float)
        for _ in range(3):
            crossings = self.absolute_start_time + np.cumsum(durations)
            points = [self.head_state.position]
            points.extend(w.to_point(d[i], crossings[i]) for i, w in enumerate(self.windows))
            lengths = np.linalg.norm(np.diff(np.asarray(points), axis=0), axis=1)
            durations = np.maximum(lengths / self.config.initial_speed,
                                   self.config.minimum_initial_duration)
        return np.concatenate((k_from_durations(durations), *d, self.tail_seed[:, 1:].reshape(-1)))

    def forward(self, x):
        k,d,durations,crossings,points,local,_,_,tail = self._geometry(x)
        trajectory = MincoSnap(self.head_state, BoundaryState.from_array(tail),
                               points[:-1], durations)
        return NativeForward(k.copy(), tuple(q.copy() for q in d), durations,
                             crossings, points, local, trajectory)
