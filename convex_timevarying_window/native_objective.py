"""Joint moving-window TOGT objective backed by released C++ gradients."""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np

from convex_timevarying_window.native_backend import AnalyticTOGTCore
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import (
    add_traversal_time_gradients, backpropagate_to_k, durations_from_k,
    k_from_durations, traversal_times,
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
