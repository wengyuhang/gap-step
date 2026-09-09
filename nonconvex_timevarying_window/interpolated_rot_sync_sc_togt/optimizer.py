"""Joint optimization for the two-SC-input crossing method."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from nonconvex_timevarying_window.rot_sync_sc_togt.optimizer import (
    RotSyncObjective,
    RotSyncOptimizationConfig,
    RotSyncOptimizationResult,
    _optimize_objective,
)
from nonconvex_timevarying_window.rot_sync_sc_togt.scenarios import RotSyncScenario
from nonconvex_timevarying_window.rot_sync_sc_togt.trajectory import CompositeTrajectory
from nonconvex_timevarying_window.sc_dynatogt.minco import MincoSnap
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import durations_from_k

from .trajectory import SCInputInterpolatedSyncSegment, SCInputSplineSyncSegment


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class InterpolatedRotSyncForwardPass:
    free_durations: FloatArray
    sync_durations: FloatArray
    latent_points: FloatArray
    local_points: FloatArray
    latent_entry_points: FloatArray
    latent_exit_points: FloatArray
    local_entry_points: FloatArray
    local_exit_points: FloatArray
    entry_times: FloatArray
    crossing_times: FloatArray
    exit_times: FloatArray
    trajectory: CompositeTrajectory


@dataclass(frozen=True)
class SplineRotSyncForwardPass:
    free_durations: FloatArray
    sync_durations: FloatArray
    latent_points: FloatArray
    local_points: FloatArray
    latent_entry_points: FloatArray
    latent_exit_points: FloatArray
    local_entry_points: FloatArray
    local_exit_points: FloatArray
    latent_control_points: FloatArray
    normal_shape_parameters: FloatArray
    normal_control_points: FloatArray
    entry_times: FloatArray
    crossing_times: FloatArray
    exit_times: FloatArray
    trajectory: CompositeTrajectory


class InterpolatedRotSyncObjective(RotSyncObjective):
    """Optimize ``[K_free,K_sync,d_entry,d_exit]`` with the original objective."""

    def __init__(
        self,
        scenario: RotSyncScenario,
        config: RotSyncOptimizationConfig | None = None,
    ) -> None:
        super().__init__(scenario, config)
        self.dimension = (self.window_count + 1) + self.window_count + 4 * self.window_count

    def split(
        self, x: ArrayLike
    ) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray]:
        values = np.asarray(x, dtype=float)
        if values.shape != (self.dimension,) or not np.all(np.isfinite(values)):
            raise ValueError(f"x must be finite with shape ({self.dimension},)")
        n = self.window_count
        free_k = values[: n + 1]
        sync_k = values[n + 1 : 2 * n + 1]
        latent_entry = values[2 * n + 1 : 4 * n + 1].reshape(n, 2)
        latent_exit = values[4 * n + 1 :].reshape(n, 2)
        return free_k, sync_k, latent_entry, latent_exit

    def initial_guess(self) -> FloatArray:
        original = RotSyncObjective.initial_guess(self)
        n = self.window_count
        free_and_sync = original[: 2 * n + 1]
        latent = original[2 * n + 1 :].reshape(n, 2)
        return np.concatenate(
            (free_and_sync, latent.reshape(-1), latent.reshape(-1))
        )

    def forward(self, x: ArrayLike) -> InterpolatedRotSyncForwardPass:
        free_k, sync_k, latent_entry, latent_exit = self.split(x)
        free_durations = durations_from_k(free_k)
        sync_durations = durations_from_k(sync_k)
        free_segments: list[MincoSnap] = []
        sync_segments: list[SCInputInterpolatedSyncSegment] = []
        entries, crossings, exits = [], [], []
        local_entry, local_midpoint, local_exit = [], [], []
        elapsed = 0.0
        current_state = self.scenario.start_state
        empty = np.empty((0, 3), dtype=float)
        for index, window in enumerate(self.scenario.windows):
            entry_time = elapsed + float(free_durations[index])
            sync = SCInputInterpolatedSyncSegment(
                window,
                latent_entry[index],
                latent_exit[index],
                entry_time,
                float(sync_durations[index]),
            )
            free_segments.append(
                MincoSnap(
                    current_state,
                    sync.entry_state,
                    empty,
                    np.asarray((free_durations[index],)),
                )
            )
            sync_segments.append(sync)
            entries.append(entry_time)
            crossings.append(entry_time + 0.5 * float(sync_durations[index]))
            elapsed = entry_time + float(sync_durations[index])
            exits.append(elapsed)
            local_entry.append(sync.local_entry_point)
            local_midpoint.append(sync.local_point)
            local_exit.append(sync.local_exit_point)
            current_state = sync.exit_state
        free_segments.append(
            MincoSnap(
                current_state,
                self.scenario.goal_state,
                empty,
                np.asarray((free_durations[-1],)),
            )
        )
        return InterpolatedRotSyncForwardPass(
            free_durations=free_durations,
            sync_durations=sync_durations,
            latent_points=0.5 * (latent_entry + latent_exit),
            local_points=np.asarray(local_midpoint),
            latent_entry_points=latent_entry.copy(),
            latent_exit_points=latent_exit.copy(),
            local_entry_points=np.asarray(local_entry),
            local_exit_points=np.asarray(local_exit),
            entry_times=np.asarray(entries),
            crossing_times=np.asarray(crossings),
            exit_times=np.asarray(exits),
            trajectory=CompositeTrajectory(free_segments, sync_segments),
        )


class SplineRotSyncObjective(RotSyncObjective):
    """Optimize degree-7 SC-input and monotone-normal crossing curves."""

    bezier_control_count = 8
    inner_control_count = 6
    normal_shape_dimension = 6
    latent_offset_scale = 1.0e-2

    def __init__(
        self,
        scenario: RotSyncScenario,
        config: RotSyncOptimizationConfig | None = None,
    ) -> None:
        super().__init__(scenario, config)
        # Per window: entry/exit (4), six planar offsets (12), and six
        # normal log-increment ratios (6), in addition to the time variables.
        self.dimension = 2 * self.window_count + 1 + 22 * self.window_count

    def split(
        self, x: ArrayLike
    ) -> tuple[
        FloatArray,
        FloatArray,
        FloatArray,
        FloatArray,
        FloatArray,
        FloatArray,
    ]:
        values = np.asarray(x, dtype=float)
        if values.shape != (self.dimension,) or not np.all(np.isfinite(values)):
            raise ValueError(f"x must be finite with shape ({self.dimension},)")
        n = self.window_count
        cursor = 0
        free_k = values[cursor : cursor + n + 1]
        cursor += n + 1
        sync_k = values[cursor : cursor + n]
        cursor += n
        latent_entry = values[cursor : cursor + 2 * n].reshape(n, 2)
        cursor += 2 * n
        latent_exit = values[cursor : cursor + 2 * n].reshape(n, 2)
        cursor += 2 * n
        latent_offsets = values[cursor : cursor + 12 * n].reshape(n, 6, 2)
        cursor += 12 * n
        normal_shape = values[cursor:].reshape(n, 6)
        return (
            free_k,
            sync_k,
            latent_entry,
            latent_exit,
            latent_offsets,
            normal_shape,
        )

    def pack_linear(
        self,
        free_k: ArrayLike,
        sync_k: ArrayLike,
        latent_entry: ArrayLike,
        latent_exit: ArrayLike,
    ) -> FloatArray:
        """Embed the old two-point linear crossing exactly in the spline space."""

        n = self.window_count
        return np.concatenate(
            (
                np.asarray(free_k, dtype=float).reshape(n + 1),
                np.asarray(sync_k, dtype=float).reshape(n),
                np.asarray(latent_entry, dtype=float).reshape(n, 2).reshape(-1),
                np.asarray(latent_exit, dtype=float).reshape(n, 2).reshape(-1),
                np.zeros(12 * n, dtype=float),
                np.zeros(6 * n, dtype=float),
            )
        )

    def initial_guess(self) -> FloatArray:
        original = RotSyncObjective.initial_guess(self)
        n = self.window_count
        free_k = original[: n + 1]
        sync_k = original[n + 1 : 2 * n + 1]
        latent = original[2 * n + 1 :].reshape(n, 2)
        return self.pack_linear(free_k, sync_k, latent, latent)

    def forward(self, x: ArrayLike) -> SplineRotSyncForwardPass:
        (
            free_k,
            sync_k,
            latent_entry,
            latent_exit,
            latent_offsets,
            normal_shape,
        ) = self.split(x)
        free_durations = durations_from_k(free_k)
        sync_durations = durations_from_k(sync_k)
        progress = np.linspace(0.0, 1.0, self.bezier_control_count)[None, :, None]
        latent_controls = (
            (1.0 - progress) * latent_entry[:, None, :]
            + progress * latent_exit[:, None, :]
        )
        latent_controls[:, 1:-1, :] += self.latent_offset_scale * latent_offsets

        free_segments: list[MincoSnap] = []
        sync_segments: list[SCInputSplineSyncSegment] = []
        entries, crossings, exits = [], [], []
        local_entry, local_crossing, local_exit = [], [], []
        normal_controls = []
        elapsed = 0.0
        current_state = self.scenario.start_state
        empty = np.empty((0, 3), dtype=float)
        for index, window in enumerate(self.scenario.windows):
            entry_time = elapsed + float(free_durations[index])
            sync = SCInputSplineSyncSegment(
                window,
                latent_controls[index],
                normal_shape[index],
                entry_time,
                float(sync_durations[index]),
            )
            free_segments.append(
                MincoSnap(
                    current_state,
                    sync.entry_state,
                    empty,
                    np.asarray((free_durations[index],)),
                )
            )
            sync_segments.append(sync)
            entries.append(entry_time)
            crossings.append(entry_time + sync.plane_crossing_time)
            elapsed = entry_time + float(sync_durations[index])
            exits.append(elapsed)
            local_entry.append(sync.local_entry_point)
            local_crossing.append(sync.local_point)
            local_exit.append(sync.local_exit_point)
            normal_controls.append(sync.normal_control_points)
            current_state = sync.exit_state
        free_segments.append(
            MincoSnap(
                current_state,
                self.scenario.goal_state,
                empty,
                np.asarray((free_durations[-1],)),
            )
        )
        crossing_latent = np.stack(
            [
                segment.latent_at(segment.plane_crossing_time)
                for segment in sync_segments
            ]
        )
        return SplineRotSyncForwardPass(
            free_durations=free_durations,
            sync_durations=sync_durations,
            latent_points=crossing_latent,
            local_points=np.asarray(local_crossing),
            latent_entry_points=latent_entry.copy(),
            latent_exit_points=latent_exit.copy(),
            local_entry_points=np.asarray(local_entry),
            local_exit_points=np.asarray(local_exit),
            latent_control_points=latent_controls.copy(),
            normal_shape_parameters=normal_shape.copy(),
            normal_control_points=np.asarray(normal_controls),
            entry_times=np.asarray(entries),
            crossing_times=np.asarray(crossings),
            exit_times=np.asarray(exits),
            trajectory=CompositeTrajectory(free_segments, sync_segments),
        )


def optimize_interpolated_track(
    scenario: RotSyncScenario,
    *,
    config: RotSyncOptimizationConfig | None = None,
    initial_x: ArrayLike | None = None,
) -> RotSyncOptimizationResult:
    """Run the existing L-BFGS driver on the two-input parameterization."""

    objective = InterpolatedRotSyncObjective(scenario, config)
    return _optimize_objective(objective, scenario, initial_x)


def optimize_spline_track(
    scenario: RotSyncScenario,
    *,
    config: RotSyncOptimizationConfig | None = None,
    initial_x: ArrayLike | None = None,
) -> RotSyncOptimizationResult:
    """Run the reused L-BFGS driver on the degree-7 crossing curves."""

    objective = SplineRotSyncObjective(scenario, config)
    return _optimize_objective(objective, scenario, initial_x)


__all__ = [
    "InterpolatedRotSyncForwardPass",
    "InterpolatedRotSyncObjective",
    "SplineRotSyncForwardPass",
    "SplineRotSyncObjective",
    "optimize_interpolated_track",
    "optimize_spline_track",
]
