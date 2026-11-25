"""Turn an accepted planner solution into sparse, time-free control privileges."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track


REPO = Path(__file__).resolve().parents[2]
DEFAULT_PLAN = (
    REPO
    / "convex_dynamic_seven_window_gazebo"
    / "trajectories"
    / "our_method_x500_accepted_100hz.npz"
)


@dataclass(frozen=True)
class GatePrivilege:
    local_point: np.ndarray
    crossing_velocity: np.ndarray
    crossing_direction: float
    nominal_segment_time: float


class PlannerPrivilege:
    """Sparse route information extracted from a hard-audit-accepted plan.

    The online policy receives gate crossing targets and velocities, but never a
    time-indexed desired vehicle state.  Consequently it cannot look up where
    the vehicle "should" be along the original trajectory.
    """

    def __init__(self, path: str | Path = DEFAULT_PLAN):
        self.path = Path(path).resolve()
        data = np.load(self.path)
        self.track, _ = build_track()
        self.crossing_times = np.asarray(data["traversal_times"], dtype=float)
        self.waypoints = np.asarray(data["waypoints_enu"], dtype=float)
        source_time = np.asarray(data["time"], dtype=float)
        source_velocity = np.asarray(data["velocity_enu"], dtype=float)
        self.start = np.asarray(data["position_enu"][0], dtype=float)
        self.goal = np.asarray(data["position_enu"][-1], dtype=float)
        segment_times = np.diff(np.r_[0.0, self.crossing_times, float(source_time[-1])])

        privileges: list[GatePrivilege] = []
        route_points = np.vstack((self.start, self.waypoints, self.goal))
        for route_index, (window_index, crossing_time) in enumerate(
            zip(self.track.order, self.crossing_times)
        ):
            window = self.track.windows[window_index]
            local, plane = window.world_to_local(self.waypoints[route_index], crossing_time)
            if abs(plane) > 1.0e-5:
                raise ValueError(f"planned waypoint {route_index} is off its gate plane")
            velocity = np.array(
                [np.interp(crossing_time, source_time, source_velocity[:, axis]) for axis in range(3)]
            )
            _, rotation, _, _ = window.state_at(crossing_time)
            route_direction = route_points[route_index + 2] - route_points[route_index]
            direction = float(np.sign(route_direction @ rotation[:, 2]))
            if direction == 0.0:
                direction = 1.0
            privileges.append(
                GatePrivilege(
                    local_point=np.asarray(local, dtype=float),
                    crossing_velocity=velocity,
                    crossing_direction=direction,
                    nominal_segment_time=float(segment_times[route_index]),
                )
            )
        self.gates = tuple(privileges)
        self.final_segment_time = float(segment_times[-1])

    def crossing_point(self, route_index: int, time: float) -> np.ndarray:
        window_index = self.track.order[route_index]
        window = self.track.windows[window_index]
        center, rotation, _, _ = window.state_at(time)
        local = self.gates[route_index].local_point
        return center + rotation[:, :2] @ local

    def target(
        self,
        route_index: int,
        time: float,
        overshoot: float = 1.8,
        *,
        entry_side: bool = False,
    ) -> tuple[np.ndarray, np.ndarray]:
        if route_index >= len(self.gates):
            return self.goal.copy(), np.zeros(3)
        gate = self.gates[route_index]
        window = self.track.windows[self.track.order[route_index]]
        center, rotation, center_velocity, rotation_derivative = window.state_at(time)
        point = center + rotation[:, :2] @ gate.local_point
        normal = rotation[:, 2]
        side = -gate.crossing_direction if entry_side else gate.crossing_direction
        target = point + side * overshoot * normal
        target_velocity = (
            gate.crossing_velocity
            + center_velocity
            + rotation_derivative[:, :2] @ gate.local_point
        )
        return target, target_velocity
