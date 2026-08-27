"""AVS-PPO transfer environment for the hardest six-window comparison track.

The task keeps the comparison's exact moving windows and oriented cuboid.  A
categorical policy controls the rate at which a safe reference curve is
tracked; the viability shield admits an action only when that action followed
by nominal recovery stays collision-free in its finite prediction horizon.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from scipy.optimize import minimize_scalar
import yaml

from nonconvex_timevarying_window.sc_dynatogt.collision import (
    point_to_oriented_cuboid_distance_squared,
)
from nonconvex_timevarying_window.sc_dynatogt.dynamics import flatness_map
from nonconvex_timevarying_window.sip_dynatogt.io import load_run

from .geometry import point_in_polygon
from .config import PPOConfig


DEFAULT_RUN = Path(
    "nonconvex_timevarying_window/comparisons/sc_sip_fast_closed_loop/results/"
    "wide_scrambled_certified_final/sip_dynatogt/run"
)

# (reference phase rate, acceleration along local normal 1, normal 2).
# Action zero is the nominal recovery action.  Other actions briefly advance or
# retard the path phase, while a phase servo removes accumulated timing error.
HARD_ACTION_LIBRARY: tuple[tuple[float, float, float], ...] = (
    (1.00, 0.0, 0.0),
    (0.80, 0.0, 0.0),
    (0.90, 0.0, 0.0),
    (1.06, 0.0, 0.0),
    (1.12, 0.0, 0.0),
    (1.18, 0.0, 0.0),
    (1.00, 1.4, 0.0),
    (1.00, -1.4, 0.0),
    (1.00, 0.0, 1.4),
    (1.00, 0.0, -1.4),
    (1.06, 1.0, 0.0),
    (1.06, -1.0, 0.0),
    (1.06, 0.0, 1.0),
)


@dataclass
class HardestTrackConfig:
    run_directory: str = str(DEFAULT_RUN)
    dt: float = 0.05
    max_steps: int = 390
    max_acceleration: float = 32.0
    shield_horizon_steps: int = 1
    shield_substeps: int = 10
    boundary_samples: int = 101
    numerical_guard: float = 0.001
    plane_event_tolerance: float = 0.018
    goal_tolerance: float = 0.55
    reference_phase_guard: float = 0.02
    tracking_tube_radius: float = 0.03
    velocity_tube_radius: float = 0.03
    frame_recovery_radius: float = 9.0


@dataclass
class HardestExperimentConfig:
    environment: HardestTrackConfig = field(default_factory=HardestTrackConfig)
    ppo: PPOConfig = field(default_factory=PPOConfig)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def load_hardest_config(path: str | Path) -> HardestExperimentConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    return HardestExperimentConfig(
        environment=HardestTrackConfig(**raw.get("environment", {})),
        ppo=PPOConfig(**raw.get("ppo", {})),
    )


@dataclass
class HardestState:
    position: np.ndarray
    velocity: np.ndarray
    time: float
    phase: float
    gate_index: int
    steps: int

    def copy(self) -> "HardestState":
        return HardestState(
            self.position.copy(), self.velocity.copy(), self.time,
            self.phase, self.gate_index, self.steps,
        )


def _cuboid_vertices(center: np.ndarray, rotation: np.ndarray, half_extents) -> np.ndarray:
    signs = np.asarray([
        [-1, -1, -1], [-1, -1, 1], [-1, 1, -1], [-1, 1, 1],
        [1, -1, -1], [1, -1, 1], [1, 1, -1], [1, 1, 1],
    ], dtype=float)
    return center + (signs * np.asarray(half_extents)) @ rotation.T


_CUBOID_EDGES = (
    (0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
    (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7),
)


def _plane_section(vertices: np.ndarray, plane_center: np.ndarray, normal: np.ndarray) -> np.ndarray:
    signed = (vertices - plane_center) @ normal
    result: list[np.ndarray] = []
    for left, right in _CUBOID_EDGES:
        a, b = vertices[left], vertices[right]
        da, db = float(signed[left]), float(signed[right])
        if abs(da) <= 1.0e-10:
            result.append(a)
        if da * db < 0.0:
            result.append(a + da / (da - db) * (b - a))
    unique: list[np.ndarray] = []
    for point in result:
        if not any(np.linalg.norm(point - old) <= 1.0e-9 for old in unique):
            unique.append(point)
    return np.asarray(unique, dtype=float) if unique else np.empty((0, 3))


class HardestComparisonAVSEnvironment:
    """Fixed hardest comparison scenario with the original oriented cuboid."""

    action_dim = len(HARD_ACTION_LIBRARY)
    recovery_action = 0

    def __init__(self, config: HardestTrackConfig | None = None, *, seed: int = 0):
        self.config = config or HardestTrackConfig()
        self.seed = int(seed)
        self.problem, self.sip_config, self.reference, self.certificate = load_run(
            self.config.run_directory
        )
        self.body = self.sip_config.body
        self.required_clearance = float(self.sip_config.clearance)
        self.traversal_times = np.asarray(self.certificate["traversal_times"], dtype=float)
        self._cumulative = np.concatenate(([0.0], np.cumsum(self.reference.durations)))
        parameters = np.linspace(0.0, 1.0, self.config.boundary_samples)
        self._local_segments = tuple(
            tuple(
                np.asarray([segment.evaluate(float(u)) for u in parameters], dtype=float)
                for segment in window.boundary
            )
            for window in self.problem.windows
        )
        self._local_polygons = tuple(self._sample_local_polygon(window, 80) for window in self.problem.windows)
        self.state: HardestState
        self.safety_violations = 0
        self.shield_interventions = 0
        self.crossing_records: list[dict[str, Any]] = []
        self.minimum_clearance = float("inf")
        self.history: list[dict[str, Any]] = []
        self.reset(seed=seed)

    @property
    def observation_dim(self) -> int:
        return int(self.observe().shape[0])

    @property
    def total_reference_time(self) -> float:
        return float(self.reference.total_time)

    def _sample_local_polygon(self, window, samples: int) -> np.ndarray:
        pieces = []
        for index, segment in enumerate(window.boundary):
            values = np.linspace(0.0, 1.0, samples, endpoint=index == len(window.boundary) - 1)
            pieces.append(np.asarray([segment.evaluate(float(value)) for value in values]))
        return np.vstack(pieces)

    def _reference_values(self, phase: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        query = float(np.clip(phase, 0.0, self.reference.total_time))
        segment = min(
            int(np.searchsorted(self._cumulative[1:], query, side="right")),
            len(self.reference.durations) - 1,
        )
        local = query - self._cumulative[segment]
        local = float(np.clip(local, 0.0, self.reference.durations[segment]))
        return tuple(
            np.asarray(self.reference.evaluate_segment(segment, local, derivative), dtype=float)
            for derivative in (0, 1, 2)
        )

    def reset(self, *, seed: int | None = None) -> tuple[np.ndarray, dict]:
        if seed is not None:
            self.seed = int(seed)
        position, velocity, _ = self._reference_values(0.0)
        self.state = HardestState(position.copy(), velocity.copy(), 0.0, 0.0, 0, 0)
        self.safety_violations = 0
        self.shield_interventions = 0
        self.crossing_records = []
        self.minimum_clearance = float("inf")
        self.history = []
        return self.observe(), {"scenario": self.problem.name, "seed": self.seed}

    def _frame(self, velocity: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        speed = float(np.linalg.norm(velocity))
        tangent = velocity / speed if speed > 1.0e-8 else np.array([1.0, 0.0, 0.0])
        helper = np.array([0.0, 0.0, 1.0]) if abs(tangent[2]) < 0.88 else np.array([0.0, 1.0, 0.0])
        normal1 = np.cross(tangent, helper)
        normal1 /= max(np.linalg.norm(normal1), 1.0e-12)
        normal2 = np.cross(tangent, normal1)
        return tangent, normal1, normal2

    def command(self, action: int, state: HardestState | None = None) -> tuple[np.ndarray, float]:
        current = self.state if state is None else state
        rate, residual_1, residual_2 = HARD_ACTION_LIBRARY[int(action)]
        phase_error = current.phase - current.time
        next_phase = min(
            self.reference.total_time,
            current.phase + rate * self.config.dt - 10.0 * phase_error * self.config.dt,
        )
        desired_position, _, _ = self._reference_values(next_phase)
        acceleration = 2.0 * (
            desired_position - current.position - current.velocity * self.config.dt
        ) / (self.config.dt * self.config.dt)
        _, normal1, normal2 = self._frame(self._reference_values(current.phase)[1])
        acceleration = acceleration + residual_1 * normal1 + residual_2 * normal2
        norm = float(np.linalg.norm(acceleration))
        if norm > self.config.max_acceleration:
            acceleration *= self.config.max_acceleration / norm
        return acceleration, float(next_phase)

    def _rotation(self, acceleration: np.ndarray) -> np.ndarray:
        return np.asarray(
            flatness_map(
                acceleration, np.zeros(3), np.zeros(3), parameters=self.sip_config.quadrotor
            ).rotation,
            dtype=float,
        )

    def _boundary_world(self, window_index: int, time: float) -> np.ndarray:
        window = self.problem.windows[window_index]
        center, rotation, scale = window.state_at(time)
        local = np.vstack(self._local_segments[window_index])
        return center + (rotation @ np.column_stack((scale * local, np.zeros(len(local)))).T).T

    def _clearance_at(self, position: np.ndarray, rotation: np.ndarray, time: float) -> float:
        best = float("inf")
        for window_index, window in enumerate(self.problem.windows):
            center, _, scale = window.state_at(time)
            local = np.vstack(self._local_segments[window_index])
            radius = float(np.max(np.linalg.norm(local, axis=1)) * scale + 1.0)
            if np.linalg.norm(position - center) > radius + 1.0:
                continue
            boundary = self._boundary_world(window_index, time)
            distance = np.sqrt(
                point_to_oriented_cuboid_distance_squared(
                    boundary, position, rotation, self.body
                )
            )
            best = min(best, float(np.min(distance)))
        return best

    def _plane_value(self, window_index: int, position: np.ndarray, time: float) -> float:
        center, basis, _ = self.problem.windows[window_index].state_at(time)
        return float(basis[:, 2] @ (position - center))

    def _crossing_event(
        self,
        state: HardestState,
        acceleration: np.ndarray,
        rotation: np.ndarray,
    ) -> dict[str, Any] | None:
        if state.gate_index >= len(self.problem.order):
            return None
        window_index = int(self.problem.order[state.gate_index])
        dt = self.config.dt

        def position_at(tau: float) -> np.ndarray:
            return state.position + state.velocity * tau + 0.5 * acceleration * tau * tau

        def squared_plane(tau: float) -> float:
            value = self._plane_value(window_index, position_at(float(tau)), state.time + float(tau))
            return value * value

        nodes = np.linspace(0.0, dt, 7)
        values = np.asarray([
            self._plane_value(window_index, position_at(float(tau)), state.time + float(tau))
            for tau in nodes
        ])
        candidate_tau: float | None = None
        for left, right, a, b in zip(nodes[:-1], nodes[1:], values[:-1], values[1:]):
            if a <= 0.0 <= b:
                result = minimize_scalar(squared_plane, bounds=(float(left), float(right)), method="bounded")
                candidate_tau = float(result.x)
                break
        if candidate_tau is None:
            result = minimize_scalar(squared_plane, bounds=(0.0, dt), method="bounded")
            if float(np.sqrt(result.fun)) <= self.config.plane_event_tolerance:
                candidate_tau = float(result.x)
        if candidate_tau is None:
            return None

        event_time = state.time + candidate_tau
        event_position = position_at(candidate_tau)
        window = self.problem.windows[window_index]
        center, basis, scale = window.state_at(event_time)
        section = _plane_section(
            _cuboid_vertices(event_position, rotation, self.body.half_extents),
            center,
            basis[:, 2],
        )
        inside = False
        projected = np.empty((0, 2))
        if len(section) >= 3:
            projected = ((section - center) @ basis[:, :2]) / scale
            polygon = self._local_polygons[window_index]
            inside = all(point_in_polygon(point, polygon) for point in projected)
        boundary = self._boundary_world(window_index, event_time)
        margin = float(np.sqrt(
            point_to_oriented_cuboid_distance_squared(
                boundary, event_position, rotation, self.body
            )
        ).min())
        return {
            "route_index": state.gate_index,
            "window_index": window_index,
            "window_name": window.name,
            "time": float(event_time),
            "position": event_position,
            "rotation": rotation,
            "section": section,
            "projected_section": projected,
            "inside": bool(inside),
            "clearance": margin,
        }

    def _integrate(self, state: HardestState, action: int) -> tuple[HardestState, bool, float, dict[str, Any] | None, np.ndarray]:
        acceleration, next_phase = self.command(action, state)
        rotation = self._rotation(acceleration)
        safe = True
        step_clearance = float("inf")
        for tau in np.linspace(0.0, self.config.dt, self.config.shield_substeps + 1)[1:]:
            position = state.position + state.velocity * tau + 0.5 * acceleration * tau * tau
            clearance = self._clearance_at(position, rotation, state.time + float(tau))
            step_clearance = min(step_clearance, clearance)
            if clearance < self.required_clearance + self.config.numerical_guard:
                safe = False
                break
        event = self._crossing_event(state, acceleration, rotation)
        if event is not None and (
            not event["inside"]
            or event["clearance"] < self.required_clearance + self.config.numerical_guard
        ):
            # The benchmark models a moving frame, not an infinite solid wall.
            # Crossing its supporting plane outside the opening is not a gate
            # traversal; collision with the actual frame is handled above.
            event = None
        velocity = state.velocity + acceleration * self.config.dt
        position = state.position + state.velocity * self.config.dt + 0.5 * acceleration * self.config.dt**2
        gate_index = state.gate_index + int(event is not None and safe)
        next_state = HardestState(
            position, velocity, state.time + self.config.dt, next_phase,
            gate_index, state.steps + 1,
        )
        return next_state, safe, step_clearance, event, acceleration

    def action_mask(self) -> np.ndarray:
        mask = np.zeros(self.action_dim, dtype=bool)
        near_frame = any(
            np.linalg.norm(self.state.position - window.state_at(self.state.time)[0])
            <= self.config.frame_recovery_radius
            for window in self.problem.windows
        )
        candidate_actions = (self.recovery_action,) if near_frame else range(self.action_dim)
        for action in candidate_actions:
            simulated = self.state.copy()
            safe = True
            for horizon in range(self.config.shield_horizon_steps + 1):
                rollout_action = action if horizon == 0 else self.recovery_action
                simulated, transition_safe, _, _, _ = self._integrate(simulated, rollout_action)
                if (
                    simulated.gate_index == len(self.problem.order)
                    and simulated.phase >= self.reference.total_time - 0.05
                ):
                    break
                reference_position = self._reference_values(simulated.phase)[0]
                reference_velocity = self._reference_values(simulated.phase)[1]
                recoverable = (
                    abs(simulated.phase - simulated.time) <= self.config.reference_phase_guard
                    and np.linalg.norm(simulated.position - reference_position)
                    <= self.config.tracking_tube_radius
                    and np.linalg.norm(simulated.velocity - reference_velocity)
                    <= self.config.velocity_tube_radius
                )
                if not transition_safe or not recoverable:
                    safe = False
                    break
            mask[action] = safe
        if not mask.any():
            # The nominal action is the certified-reference recovery direction.
            # If the conservative tube test is empty but its immediate full-body
            # transition is still safe, retain that single recovery action.
            _, recovery_safe, _, _, _ = self._integrate(self.state, self.recovery_action)
            if recovery_safe:
                mask[self.recovery_action] = True
            else:
                raise RuntimeError("hardest-track viability shield found no recoverable action")
        return mask

    def observe(self) -> np.ndarray:
        state = self.state
        ref_position, ref_velocity, _ = self._reference_values(state.phase)
        features: list[float] = [
            *(state.position / 20.0),
            *(state.velocity / 20.0),
            state.time / (self.config.max_steps * self.config.dt),
            state.phase / self.reference.total_time,
            *((ref_position - state.position) / 2.0),
            *((ref_velocity - state.velocity) / 10.0),
        ]
        one_hot = np.zeros(len(self.problem.order) + 1)
        one_hot[min(state.gate_index, len(self.problem.order))] = 1.0
        features.extend(one_hot.tolist())
        for route_index, window_index in enumerate(self.problem.order):
            center, basis, scale = self.problem.windows[window_index].state_at(state.time)
            features.extend([
                *((center - state.position) / 25.0),
                *basis[:, 2],
                scale,
                self._plane_value(window_index, state.position, state.time) / 20.0,
                float(route_index == state.gate_index),
            ])
        return np.asarray(features, dtype=np.float32)

    def step(self, action: int, *, mask: np.ndarray | None = None):
        safe_mask = self.action_mask() if mask is None else np.asarray(mask, dtype=bool)
        requested = int(action)
        if not safe_mask[requested]:
            requested = self.recovery_action
            self.shield_interventions += 1
        old = self.state.copy()
        next_state, safe, clearance, event, acceleration = self._integrate(self.state, requested)
        if not safe:
            self.safety_violations += 1
        if event is not None and safe:
            self.crossing_records.append(event)
        self.minimum_clearance = min(self.minimum_clearance, clearance)
        self.state = next_state
        self.history.append({
            "time": old.time,
            "phase": old.phase,
            "position": old.position.copy(),
            "velocity": old.velocity.copy(),
            "acceleration": acceleration.copy(),
            "rotation": self._rotation(acceleration),
            "action": requested,
            "mask_fraction": float(np.mean(safe_mask)),
            "clearance": clearance,
        })
        phase_progress = self.state.phase - old.phase
        tracking_error = float(np.linalg.norm(self.state.position - self._reference_values(self.state.phase)[0]))
        gates_crossed = self.state.gate_index - old.gate_index
        reward = 0.55 * phase_progress - 0.012 - 0.035 * tracking_error + 3.0 * gates_crossed
        goal_position = self._reference_values(self.reference.total_time)[0]
        goal_distance = float(np.linalg.norm(self.state.position - goal_position))
        success = bool(
            self.state.gate_index == len(self.problem.order)
            and self.state.phase >= self.reference.total_time - 0.05
            and goal_distance <= self.config.goal_tolerance
        )
        if success:
            reward += 12.0 + 0.025 * (self.config.max_steps - self.state.steps)
        if not safe:
            reward -= 50.0
        terminated = bool(success or not safe)
        truncated = bool(self.state.steps >= self.config.max_steps and not terminated)
        info = {
            "success": success,
            "safe": bool(self.safety_violations == 0),
            "safety_violations": self.safety_violations,
            "shield_interventions": self.shield_interventions,
            "elapsed_time": self.state.time,
            "reference_phase": self.state.phase,
            "gates_crossed": self.state.gate_index,
            "minimum_whole_body_clearance": self.minimum_clearance,
            "minimum_crossing_clearance": min(
                (record["clearance"] for record in self.crossing_records),
                default=float("inf"),
            ),
            "required_clearance": self.required_clearance,
            "action_mask_fraction": float(np.mean(safe_mask)),
            "applied_action": requested,
            "goal_distance": goal_distance,
        }
        return self.observe(), float(reward), terminated, truncated, info


__all__ = [
    "DEFAULT_RUN", "HARD_ACTION_LIBRARY", "HardestComparisonAVSEnvironment",
    "HardestExperimentConfig", "HardestState", "HardestTrackConfig",
    "load_hardest_config",
]
