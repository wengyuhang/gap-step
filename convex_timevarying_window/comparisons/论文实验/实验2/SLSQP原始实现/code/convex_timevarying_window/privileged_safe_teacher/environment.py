"""Direct CTBR closed-loop environment for the privileged teacher."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from closed_loop_deformable_window.fapp_ppo.config import QuadrotorConfig
from closed_loop_deformable_window.fapp_ppo.dynamics import (
    QuadrotorDynamics,
    QuadrotorState,
)
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import (
    X500_FRAME_RADIUS,
    X500_INERTIA,
    X500_MASS,
    X500_MAX_RATE_Z,
    X500_MAX_ROTOR_THRUST,
    X500_MOMENT_CONSTANT,
    X500_ROTOR_ARM,
)

from .privilege import PlannerPrivilege


def _vee(matrix: np.ndarray) -> np.ndarray:
    return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]], dtype=float)


def _normalize(vector: np.ndarray) -> np.ndarray:
    return vector / max(float(np.linalg.norm(vector)), 1.0e-9)


@dataclass(frozen=True)
class StepInfo:
    collision: bool
    crossed: bool
    finished: bool
    rotor_saturated: bool
    minimum_frame_margin: float


class PrivilegedTeacherEnv:
    dt = 0.02
    maximum_time = 110.0
    observation_dim = 34
    action_dim = 4

    def __init__(self, privilege: PlannerPrivilege | None = None, seed: int = 0):
        self.privilege = privilege or PlannerPrivilege()
        self.rng = np.random.default_rng(seed)
        self.quad_config = QuadrotorConfig(
            mass=X500_MASS,
            gravity=9.8066,
            inertia=tuple(float(value) for value in X500_INERTIA),
            arm_length=X500_ROTOR_ARM,
            yaw_moment_coefficient=X500_MOMENT_CONSTANT,
            rotor_thrust_min=0.0,
            rotor_thrust_max=X500_MAX_ROTOR_THRUST,
            max_body_rate=X500_MAX_RATE_Z,
            body_rate_time_constant=0.035,
            linear_drag=0.08,
        )
        self.dynamics = QuadrotorDynamics(self.quad_config)
        self.state: QuadrotorState
        self.time = 0.0
        self.route_index = 0
        self._pending_crossing = False
        self._approach_entry = False
        self.minimum_frame_margin = float("inf")
        self.reset()

    def reset(self, perturbation_scale: float = 0.0) -> np.ndarray:
        position = self.privilege.start + self.rng.normal(0.0, 0.18 * perturbation_scale, 3)
        velocity = self.rng.normal(0.0, 0.25 * perturbation_scale, 3)
        self.state = self.dynamics.hover_state(position)
        self.state.velocity = velocity
        self.time = 0.0
        self.route_index = 0
        self._pending_crossing = False
        self._approach_entry = False
        self.minimum_frame_margin = float("inf")
        return self.observe()

    def _gate_features(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float]:
        if self.route_index >= len(self.privilege.gates):
            relative = self.privilege.goal - self.state.position
            return relative, np.zeros(3), np.array([0.0, 0.0, 1.0]), 0.0, 10.0
        target, target_velocity = self.privilege.target(
            self.route_index, self.time, entry_side=self._approach_entry
        )
        window = self.privilege.track.windows[self.privilege.track.order[self.route_index]]
        local, plane = window.world_to_local(self.state.position, self.time)
        _, rotation, _, _ = window.state_at(self.time)
        clearance = float(window.aperture.boundary_distance(local)) - X500_FRAME_RADIUS
        return target - self.state.position, target_velocity, rotation[:, 2], plane, clearance

    def observe(self) -> np.ndarray:
        relative, target_velocity, normal, plane, clearance = self._gate_features()
        next_relative = np.zeros(3)
        if self.route_index + 1 < len(self.privilege.gates):
            next_point = self.privilege.crossing_point(self.route_index + 1, self.time)
            next_relative = next_point - self.state.position
        nominal_time = (
            self.privilege.gates[self.route_index].nominal_segment_time
            if self.route_index < len(self.privilege.gates)
            else self.privilege.final_segment_time
        )
        observation = np.concatenate(
            (
                relative / 25.0,
                self.state.velocity / 12.0,
                self.state.rotation.reshape(-1),
                self.state.body_rate / self.quad_config.max_body_rate,
                target_velocity / 12.0,
                self.state.rotation.T @ normal,
                next_relative / 35.0,
                [
                    plane / 20.0,
                    clearance / 2.0,
                    self.route_index / 7.0,
                    nominal_time / 6.0,
                    self.time / 80.0,
                    float(self._approach_entry),
                    float(self._pending_crossing),
                ],
            )
        ).astype(np.float32)
        if observation.shape != (self.observation_dim,):
            raise RuntimeError(f"observation has shape {observation.shape}")
        return observation

    def expert_action(self) -> np.ndarray:
        relative, target_velocity, normal, _, _ = self._gate_features()
        target_speed = float(np.linalg.norm(target_velocity))
        if target_speed > 3.5:
            target_velocity = target_velocity * (3.5 / target_speed)
        distance = float(np.linalg.norm(relative))
        desired_speed = max(float(np.linalg.norm(target_velocity)), 3.0)
        tau = float(np.clip(distance / desired_speed, 0.55, 3.5))
        desired_acceleration = 6.0 * relative / tau**2 - (
            4.0 * self.state.velocity + 2.0 * target_velocity
        ) / tau
        # The sparse planner privilege specifies the desired point in the gate
        # plane.  As the vehicle approaches that plane, a geometric recovery
        # term suppresses accumulated lateral error without consulting a
        # time-indexed reference trajectory.
        if self.route_index < len(self.privilege.gates):
            window = self.privilege.track.windows[self.privilege.track.order[self.route_index]]
            local, plane = window.world_to_local(self.state.position, self.time)
            _, rotation, center_velocity, _ = window.state_at(self.time)
            local_error = self.privilege.gates[self.route_index].local_point - local
            relative_velocity = self.state.velocity - center_velocity
            lateral_velocity = rotation[:, :2].T @ relative_velocity
            proximity = float(np.clip(1.0 - abs(plane) / 8.0, 0.0, 1.0))
            desired_acceleration += proximity * rotation[:, :2] @ (
                5.0 * local_error - 1.8 * lateral_velocity
            )
            if abs(plane) < 4.5:
                # Gate-local recovery field: center in the aperture while
                # maintaining a modest through-plane velocity.  This is
                # deliberately independent of the plan's time parameter.
                local_velocity_command = np.clip(1.8 * local_error, -2.0, 2.0)
                safe_velocity = (
                    center_velocity
                    + rotation[:, :2] @ local_velocity_command
                    + 1.35
                    * (-1.0 if self._approach_entry else 1.0)
                    * self.privilege.gates[self.route_index].crossing_direction
                    * rotation[:, 2]
                )
                desired_acceleration = 2.8 * (safe_velocity - self.state.velocity)
        speed = float(np.linalg.norm(self.state.velocity))
        if speed > 4.5:
            desired_acceleration -= 2.0 * (speed - 4.5) * self.state.velocity / speed
        norm = float(np.linalg.norm(desired_acceleration))
        if norm > 6.0:
            desired_acceleration *= 6.0 / norm
        desired_force = desired_acceleration + np.array([0.0, 0.0, self.quad_config.gravity])
        desired_z = _normalize(desired_force)
        horizontal = relative.copy()
        horizontal[2] = 0.0
        desired_x_hint = _normalize(horizontal) if np.linalg.norm(horizontal) > 1.0e-5 else np.array([1.0, 0.0, 0.0])
        desired_y = np.cross(desired_z, desired_x_hint)
        if np.linalg.norm(desired_y) < 1.0e-5:
            desired_y = np.array([0.0, 1.0, 0.0])
        desired_y = _normalize(desired_y)
        desired_x = _normalize(np.cross(desired_y, desired_z))
        desired_rotation = np.column_stack((desired_x, desired_y, desired_z))
        error_matrix = 0.5 * (
            desired_rotation.T @ self.state.rotation
            - self.state.rotation.T @ desired_rotation
        )
        desired_rate = -4.5 * _vee(error_matrix)
        desired_rate = np.clip(
            desired_rate, -self.quad_config.max_body_rate, self.quad_config.max_body_rate
        )
        specific_thrust = float(desired_force @ self.state.rotation[:, 2])
        return np.r_[
            self.dynamics.collective_action(specific_thrust),
            desired_rate / self.quad_config.max_body_rate,
        ].astype(np.float32)

    def _minimum_margin(self, state: QuadrotorState, time: float) -> float:
        return min(
            self._window_margin(state, time, index)
            for index in range(len(self.privilege.track.windows))
        )

    def _window_margin(self, state: QuadrotorState, time: float, index: int) -> float:
        window = self.privilege.track.windows[index]
        local, plane = window.world_to_local(state.position, time)
        return float(
            np.hypot(window.aperture.boundary_distance(local), plane)
            - X500_FRAME_RADIUS
        )

    def _nearby_windows(self, horizon: float) -> tuple[int, ...]:
        travel_bound = (
            float(np.linalg.norm(self.state.velocity)) * horizon
            + 0.5 * 30.0 * horizon**2
            + 0.5
        )
        nearby = []
        for index, window in enumerate(self.privilege.track.windows):
            center, _, _, _ = window.state_at(self.time)
            points = window.aperture.boundary_points(circle_samples=24)
            frame_radius = float(np.linalg.norm(points, axis=1).max()) + X500_FRAME_RADIUS
            if np.linalg.norm(self.state.position - center) <= frame_radius + travel_bound:
                nearby.append(index)
        return tuple(nearby)

    def rollout_safety(self, action: np.ndarray, horizon: float = 0.30) -> tuple[float, float]:
        state = self.state.copy()
        time = self.time
        minimum_margin = float("inf")
        maximum_saturation = 0.0
        nearby = self._nearby_windows(horizon)
        for _ in range(max(1, int(round(horizon / self.dt)))):
            state, diagnostics = self.dynamics.step(state, action, self.dt)
            time += self.dt
            if nearby:
                minimum_margin = min(
                    minimum_margin,
                    *(self._window_margin(state, time, index) for index in nearby),
                )
            maximum_saturation = max(maximum_saturation, diagnostics.saturation_fraction)
        return minimum_margin, maximum_saturation

    def action_is_recoverable(self, action: np.ndarray, horizon: float = 0.30) -> bool:
        margin, saturation = self.rollout_safety(action, horizon)
        return margin >= 0.04 and saturation <= 0.75

    def shield(self, policy_action: np.ndarray) -> tuple[np.ndarray, bool]:
        from .safety_filter import PredictiveSafetyFilter

        decision = PredictiveSafetyFilter().filter(self, policy_action)
        return decision.action, decision.intervened

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, StepInfo]:
        previous_state = self.state.copy()
        previous_time = self.time
        self.state, diagnostics = self.dynamics.step(self.state, action, self.dt)
        self.time += self.dt
        info = self._assess_transition(
            previous_state,
            previous_time,
            rotor_saturated=diagnostics.saturation_fraction > 0.0,
        )
        done = info.collision or info.finished or self.time >= self.maximum_time
        reward = -self.dt + (4.0 if info.crossed else 0.0) + (20.0 if info.finished else 0.0) - (20.0 if info.collision else 0.0)
        return self.observe(), reward, done, info

    def accept_external_state(self, state: QuadrotorState, time: float) -> StepInfo:
        """Advance route state from a measured Gazebo/PX4 state."""

        previous_state = self.state.copy()
        previous_time = self.time
        self.state = state.copy()
        self.time = float(time)
        return self._assess_transition(previous_state, previous_time, rotor_saturated=False)

    def _assess_transition(
        self,
        previous_state: QuadrotorState,
        previous_time: float,
        *,
        rotor_saturated: bool,
    ) -> StepInfo:
        margin = self._minimum_margin(self.state, self.time)
        self.minimum_frame_margin = min(self.minimum_frame_margin, margin)
        collision = margin < 0.0
        crossed = False
        if self.route_index < len(self.privilege.gates):
            window = self.privilege.track.windows[self.privilege.track.order[self.route_index]]
            _, previous_plane = window.world_to_local(previous_state.position, previous_time)
            local, current_plane = window.world_to_local(self.state.position, self.time)
            gate = self.privilege.gates[self.route_index]
            if (
                self._approach_entry
                and gate.crossing_direction * current_plane <= -(X500_FRAME_RADIUS + 0.90)
                and np.linalg.norm(local - gate.local_point) < 0.45
            ):
                self._approach_entry = False
            direction_ok = (
                not self._approach_entry
                and gate.crossing_direction * previous_plane < 0.0
                <= gate.crossing_direction * current_plane
            )
            safe_opening = window.aperture.contains(local) and float(window.aperture.boundary_distance(local)) >= X500_FRAME_RADIUS
            if direction_ok and safe_opening:
                self._pending_crossing = True
            if (
                self._pending_crossing
                and gate.crossing_direction * current_plane >= X500_FRAME_RADIUS + 0.90
            ):
                self.route_index += 1
                self._pending_crossing = False
                crossed = True
                if self.route_index < len(self.privilege.gates):
                    next_window = self.privilege.track.windows[
                        self.privilege.track.order[self.route_index]
                    ]
                    _, next_plane = next_window.world_to_local(self.state.position, self.time)
                    next_gate = self.privilege.gates[self.route_index]
                    self._approach_entry = (
                        next_gate.crossing_direction * next_plane > -(X500_FRAME_RADIUS + 0.90)
                    )
        final_error = float(np.linalg.norm(self.state.position - self.privilege.goal))
        final_speed = float(np.linalg.norm(self.state.velocity))
        finished = self.route_index == len(self.privilege.gates) and final_error < 0.45 and final_speed < 0.65
        return StepInfo(
            collision=collision,
            crossed=crossed,
            finished=finished,
            rotor_saturated=rotor_saturated,
            minimum_frame_margin=margin,
        )
