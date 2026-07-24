"""Closed-loop deformable-window environment and privileged observations."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from shapely.geometry import Point

from .config import EnvironmentConfig, QuadrotorConfig
from .dynamics import (
    DynamicsDiagnostics,
    QuadrotorDynamics,
    QuadrotorState,
    attitude_error_angle,
)
from .geometry import WindowState, normalize, signed_margin
from .scenario import ClosedLoopScenario, build_scenario


Array = np.ndarray


@dataclass(frozen=True)
class Observation:
    actor: Array
    critic: Array


def _vee(matrix: Array) -> Array:
    return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]], dtype=float)


class ClosedLoopWindowEnv:
    """Simulation-only MDP for one-pass ordered gates and full-state return."""

    action_dim = 4

    def __init__(
        self,
        environment: EnvironmentConfig,
        quadrotor: QuadrotorConfig,
        *,
        stage: str = "full",
        seed: int = 0,
    ):
        self.config = environment
        self.quadrotor_config = quadrotor
        self.stage = stage
        self.base_seed = int(seed)
        self._episode_index = 0
        self.dynamics = QuadrotorDynamics(quadrotor)
        self.scenario: ClosedLoopScenario
        self.state: QuadrotorState
        self.time = 0.0
        self.step_count = 0
        self.progress_index = 0
        self.previous_action = np.zeros(4, dtype=float)
        self.crossing_records: list[dict[str, float | int | str]] = []
        self.missed_opportunities = 0
        self._previous_potential = 0.0
        self._last_diagnostics: DynamicsDiagnostics | None = None
        self.energy_proxy = 0.0
        self.saturated_steps = 0
        self.closed_target_time = 0.0
        self.reset(seed=seed)

    @property
    def actor_obs_dim(self) -> int:
        return int(self.observe().actor.shape[0])

    @property
    def critic_obs_dim(self) -> int:
        return int(self.observe().critic.shape[0])

    def reset(self, *, seed: int | None = None) -> tuple[Observation, dict]:
        scenario_seed = self.base_seed + self._episode_index if seed is None else int(seed)
        self._episode_index += 1
        self.scenario = build_scenario(
            seed=scenario_seed,
            stage=self.stage,
            environment=self.config,
            quadrotor=self.quadrotor_config,
        )
        self.state = self.scenario.initial_state.copy()
        self.time = 0.0
        self.step_count = 0
        self.progress_index = 0
        self.previous_action = np.zeros(4, dtype=float)
        self.crossing_records = []
        self.missed_opportunities = 0
        self._last_diagnostics = None
        self.energy_proxy = 0.0
        self.saturated_steps = 0
        self.closed_target_time = 0.0
        self._previous_potential = self._potential()
        observation = self.observe()
        return observation, {
            "scenario": self.scenario.name,
            "scenario_seed": scenario_seed,
            "stage": self.stage,
        }

    def _target(
        self,
        horizon: float = 0.0,
        route_offset: int = 0,
        *,
        schedule_aware: bool = False,
    ) -> tuple[Array, Array]:
        route_index = self.progress_index + route_offset
        if route_index < len(self.scenario.order):
            window_index = self.scenario.order[route_index]
            window = self.scenario.windows[window_index]
            query_time = min(
                self.scenario.horizon, self.time + max(0.0, horizon)
            )
            if (
                schedule_aware
                and route_offset == 0
                and self.config.opportunity_mode != "always_open"
            ):
                selected: tuple[float, float, float] | None = None
                timing_buffer = max(0.08, 0.5 * self.config.opportunity_transition)
                for start, end in window.planned_opportunities:
                    if end - timing_buffer <= self.time:
                        continue
                    representative_time = 0.5 * (start + end)
                    representative = window.state(representative_time)
                    distance = float(
                        np.linalg.norm(
                            representative.safe_anchor_world - self.state.position
                        )
                    )
                    predicted_arrival = (
                        self.time
                        + distance / (0.70 * self.config.cruise_speed)
                        + 0.20
                    )
                    if predicted_arrival <= end - timing_buffer:
                        crossing_time = float(
                            np.clip(
                                predicted_arrival,
                                start + timing_buffer,
                                end - timing_buffer,
                            )
                        )
                        selected = (start, end, crossing_time)
                        break
                if selected is not None:
                    start, _, crossing_time = selected
                    open_state = window.state(crossing_time)
                    distance = float(
                        np.linalg.norm(
                            open_state.safe_anchor_world - self.state.position
                        )
                    )
                    predicted_arrival = (
                        self.time
                        + distance / (0.70 * self.config.cruise_speed)
                        + 0.20
                    )
                    if predicted_arrival < start + timing_buffer:
                        holding_point = (
                            open_state.safe_anchor_world
                            - self.config.opportunity_holding_distance
                            * open_state.normal
                        )
                        return holding_point, window.center_velocity(crossing_time)
                    query_time = crossing_time
                    crossing_target = (
                        open_state.safe_anchor_world
                        + self.config.opportunity_crossing_overshoot
                        * open_state.normal
                    )
                    return crossing_target, window.center_velocity(crossing_time)
                elif window.planned_opportunities:
                    _, last_end = window.planned_opportunities[-1]
                    last_open_state = window.state(last_end)
                    holding_point = (
                        last_open_state.safe_anchor_world
                        - self.config.opportunity_holding_distance
                        * last_open_state.normal
                    )
                    return holding_point, np.zeros(3)
            return window.state(query_time).safe_anchor_world, window.center_velocity(query_time)
        if route_index == len(self.scenario.order):
            return self.scenario.initial_state.position.copy(), np.zeros(3)
        return self.scenario.initial_state.position.copy(), np.zeros(3)

    def _estimated_arrival_horizon(self) -> float:
        target, _ = self._target(0.0)
        distance = float(np.linalg.norm(target - self.state.position))
        return float(np.clip(distance / self.config.cruise_speed, 0.15, max(self.config.preview_horizons)))

    def nominal_action(self) -> Array:
        """Physics prior; PPO learns a bounded residual around this CTBR command."""

        arrival = self._estimated_arrival_horizon()
        target, target_velocity = self._target(
            arrival,
            schedule_aware=self.config.opportunity_aware_nominal,
        )
        position_error = target - self.state.position
        velocity_error = target_velocity - self.state.velocity
        returning = self.progress_index >= len(self.scenario.order)
        kp = 1.45 if not returning else 1.15
        kd = 1.15 if not returning else 1.55
        desired_acceleration = kp * position_error + kd * velocity_error
        acceleration_norm = float(np.linalg.norm(desired_acceleration))
        if acceleration_norm > 7.0:
            desired_acceleration *= 7.0 / acceleration_norm
        desired_force = desired_acceleration + np.array(
            [0.0, 0.0, self.quadrotor_config.gravity]
        )
        desired_z = normalize(desired_force)
        if returning:
            desired_x_hint = self.scenario.initial_state.rotation[:, 0]
        else:
            heading = position_error.copy()
            heading[2] = 0.0
            if np.linalg.norm(heading) < 1.0e-6:
                desired_x_hint = self.scenario.initial_state.rotation[:, 0]
            else:
                desired_x_hint = normalize(heading)
        desired_y = np.cross(desired_z, desired_x_hint)
        if np.linalg.norm(desired_y) < 1.0e-6:
            desired_y = np.array([0.0, 1.0, 0.0])
        desired_y = normalize(desired_y)
        desired_x = normalize(np.cross(desired_y, desired_z))
        desired_rotation = np.column_stack((desired_x, desired_y, desired_z))
        error_matrix = 0.5 * (
            desired_rotation.T @ self.state.rotation
            - self.state.rotation.T @ desired_rotation
        )
        attitude_error = _vee(error_matrix)
        desired_rate = -4.0 * attitude_error
        desired_rate = np.clip(
            desired_rate,
            -self.quadrotor_config.max_body_rate,
            self.quadrotor_config.max_body_rate,
        )
        specific_thrust = float(np.dot(desired_force, self.state.rotation[:, 2]))
        return np.concatenate(
            (
                [self.dynamics.collective_action(specific_thrust)],
                desired_rate / self.quadrotor_config.max_body_rate,
            )
        ).astype(np.float32)

    def _window_preview_features(
        self,
        *,
        route_offset: int,
        horizon: float,
    ) -> Array:
        route_index = self.progress_index + route_offset
        scale = max(self.config.route_radius, 1.0)
        extra_dimension = 4 if self.config.opportunity_features else 0
        if route_index > len(self.scenario.order):
            return np.zeros(21 + extra_dimension, dtype=np.float32)
        if route_index == len(self.scenario.order):
            target = self.scenario.initial_state.position
            relative = (target - self.state.position) / scale
            features = np.concatenate(
                (
                    [1.0],
                    relative,
                    np.zeros(3),
                    np.zeros(3),
                    [0.0, 1.0, 1.0],
                    np.zeros(8),
                )
            )
            if self.config.opportunity_features:
                features = np.concatenate((features, np.zeros(4)))
            return features.astype(np.float32)

        window_index = self.scenario.order[route_index]
        window = self.scenario.windows[window_index]
        query_time = min(self.scenario.horizon, self.time + horizon)
        window_state = window.state(query_time)
        anchor = window_state.safe_anchor_world
        relative = (anchor - self.state.position) / scale
        normal_body = self.state.rotation.T @ window_state.normal
        center_velocity = window.center_velocity(query_time) / max(self.config.cruise_speed, 1.0e-6)
        local_position = window_state.world_to_local(self.state.position)
        plane_distance = float(local_position[2]) / scale
        margin = signed_margin(window_state.safe_polygon, local_position[:2])
        safe_area = float(window_state.safe_polygon.area)
        signature = window.boundary_signature(query_time) / 2.0
        features = np.concatenate(
            (
                [1.0],
                relative,
                normal_body,
                center_velocity,
                [plane_distance, np.clip(margin, -2.0, 2.0) / 2.0, safe_area / 4.0],
                signature,
            )
        )
        if self.config.opportunity_features:
            features = np.concatenate(
                (
                    features,
                    window.opportunity_features(
                        query_time, self.scenario.horizon
                    ),
                )
            )
        return features.astype(np.float32)

    def _actor_observation(self) -> Array:
        total = len(self.scenario.order)
        progress = self.progress_index / max(total, 1)
        remaining = (total - self.progress_index) / max(total, 1)
        returning = float(self.progress_index >= total)
        base = np.concatenate(
            (
                self.state.velocity / 6.0,
                self.state.rotation.reshape(-1),
                self.state.body_rate / self.quadrotor_config.max_body_rate,
                [progress, remaining, returning],
                self.nominal_action(),
            )
        )
        previews = [
            self._window_preview_features(route_offset=offset, horizon=horizon)
            for offset in range(self.config.preview_gate_count)
            for horizon in self.config.preview_horizons
        ]
        return np.concatenate((base, *previews)).astype(np.float32)

    def _critic_privileged_features(self) -> Array:
        scale = max(self.config.route_radius, 1.0)
        route_lookup = {
            window_index: route_index
            for route_index, window_index in enumerate(self.scenario.order)
        }
        features: list[float] = (
            [self.time / self.scenario.horizon]
            if self.config.critic_time_feature
            else []
        )
        if not self.config.critic_privileged_route:
            return np.asarray(features, dtype=np.float32)
        per_window_dimension = 12 + (
            4 if self.config.opportunity_features else 0
        )
        arrival = self._estimated_arrival_horizon()
        for window_index in range(self.config.max_windows):
            if window_index >= len(self.scenario.windows):
                features.extend([0.0] * per_window_dimension)
                continue
            window = self.scenario.windows[window_index]
            route_index = route_lookup[window_index]
            passed = float(route_index < self.progress_index)
            lookahead = max(0, route_index - self.progress_index)
            query_time = min(self.scenario.horizon, self.time + arrival * (lookahead + 1))
            state = window.state(query_time)
            relative = (state.safe_anchor_world - self.state.position) / scale
            velocity = window.center_velocity(query_time) / max(self.config.cruise_speed, 1.0e-6)
            features.extend(
                [
                    1.0,
                    passed,
                    *relative.tolist(),
                    *state.normal.tolist(),
                    *velocity.tolist(),
                    float(state.safe_polygon.area) / 4.0,
                ]
            )
            if self.config.opportunity_features:
                features.extend(
                    window.opportunity_features(
                        query_time, self.scenario.horizon
                    ).tolist()
                )
        return np.asarray(features, dtype=np.float32)

    def observe(self) -> Observation:
        actor = self._actor_observation()
        critic = np.concatenate((actor, self._critic_privileged_features())).astype(np.float32)
        return Observation(actor=actor, critic=critic)

    def _potential(self) -> float:
        target, target_velocity = self._target(
            self._estimated_arrival_horizon(),
            schedule_aware=self.config.opportunity_aware_nominal,
        )
        position_cost = float(np.linalg.norm(target - self.state.position))
        velocity_cost = 0.15 * float(np.linalg.norm(target_velocity - self.state.velocity))
        if self.progress_index >= len(self.scenario.order):
            attitude_cost = 0.25 * attitude_error_angle(
                self.state.rotation, self.scenario.initial_state.rotation
            )
            rate_cost = 0.08 * float(np.linalg.norm(self.state.body_rate))
        else:
            attitude_cost = 0.0
            rate_cost = 0.0
        return -(position_cost + velocity_cost + attitude_cost + rate_cost)

    def _return_errors(self) -> dict[str, float]:
        initial = self.scenario.initial_state
        return {
            "position_error": float(np.linalg.norm(self.state.position - initial.position)),
            "velocity_error": float(np.linalg.norm(self.state.velocity - initial.velocity)),
            "attitude_error": attitude_error_angle(self.state.rotation, initial.rotation),
            "rate_error": float(np.linalg.norm(self.state.body_rate - initial.body_rate)),
        }

    def _closed_loop_complete(self) -> tuple[bool, dict[str, float]]:
        errors = self._return_errors()
        complete = (
            self.progress_index == len(self.scenario.order)
            and errors["position_error"] <= self.config.return_position_tolerance
            and errors["velocity_error"] <= self.config.return_velocity_tolerance
            and errors["attitude_error"] <= self.config.return_attitude_tolerance
            and errors["rate_error"] <= self.config.return_rate_tolerance
        )
        return bool(complete), errors

    def step(self, action: Array) -> tuple[Observation, float, bool, bool, dict]:
        previous_state = self.state.copy()
        previous_time = self.time
        previous_expected_window = (
            self.scenario.order[self.progress_index]
            if self.progress_index < len(self.scenario.order)
            else None
        )
        applied_action = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        self.state, diagnostics = self.dynamics.step(
            self.state, applied_action, self.config.dt
        )
        self._last_diagnostics = diagnostics
        normalized_rotors = diagnostics.rotor_thrusts / max(
            self.quadrotor_config.rotor_thrust_max, 1.0e-6
        )
        self.energy_proxy += self.config.dt * float(
            np.sum(normalized_rotors**2)
        )
        self.saturated_steps += int(diagnostics.saturation_fraction > 0.0)
        self.time += self.config.dt
        self.step_count += 1

        failure: str | None = None
        passed_gate = False
        crossing_margin = float("nan")
        expected_window = (
            self.scenario.order[self.progress_index]
            if self.progress_index < len(self.scenario.order)
            else None
        )
        for window_index, window in enumerate(self.scenario.windows):
            event = window.crossing_event(
                previous_state.position,
                previous_time,
                self.state.position,
                self.time,
            )
            if not event.occurred:
                continue
            if event.frame_collision:
                failure = "window_collision"
                crossing_margin = event.margin
                break
            if not event.safe:
                continue
            if window_index != expected_window:
                failure = "order_violation"
                crossing_margin = event.margin
                break
            passed_gate = True
            crossing_margin = event.margin
            self.crossing_records.append(
                {
                    "window_index": window_index,
                    "window_name": window.name,
                    "time": float(event.time),
                    "margin": float(event.margin),
                    "safe_area": float(window.state(float(event.time)).safe_polygon.area),
                    "temporal_slack": float(
                        min(
                            event.time - window.containing_opportunity(float(event.time))[0],
                            window.containing_opportunity(float(event.time))[1] - event.time,
                        )
                    )
                    if window.containing_opportunity(float(event.time)) is not None
                    else float("nan"),
                }
            )
            self.progress_index += 1
            expected_window = (
                self.scenario.order[self.progress_index]
                if self.progress_index < len(self.scenario.order)
                else None
            )

        missed_this_step = 0
        if previous_expected_window is not None and not passed_gate:
            expected = self.scenario.windows[previous_expected_window]
            if not expected.is_passable(previous_time):
                self.closed_target_time += self.config.dt
            for _, opportunity_end in expected.planned_opportunities:
                if previous_time < opportunity_end <= self.time:
                    missed_this_step += 1
            self.missed_opportunities += missed_this_step

        if self.state.position[2] <= self.config.floor_height:
            failure = failure or "floor_collision"
        if float(np.linalg.norm(self.state.position[:2])) > self.config.workspace_radius:
            failure = failure or "workspace_exit"

        success, return_errors = self._closed_loop_complete()
        terminated = bool(failure is not None or success)
        truncated = bool(not terminated and self.step_count >= self.config.max_steps)
        current_potential = self._potential()
        progress_reward = self.config.reward_progress * (
            current_potential - self._previous_potential
        )
        if failure is not None:
            progress_reward = min(0.0, progress_reward)
        reward = -self.config.reward_time * self.config.dt + progress_reward
        if passed_gate:
            reward += self.config.reward_gate
        if success:
            reward += self.config.reward_success
        if failure == "order_violation":
            reward += self.config.reward_order_violation
        elif failure is not None:
            reward += self.config.reward_collision
        if truncated:
            reward += self.config.reward_timeout
        reward += (
            self.config.reward_missed_opportunity * missed_this_step
        )
        reward -= self.config.reward_smoothness * float(
            np.sum((applied_action - self.previous_action) ** 2)
        )
        reward -= self.config.reward_energy * float(np.sum(normalized_rotors**2))

        self.previous_action = applied_action
        self._previous_potential = current_potential
        observation = self.observe()
        opportunity_passable = False
        opportunity_start = float("nan")
        opportunity_end = float("nan")
        if self.progress_index < len(self.scenario.order):
            current_window = self.scenario.windows[
                self.scenario.order[self.progress_index]
            ]
            opportunity_passable = current_window.is_passable(self.time)
            interval = current_window.next_opportunity(self.time)
            if interval is not None:
                opportunity_start, opportunity_end = interval
        info = {
            "success": success,
            "failure": failure,
            "stage": self.stage,
            "scenario_seed": self.scenario.seed,
            "time": self.time,
            "progress_index": self.progress_index,
            "windows_total": len(self.scenario.order),
            "passed_gate": passed_gate,
            "crossing_margin": crossing_margin,
            "rotor_thrust_max": float(np.max(diagnostics.rotor_thrusts)),
            "rotor_saturation_fraction": diagnostics.saturation_fraction,
            "opportunity_passable": opportunity_passable,
            "opportunity_start": opportunity_start,
            "opportunity_end": opportunity_end,
            "missed_opportunities": self.missed_opportunities,
            "missed_this_step": missed_this_step,
            "energy_proxy": self.energy_proxy,
            "saturated_step_fraction": self.saturated_steps
            / max(self.step_count, 1),
            "closed_target_time": self.closed_target_time,
            **return_errors,
        }
        return observation, float(reward), terminated, truncated, info

    def compose_action(self, residual: Array) -> Array:
        return np.clip(
            self.nominal_action() + self.config.residual_scale * np.asarray(residual),
            -1.0,
            1.0,
        ).astype(np.float32)
