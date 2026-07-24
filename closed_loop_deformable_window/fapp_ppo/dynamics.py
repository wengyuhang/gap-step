"""Rigid-body quadrotor dynamics with CTBR commands and rotor allocation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import QuadrotorConfig


Array = np.ndarray


def skew(vector: Array) -> Array:
    x, y, z = np.asarray(vector, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)


def exp_so3(rotation_vector: Array) -> Array:
    vector = np.asarray(rotation_vector, dtype=float)
    angle = float(np.linalg.norm(vector))
    if angle < 1.0e-9:
        return np.eye(3) + skew(vector)
    axis_hat = skew(vector / angle)
    return np.eye(3) + np.sin(angle) * axis_hat + (1.0 - np.cos(angle)) * (axis_hat @ axis_hat)


def attitude_error_angle(rotation_a: Array, rotation_b: Array) -> float:
    cosine = 0.5 * (float(np.trace(rotation_a.T @ rotation_b)) - 1.0)
    return float(np.arccos(np.clip(cosine, -1.0, 1.0)))


@dataclass
class QuadrotorState:
    position: Array
    velocity: Array
    rotation: Array
    body_rate: Array

    def copy(self) -> "QuadrotorState":
        return QuadrotorState(
            self.position.copy(),
            self.velocity.copy(),
            self.rotation.copy(),
            self.body_rate.copy(),
        )


@dataclass(frozen=True)
class DynamicsDiagnostics:
    rotor_thrusts: Array
    collective_thrust: float
    torque: Array
    saturation_fraction: float
    acceleration: Array


class QuadrotorDynamics:
    def __init__(self, config: QuadrotorConfig):
        self.config = config
        self.inertia = np.diag(np.asarray(config.inertia, dtype=float))
        arm = config.arm_length
        yaw = config.yaw_moment_coefficient
        self.mixing = np.array(
            [
                [1.0, 1.0, 1.0, 1.0],
                [arm, -arm, -arm, arm],
                [-arm, -arm, arm, arm],
                [yaw, -yaw, yaw, -yaw],
            ],
            dtype=float,
        )
        self.allocation = np.linalg.inv(self.mixing)

    def hover_state(self, position: Array) -> QuadrotorState:
        return QuadrotorState(
            np.asarray(position, dtype=float).copy(),
            np.zeros(3),
            np.eye(3),
            np.zeros(3),
        )

    def _specific_thrust_from_action(self, value: float) -> float:
        cfg = self.config
        minimum = 0.15 * cfg.gravity
        maximum = 4.0 * cfg.rotor_thrust_max / cfg.mass
        clipped = float(np.clip(value, -1.0, 1.0))
        if clipped >= 0.0:
            return cfg.gravity + clipped * (maximum - cfg.gravity)
        return cfg.gravity + clipped * (cfg.gravity - minimum)

    def collective_action(self, specific_thrust: float) -> float:
        cfg = self.config
        minimum = 0.15 * cfg.gravity
        maximum = 4.0 * cfg.rotor_thrust_max / cfg.mass
        value = float(np.clip(specific_thrust, minimum, maximum))
        if value >= cfg.gravity:
            return (value - cfg.gravity) / max(maximum - cfg.gravity, 1.0e-9)
        return (value - cfg.gravity) / max(cfg.gravity - minimum, 1.0e-9)

    def step(
        self,
        state: QuadrotorState,
        ctbr_action: Array,
        dt: float,
    ) -> tuple[QuadrotorState, DynamicsDiagnostics]:
        cfg = self.config
        action = np.clip(np.asarray(ctbr_action, dtype=float), -1.0, 1.0)
        if action.shape != (4,):
            raise ValueError("CTBR action must have shape (4,)")
        desired_collective = cfg.mass * self._specific_thrust_from_action(float(action[0]))
        desired_rate = action[1:] * cfg.max_body_rate
        desired_rate_derivative = (desired_rate - state.body_rate) / cfg.body_rate_time_constant
        angular_momentum = self.inertia @ state.body_rate
        desired_torque = self.inertia @ desired_rate_derivative + np.cross(
            state.body_rate, angular_momentum
        )
        desired_wrench = np.concatenate(([desired_collective], desired_torque))
        raw_rotors = self.allocation @ desired_wrench
        rotor_thrusts = np.clip(
            raw_rotors, cfg.rotor_thrust_min, cfg.rotor_thrust_max
        )
        saturation = float(np.mean(np.abs(rotor_thrusts - raw_rotors) > 1.0e-8))
        wrench = self.mixing @ rotor_thrusts
        collective = float(wrench[0])
        torque = wrench[1:]

        omega_dot = np.linalg.solve(
            self.inertia,
            torque - np.cross(state.body_rate, self.inertia @ state.body_rate),
        )
        next_rate = state.body_rate + dt * omega_dot
        next_rotation = state.rotation @ exp_so3(0.5 * dt * (state.body_rate + next_rate))
        # A cheap polar projection prevents long rollouts from accumulating drift.
        u, _, vt = np.linalg.svd(next_rotation)
        next_rotation = u @ vt
        if np.linalg.det(next_rotation) < 0.0:
            u[:, -1] *= -1.0
            next_rotation = u @ vt

        thrust_world = next_rotation[:, 2] * (collective / cfg.mass)
        acceleration = (
            thrust_world
            - np.array([0.0, 0.0, cfg.gravity])
            - cfg.linear_drag * state.velocity
        )
        next_velocity = state.velocity + dt * acceleration
        next_position = state.position + 0.5 * dt * (state.velocity + next_velocity)
        next_state = QuadrotorState(next_position, next_velocity, next_rotation, next_rate)
        return next_state, DynamicsDiagnostics(
            rotor_thrusts=rotor_thrusts,
            collective_thrust=collective,
            torque=torque,
            saturation_fraction=saturation,
            acceleration=acceleration,
        )

