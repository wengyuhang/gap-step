"""3-D double-integrator racing MDP with exact dynamic-gate crossing audits."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import EnvironmentConfig, ShieldConfig
from .geometry import DynamicGate, crossing_time, make_shape

Array = np.ndarray

# backup plus (desired x speed, local anchor u offset, local anchor v offset)
ACTION_LIBRARY: tuple[tuple[float, float, float], ...] = (
    (0.0, 0.0, 0.0),
    (0.85, 0.0, 0.0), (1.55, 0.0, 0.0), (2.35, 0.0, 0.0), (2.85, 0.0, 0.0),
    (1.55, 0.24, 0.0), (1.55, -0.24, 0.0), (1.55, 0.0, 0.24), (1.55, 0.0, -0.24),
    (2.35, 0.20, 0.0), (2.35, -0.20, 0.0), (2.35, 0.0, 0.20), (2.35, 0.0, -0.20),
)


@dataclass
class EnvState:
    position: Array
    velocity: Array
    time: float
    gate_index: int
    steps: int

    def copy(self) -> "EnvState":
        return EnvState(self.position.copy(), self.velocity.copy(), self.time, self.gate_index, self.steps)


class AVSEnvironment:
    """Known-dynamics benchmark; the vehicle is a clearance sphere with acceleration control."""

    action_dim = len(ACTION_LIBRARY)

    def __init__(self, environment: EnvironmentConfig | None = None, shield: ShieldConfig | None = None, *, seed: int = 0):
        self.config = environment or EnvironmentConfig()
        self.shield_config = shield or ShieldConfig()
        self.base_seed = int(seed)
        self.episode_index = 0
        self.gates: list[DynamicGate] = []
        self.state: EnvState
        self.safety_violations = 0
        self.shield_interventions = 0
        self.crossing_margins: list[float] = []
        self.reset(seed=seed)

    @property
    def observation_dim(self) -> int:
        return int(self.observe().shape[0])

    @property
    def required_margin(self) -> float:
        return self.config.drone_radius + self.config.geometry_guard

    def _make_gates(self, seed: int) -> list[DynamicGate]:
        rng = np.random.default_rng(seed)
        shapes = ("star", "u_notch", "wavy")
        gates = []
        for index in range(self.config.gate_count):
            jitter = self.config.domain_randomization * rng.uniform(-1.0, 1.0)
            gates.append(DynamicGate(
                f"G{index + 1}", 3.0 + 3.0 * index + 0.12 * jitter,
                make_shape(shapes[index % len(shapes)]),
                phase=0.9 * index + 1.7 * jitter,
                motion_scale=self.config.motion_scale * (1.0 + 0.15 * jitter),
            ))
        return gates

    def reset(self, *, seed: int | None = None) -> tuple[Array, dict]:
        scenario_seed = self.base_seed + self.episode_index if seed is None else int(seed)
        self.episode_index += 1
        self.gates = self._make_gates(scenario_seed)
        self.state = EnvState(np.array([0.0, 0.0, 1.55]), np.zeros(3), 0.0, 0, 0)
        self.safety_violations = 0
        self.shield_interventions = 0
        self.crossing_margins = []
        return self.observe(), {"scenario_seed": scenario_seed}

    def _target_gate(self, state: EnvState | None = None) -> DynamicGate | None:
        s = self.state if state is None else state
        return self.gates[s.gate_index] if s.gate_index < len(self.gates) else None

    def _offset_target(self, gate: DynamicGate, query_time: float, offset_u: float, offset_v: float) -> Array:
        centre, angle, _, _ = gate.state(query_time)
        rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        return gate.anchor(query_time) + rotation @ np.array([offset_u, offset_v])

    def command(self, action: int, state: EnvState | None = None) -> Array:
        s = self.state if state is None else state
        gate = self._target_gate(s)
        if gate is None:
            desired_speed, offset_u, offset_v = 2.35, 0.0, 0.0
            target_yz = np.array([0.0, 1.55])
            target_x = self.config.goal_x
        else:
            desired_speed, offset_u, offset_v = ACTION_LIBRARY[int(action)]
            distance = max(0.0, gate.x - s.position[0])
            if action == 0:
                desired_speed = 0.0
            eta = distance / max(desired_speed, 0.45)
            query_time = s.time + min(eta, 2.0)
            target_yz = self._offset_target(gate, query_time, offset_u, offset_v)
            target_x = gate.x + 0.20
        if gate is not None and action == 0:
            # Control-invariant backup: stop longitudinal motion before the
            # next plane while continuing to centre laterally on its opening.
            accel_x = -min(self.shield_config.backup_brake_accel, max(0.0, s.velocity[0]) / self.config.dt)
        else:
            accel_x = 2.5 * (desired_speed - s.velocity[0]) + 0.10 * (target_x - s.position[0])
        accel_yz = self.shield_config.lateral_kp * (target_yz - s.position[1:]) - self.shield_config.lateral_kd * s.velocity[1:]
        accel_x = float(np.clip(accel_x, -self.config.max_accel_x, self.config.max_accel_x))
        lateral_norm = float(np.linalg.norm(accel_yz))
        if lateral_norm > self.config.max_accel_yz:
            accel_yz *= self.config.max_accel_yz / lateral_norm
        return np.array([accel_x, *accel_yz], dtype=float)

    def _integrate(self, state: EnvState, acceleration: Array) -> tuple[EnvState, list[tuple[int, float, float]]]:
        dt = self.config.dt
        velocity = state.velocity + acceleration * dt
        velocity[0] = np.clip(velocity[0], -0.2, self.config.max_speed_x)
        lateral_speed = float(np.linalg.norm(velocity[1:]))
        if lateral_speed > self.config.max_speed_yz:
            velocity[1:] *= self.config.max_speed_yz / lateral_speed
        # Use the effective acceleration after velocity limiting so the position,
        # velocity, and continuous crossing equation describe one trajectory.
        effective_acceleration = (velocity - state.velocity) / dt
        position = state.position + state.velocity * dt + 0.5 * effective_acceleration * dt * dt
        next_state = EnvState(position, velocity, state.time + dt, state.gate_index, state.steps + 1)
        crossings: list[tuple[int, float, float]] = []
        while next_state.gate_index < len(self.gates):
            gate = self.gates[next_state.gate_index]
            tau = crossing_time(
                float(state.position[0]), float(state.velocity[0]),
                float(effective_acceleration[0]), gate.x, dt,
            )
            if tau is None:
                break
            event_time = state.time + tau
            crossing_yz = state.position[1:] + state.velocity[1:] * tau + 0.5 * effective_acceleration[1:] * tau * tau
            margin = gate.margin(crossing_yz, event_time)
            crossings.append((next_state.gate_index, event_time, margin))
            if margin + 1.0e-12 < self.required_margin:
                break
            next_state.gate_index += 1
        return next_state, crossings

    def _transition_safe(self, crossings: list[tuple[int, float, float]]) -> bool:
        return all(margin + 1.0e-12 >= self.required_margin for _, _, margin in crossings)

    def action_mask(self) -> Array:
        """Safe action support from candidate-plus-backup rollouts."""
        mask = np.zeros(self.action_dim, dtype=bool)
        for action in range(self.action_dim):
            simulated = self.state.copy()
            safe = True
            for horizon_index in range(self.shield_config.backup_horizon_steps):
                rollout_action = action if horizon_index == 0 else 0
                simulated, crossings = self._integrate(simulated, self.command(rollout_action, simulated))
                if not self._transition_safe(crossings):
                    safe = False
                    break
                if simulated.gate_index >= len(self.gates) or (horizon_index > 0 and simulated.velocity[0] <= 1.0e-6 and self._target_gate(simulated).x - simulated.position[0] > 0.32):
                    break
            mask[action] = safe
        if not mask.any():
            # This indicates a broken viability invariant; exposing it is safer than silently widening the set.
            raise RuntimeError("viability shield found no safe action")
        return mask

    def observe(self) -> Array:
        s = self.state
        features = [
            *(s.position / np.array([self.config.goal_x, 2.0, 2.0])),
            *(s.velocity / np.array([self.config.max_speed_x, self.config.max_speed_yz, self.config.max_speed_yz])),
            s.time / (self.config.max_steps * self.config.dt),
        ]
        one_hot = np.zeros(len(self.gates) + 1)
        one_hot[min(s.gate_index, len(self.gates))] = 1.0
        features.extend(one_hot.tolist())
        for index, gate in enumerate(self.gates):
            centre, angle, scale, _ = gate.state(s.time)
            anchor = gate.anchor(s.time)
            features.extend([
                (gate.x - s.position[0]) / self.config.goal_x,
                *(centre - s.position[1:]) / 2.0,
                np.sin(angle), np.cos(angle),
                *scale,
                *(anchor - s.position[1:]) / 2.0,
                float(index == s.gate_index),
            ])
        return np.asarray(features, dtype=np.float32)

    def step(self, action: int, *, mask: Array | None = None) -> tuple[Array, float, bool, bool, dict]:
        safe_mask = self.action_mask() if mask is None else np.asarray(mask, dtype=bool)
        requested_action = int(action)
        if not safe_mask[requested_action]:
            requested_action = 0
            self.shield_interventions += 1
        old_x = float(self.state.position[0])
        old_gate_index = self.state.gate_index
        next_state, crossings = self._integrate(self.state, self.command(requested_action))
        safe = self._transition_safe(crossings)
        if not safe:
            self.safety_violations += 1
        for gate_index, _, margin in crossings:
            if margin >= self.required_margin and gate_index >= len(self.crossing_margins):
                self.crossing_margins.append(float(margin))
        self.state = next_state
        progress = max(0.0, float(self.state.position[0]) - old_x)
        gates_crossed = self.state.gate_index - old_gate_index
        reward = 0.32 * progress - 0.018 + 2.5 * gates_crossed
        reached_goal = self.state.gate_index == len(self.gates) and self.state.position[0] >= self.config.goal_x - self.config.goal_tolerance
        if reached_goal:
            reward += 8.0 + 0.08 * (self.config.max_steps - self.state.steps)
        if not safe:
            reward -= 20.0
        terminated = bool(reached_goal or not safe)
        truncated = bool(self.state.steps >= self.config.max_steps and not terminated)
        info = {
            "success": bool(reached_goal),
            "safe": bool(self.safety_violations == 0),
            "safety_violations": self.safety_violations,
            "shield_interventions": self.shield_interventions,
            "elapsed_time": self.state.time,
            "gates_crossed": self.state.gate_index,
            "minimum_crossing_margin": min(self.crossing_margins, default=float("inf")),
            "required_margin": self.required_margin,
            "action_mask_fraction": float(np.mean(safe_mask)),
            "applied_action": requested_action,
        }
        return self.observe(), float(reward), terminated, truncated, info
