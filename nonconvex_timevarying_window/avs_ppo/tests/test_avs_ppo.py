from __future__ import annotations

import numpy as np
import torch

from nonconvex_timevarying_window.avs_ppo.config import ExperimentConfig
from nonconvex_timevarying_window.avs_ppo.environment import AVSEnvironment
from nonconvex_timevarying_window.avs_ppo.geometry import DynamicGate, crossing_time, make_shape, point_in_polygon, signed_margin
from nonconvex_timevarying_window.avs_ppo.model import MaskedActorCritic
from nonconvex_timevarying_window.avs_ppo.ppo import collect_rollout, generalized_advantage


def test_nonconvex_geometry_rejects_convex_hull_notch() -> None:
    polygon = make_shape("u_notch")
    assert point_in_polygon(np.array([0.0, -0.7]), polygon)
    assert not point_in_polygon(np.array([0.0, 0.7]), polygon)
    assert signed_margin(np.array([0.0, 0.7]), polygon) < 0.0


def test_dynamic_gate_changes_and_has_interior_anchor() -> None:
    gate = DynamicGate("G", 3.0, make_shape("star"), 0.3)
    assert not np.allclose(gate.state(0.0)[3], gate.state(1.0)[3])
    for time in (0.0, 0.7, 2.1):
        assert gate.margin(gate.anchor(time), time) > 0.25


def test_constant_acceleration_crossing_time_is_exact() -> None:
    tau = crossing_time(0.0, 1.0, 2.0, 0.75, 1.0)
    assert tau is not None
    assert np.isclose(1.0 * tau + 0.5 * 2.0 * tau * tau, 0.75)


def test_shield_masks_deliberately_unsafe_fast_crossing() -> None:
    env = AVSEnvironment(seed=3)
    gate = env.gates[0]
    # One metre leaves a viable braking action, while the fastest command
    # would enter the gate plane before lateral recovery is possible.
    env.state.position = np.array([gate.x - 1.0, 2.8, 2.8])
    env.state.velocity = np.array([2.0, 0.0, 0.0])
    mask = env.action_mask()
    assert not mask[4]
    assert mask.any()


def test_backup_action_stops_before_first_gate() -> None:
    env = AVSEnvironment(seed=5)
    env.state.position[0] = 1.0
    env.state.velocity[0] = 2.0
    for _ in range(20):
        next_state, crossings = env._integrate(env.state, env.command(0))
        assert env._transition_safe(crossings)
        env.state = next_state
    assert env.state.velocity[0] <= 1.0e-9
    assert env.state.position[0] < env.gates[0].x


def test_masked_policy_never_samples_forbidden_action() -> None:
    model = MaskedActorCritic(5, 4, 16)
    observations = torch.zeros((256, 5))
    masks = torch.tensor([[True, False, True, False]]).expand(256, -1)
    actions, _, _ = model.sample(observations, masks)
    assert set(actions.tolist()) <= {0, 2}


def test_rollout_is_safe_and_gae_shapes_match() -> None:
    config = ExperimentConfig()
    environments = [AVSEnvironment(config.environment, config.shield, seed=11)]
    model = MaskedActorCritic(environments[0].observation_dim, environments[0].action_dim, 24)
    rollout, _ = collect_rollout(environments, model, 12, torch.device("cpu"))
    advantages, returns = generalized_advantage(rollout, 0.99, 0.95)
    assert advantages.shape == rollout.rewards.shape == returns.shape
    assert all(record["safety_violations"] == 0 for record in rollout.episodes)
