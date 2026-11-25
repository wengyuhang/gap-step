from __future__ import annotations

import numpy as np
import torch

from convex_timevarying_window.privileged_safe_teacher import (
    DirectControlTeacher,
    PlannerPrivilege,
    PrivilegedTeacherEnv,
)
from convex_timevarying_window.privileged_safe_teacher.safety_filter import (
    PredictiveSafetyFilter,
)


def test_privilege_contains_only_sparse_gate_targets() -> None:
    privilege = PlannerPrivilege()
    assert len(privilege.gates) == 7
    assert all(gate.local_point.shape == (2,) for gate in privilege.gates)
    assert not hasattr(privilege, "reference_position")
    assert not hasattr(privilege, "reference_state")


def test_direct_teacher_action_shape_and_bounds() -> None:
    env = PrivilegedTeacherEnv(seed=1)
    model = DirectControlTeacher(env.observation_dim)
    action = model(torch.as_tensor(env.observe()).unsqueeze(0)).detach().numpy()[0]
    assert action.shape == (4,)
    assert np.all(np.abs(action) <= 1.0)


def test_nominal_recovery_controller_completes_without_collision() -> None:
    env = PrivilegedTeacherEnv(seed=2)
    env.reset(perturbation_scale=0.0)
    while True:
        _, _, done, info = env.step(env.expert_action())
        if done:
            break
    assert info.finished
    assert not info.collision
    assert env.route_index == 7
    assert env.minimum_frame_margin > 0.0


def test_filter_prefers_first_recoverable_projection() -> None:
    class FakeEnvironment:
        def expert_action(self):
            return np.array([0.0, 0.0, 0.0, 0.0])

        def rollout_safety(self, action, horizon):
            return float(0.1 - np.linalg.norm(action)), 0.0

    decision = PredictiveSafetyFilter(horizon=0.3, reserve=0.04).filter(
        FakeEnvironment(), np.array([0.2, 0.0, 0.0, 0.0])
    )
    assert decision.intervened
    assert decision.source == "planner_recovery"
    assert decision.predicted_minimum_margin >= 0.04

