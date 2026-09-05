from __future__ import annotations

import numpy as np
import pytest
import torch

from nonconvex_timevarying_window.phaseguard_rl import (
    InvalidTraversalPoint,
    PPOBatch,
    PhaseActorCritic,
    PlanBounds,
    PlanningEnvironment,
    SafePlanManager,
    VehicleState,
    action_dimension,
    admit,
    build_plan,
    observation,
    ppo_update,
    train,
)
from nonconvex_timevarying_window.sc_dynatogt.boundary import Line
from nonconvex_timevarying_window.sc_dynatogt.environment import MotionProfile
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState
from nonconvex_timevarying_window.sip_dynatogt.model import (
    CertificateResult,
    CertificateStatus,
    SIPProblem,
    SIPWindow,
)


def _problem() -> SIPProblem:
    vertices = (
        np.array([-1.0, -1.0]),
        np.array([1.0, -1.0]),
        np.array([1.0, 1.0]),
        np.array([-1.0, 1.0]),
    )
    boundary = tuple(Line(vertices[i], vertices[(i + 1) % 4]) for i in range(4))
    window = SIPWindow(
        "G1",
        np.zeros(3),
        np.zeros(3),
        MotionProfile.static(),
        boundary,
    )
    return SIPProblem("one_square", (window,), (0,))


def _proposal():
    state = BoundaryState(np.array([0.0, 0.0, -2.0]))
    return build_plan(
        _problem(),
        state,
        state,
        np.zeros(action_dimension(1)),
        PlanBounds(minimum_duration=1.0, maximum_duration=2.0),
    )


def _certificate(status: CertificateStatus) -> CertificateResult:
    return CertificateResult(status, status.value, 128, 1, 0, None, None)


def test_plan_decodes_noncentral_point_and_n_plus_one_durations() -> None:
    action = np.array([0.25, -0.20, 0.0, 0.5])
    state = BoundaryState(np.array([0.0, 0.0, -2.0]))
    proposal = build_plan(
        _problem(),
        state,
        state,
        action,
        PlanBounds(local_half_width=1.0, minimum_duration=1.0, maximum_duration=3.0),
    )
    assert proposal.local_points.shape == (1, 2)
    assert proposal.local_points[0] == pytest.approx([0.25, -0.20])
    assert proposal.trajectory.num_segments == 2
    assert proposal.durations == pytest.approx([2.0, 2.5])
    assert proposal.traversal_times == pytest.approx([2.0])


def test_outside_policy_point_is_rejected_before_trajectory_admission() -> None:
    state = BoundaryState(np.array([0.0, 0.0, -2.0]))
    with pytest.raises(InvalidTraversalPoint):
        build_plan(
            _problem(),
            state,
            state,
            np.array([1.0, 1.0, 0.0, 0.0]),
            PlanBounds(local_half_width=2.0),
        )


def test_observation_contains_state_and_phase_and_policy_shape() -> None:
    problem = _problem()
    state = VehicleState(
        np.array([0.1, -0.2, 0.3]),
        np.array([1.0, 0.0, 0.0]),
        np.eye(3),
        np.zeros(3),
        time=0.4,
    )
    encoded = observation(problem, state)
    model = PhaseActorCritic(len(encoded), action_dimension(1), hidden_size=32)
    action, log_probability, value = model.act(torch.tensor(encoded)[None])
    assert action.shape == (1, 4)
    assert log_probability.shape == (1,)
    assert value.shape == (1,)
    assert torch.all(action <= 1.0) and torch.all(action >= -1.0)


def test_plan_manager_is_fail_closed_and_keeps_last_certified_plan() -> None:
    problem = _problem()
    proposal = _proposal()
    manager = SafePlanManager()

    def safe(*_):
        return _certificate(CertificateStatus.CERTIFIED_FEASIBLE)

    def unresolved(*_):
        return _certificate(CertificateStatus.UNRESOLVED)

    accepted = admit(problem, proposal, certificate_function=safe)
    assert manager.submit(accepted)
    assert manager.require_takeoff_plan() is proposal

    rejected = admit(problem, proposal, certificate_function=unresolved)
    assert not manager.submit(rejected)
    assert manager.current is proposal


def test_takeoff_without_certificate_is_forbidden() -> None:
    with pytest.raises(RuntimeError, match="takeoff forbidden"):
        SafePlanManager().require_takeoff_plan()


def test_ppo_update_is_finite() -> None:
    torch.manual_seed(2)
    model = PhaseActorCritic(7, 4, hidden_size=16)
    optimizer = torch.optim.Adam(model.parameters(), lr=1.0e-3)
    observations = torch.randn(12, 7)
    with torch.no_grad():
        actions, old_log_probabilities, _ = model.act(observations)
    batch = PPOBatch(
        observations,
        actions,
        old_log_probabilities,
        torch.linspace(-1.0, 1.0, 12),
        torch.zeros(12),
    )
    metrics = ppo_update(model, optimizer, batch)
    assert all(np.isfinite(value) for value in metrics.values())
    assert metrics["approximate_kl"] >= -1.0e-8


def test_minimal_training_loop_uses_certified_time_reward() -> None:
    problem = _problem()
    boundary_state = BoundaryState(np.array([0.0, 0.0, -2.0]))
    vehicle_state = VehicleState(
        boundary_state.position,
        boundary_state.velocity,
        np.eye(3),
        np.zeros(3),
    )

    def safe(*_):
        return _certificate(CertificateStatus.CERTIFIED_FEASIBLE)

    environment = PlanningEnvironment(
        problem,
        vehicle_state,
        boundary_state,
        boundary_state,
        plan_bounds=PlanBounds(
            local_half_width=0.5,
            minimum_duration=1.0,
            maximum_duration=2.0,
        ),
        certificate_function=safe,
    )
    encoded = environment.reset()
    model = PhaseActorCritic(len(encoded), action_dimension(1), hidden_size=16)
    records = train(environment, model, updates=2, batch_size=4)
    assert len(records) == 2
    assert all(record.certified_rate == 1.0 for record in records)
    assert all(np.isfinite(record.mean_certified_time) for record in records)
