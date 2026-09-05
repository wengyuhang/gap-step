"""One-decision planning environment for the deliberately small RL method."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState
from nonconvex_timevarying_window.sip_dynatogt.model import (
    SIPConfig,
    SIPProblem,
)

from .model import VehicleState, observation
from .planner import InvalidTraversalPoint, PlanBounds, build_plan
from .shield import CertificateFunction, admit


@dataclass(frozen=True)
class RewardConfig:
    infeasible_penalty: float = 100.0

    def __post_init__(self) -> None:
        if not np.isfinite(self.infeasible_penalty) or self.infeasible_penalty <= 0:
            raise ValueError("infeasible_penalty must be finite and positive")


class PlanningEnvironment:
    """Contextual one-step MDP: propose a complete remaining flight plan."""

    def __init__(
        self,
        problem: SIPProblem,
        vehicle_state: VehicleState,
        start_state: BoundaryState,
        finish_state: BoundaryState,
        *,
        plan_bounds: PlanBounds | None = None,
        certificate_config: SIPConfig | None = None,
        reward_config: RewardConfig | None = None,
        certificate_function: CertificateFunction | None = None,
    ) -> None:
        self.problem = problem
        self.vehicle_state = vehicle_state
        self.start_state = start_state
        self.finish_state = finish_state
        self.plan_bounds = plan_bounds or PlanBounds()
        self.certificate_config = certificate_config
        self.reward_config = reward_config or RewardConfig()
        self.certificate_function = certificate_function

    def reset(self) -> np.ndarray:
        return observation(self.problem, self.vehicle_state)

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, dict]:
        try:
            proposal = build_plan(
                self.problem,
                self.start_state,
                self.finish_state,
                action,
                self.plan_bounds,
                start_time=self.vehicle_state.time,
                next_gate=self.vehicle_state.next_gate,
            )
        except (InvalidTraversalPoint, ValueError, np.linalg.LinAlgError) as error:
            return self.reset(), -self.reward_config.infeasible_penalty, True, {
                "accepted": False,
                "status": "INVALID_PROPOSAL",
                "reason": str(error),
            }
        result = admit(
            self.problem,
            proposal,
            self.certificate_config,
            certificate_function=self.certificate_function,
        )
        reward = (
            -proposal.total_time
            if result.accepted
            else -self.reward_config.infeasible_penalty
        )
        return self.reset(), float(reward), True, {
            "accepted": result.accepted,
            "status": result.certificate.status.value,
            "reason": result.certificate.reason,
            "total_time": proposal.total_time,
            "certificate": result.certificate,
            "proposal": proposal,
        }
