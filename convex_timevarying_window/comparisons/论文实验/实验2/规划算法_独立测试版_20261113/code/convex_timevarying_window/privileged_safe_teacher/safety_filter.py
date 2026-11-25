"""Short-horizon whole-body safety filter for direct neural actions."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from .environment import PrivilegedTeacherEnv


@dataclass(frozen=True)
class SafetyDecision:
    action: np.ndarray
    intervened: bool
    source: str
    predicted_minimum_margin: float
    predicted_saturation_fraction: float


class PredictiveSafetyFilter:
    """Choose the least-modified action passing a short forward safety audit.

    Every candidate is rolled through the x500 rigid-body model and checked
    against every physical gate frame using the true vehicle sphere radius.
    The fallback is the candidate with the largest predicted clearance, so the
    module remains deterministic even if no candidate reaches the reserve.
    """

    def __init__(self, horizon: float = 0.40, reserve: float = 0.10):
        self.horizon = float(horizon)
        self.reserve = float(reserve)

    def filter(self, env: "PrivilegedTeacherEnv", policy_action: np.ndarray) -> SafetyDecision:
        policy = np.clip(np.asarray(policy_action, dtype=float), -1.0, 1.0)
        expert = np.asarray(env.expert_action(), dtype=float)
        candidates = (
            ("policy", policy),
            ("projected_25", 0.75 * policy + 0.25 * expert),
            ("projected_50", 0.50 * policy + 0.50 * expert),
            ("planner_recovery", expert),
        )
        scored: list[tuple[str, np.ndarray, float, float]] = []
        for source, action in candidates:
            clipped = np.clip(action, -1.0, 1.0)
            margin, saturation = env.rollout_safety(clipped, self.horizon)
            scored.append((source, clipped, margin, saturation))
            # Rotor limits are already applied inside the rollout dynamics.
            # Saturation remains a diagnostic; rejecting a collision-free
            # action solely because it saturated can select a lower-clearance
            # fallback.
            if margin >= self.reserve:
                return SafetyDecision(
                    action=clipped.astype(np.float32),
                    intervened=source != "policy",
                    source=source,
                    predicted_minimum_margin=float(margin),
                    predicted_saturation_fraction=float(saturation),
                )
        if env._nearby_windows(self.horizon):
            source, action, margin, saturation = scored[-1]
        else:
            source, action, margin, saturation = max(
                scored, key=lambda item: (item[2], -item[3])
            )
        return SafetyDecision(
            action=action.astype(np.float32),
            intervened=True,
            source=f"best_effort_{source}",
            predicted_minimum_margin=float(margin),
            predicted_saturation_fraction=float(saturation),
        )
