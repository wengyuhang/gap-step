"""Same hard passage/PV-MINCO problem; unrestricted arrival phase for both."""
import numpy as np
from dataclasses import dataclass
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_planner import Planner as Parent, Config as ParentConfig


@dataclass(frozen=True)
class Config(ParentConfig):
    # None disables wall-clock truncation, rather than substituting a large budget.
    max_solve_ms: float | None = None


class Planner(Parent):
    def arrival_initialization(self, window, now, initial):
        # Equal geometric seeds, centre point in every seed; omega is solely
        # numerical scaling, not a restriction on the crossing phase.
        omega = 2*np.pi/window.motion.translation_period
        return [initial*k for k in (1., .8, 1.2, 1.5)], omega, window.motion.translation_period

    def arrival_bounds(self, window, now, seed_arrival, omega, period):
        return .3, 15.
