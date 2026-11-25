"""One septic MINCO piece per leg; events are the ends of gate legs."""
from dataclasses import dataclass
import numpy as np

from convex_timevarying_window.togt_minco_mpc.trajectory import Plan as BasePlan


@dataclass
class Plan(BasePlan):
    @property
    def crossing_times(self):
        return self.made_at + np.cumsum(self.durations)[:self.gate_count]
