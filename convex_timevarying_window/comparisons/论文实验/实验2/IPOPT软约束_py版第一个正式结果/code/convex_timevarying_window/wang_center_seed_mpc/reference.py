"""Keep clock/stage continuity, remove all incumbent passage-time restrictions."""
import numpy as np
from convex_timevarying_window.wang_free_state_mpc.reference import Reference as BaseReference


class Reference(BaseReference):
    def candidate_reason(self, candidate, gate, phase):
        with self.lock:
            if candidate.first_gate != gate or phase != 'approach':
                return 'stale_gate_stage'
            head, incumbent = self.state_at(candidate.made_at, 3)
            proposed = np.concatenate([candidate.sample(candidate.made_at, d) for d in range(3)])
            if incumbent is not None and np.max(np.abs(head-proposed)) > 1e-5:
                return 'head_timeline_changed'
            return None

    def state_at(self, t, orders=4):
        with self.lock:
            plan = self.plan_at(t)
            if plan is not None and not plan.terminal_goal and t > plan.end_time+1e-8:
                raise RuntimeError('unextended_flying_horizon: no stationary fallback')
            return super().state_at(t, orders)
