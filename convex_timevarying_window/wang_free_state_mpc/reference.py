"""One timeline for MPC, future PVA, and reference activation."""
import threading
import numpy as np


class Reference:
    def __init__(self, start):
        self.start = np.asarray(start); self.active = None; self.pending = None
        self.gate = 0; self.phase = 'approach'; self.discarded = []
        self.lock = threading.RLock()

    def advance(self, now, gate=None, phase=None):
        with self.lock:
            if gate is not None: self.gate = gate
            if phase is not None: self.phase = phase
            if self.pending is not None:
                if self.pending.first_gate != self.gate or self.phase != 'approach':
                    self.discarded.append(dict(time=float(now), first_gate=self.pending.first_gate,
                        observed_gate=self.gate, reason='passage_stage_changed'))
                    self.pending = None
                elif now >= self.pending.made_at:
                    self.active, self.pending = self.pending, None
            return self.active

    def install(self, plan):
        with self.lock:
            if plan.first_gate != self.gate or self.phase != 'approach': return False
            self.pending = plan; return True

    def plan_at(self, t):
        with self.lock:
            return self.pending if self.pending is not None and t >= self.pending.made_at else self.active

    def state_at(self, t, orders=4):
        with self.lock:
            plan = self.plan_at(t)
            if plan is None: return np.r_[self.start, np.zeros(3*(orders-1))], None
            return np.concatenate([plan.sample(t, d) if d==0 or t<plan.end_time else np.zeros(3)
                                   for d in range(orders)]), plan

    def sample(self, times):
        with self.lock: return np.array([self.state_at(t, 3)[0] for t in times])

    def candidate_reason(self, candidate, gate, phase):
        """A candidate cannot change the head timeline or regress the same race leg."""
        with self.lock:
            if candidate.first_gate != gate or phase != 'approach': return 'stale_gate_stage'
            head, incumbent = self.state_at(candidate.made_at, 3)
            proposed = np.concatenate([candidate.sample(candidate.made_at, d) for d in range(3)])
            # Initial hover has measured-state seeding, not an installed reference.
            if incumbent is not None and np.max(np.abs(head-proposed)) > 1e-5:
                return 'head_timeline_changed'
            if incumbent is not None and incumbent.first_gate == candidate.first_gate and incumbent.gate_count == candidate.gate_count:
                if candidate.end_time > incumbent.end_time+.02: return 'slower_same_horizon'
                if candidate.gate_count and candidate.crossing_times[0] > incumbent.crossing_times[0]+.02:
                    return 'delayed_current_passage'
            return None
