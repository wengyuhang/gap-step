"""Only numerical validity and measured gate identity govern installation."""
import numpy as np
from convex_timevarying_window.wang_free_state_mpc.events import Progress as Parent


def candidate_reason(candidate, gate, phase):
    if candidate.first_gate != gate: return 'stale_gate'
    if phase != 'approach': return 'body_exiting'
    for value in (candidate.durations, candidate.coefficients, candidate.x, [candidate.made_at]):
        if not np.isfinite(value).all(): return 'invalid_numeric_result'
    if np.any(np.asarray(candidate.durations) <= 0): return 'invalid_duration'
    return None


def planning_block_reason(progress, reference, target_time, min_leg_seconds):
    # No predicted-slab, passage-time, geometry or residual checks.
    return 'body_exiting' if progress.crossed else None


class Progress(Parent):
    def update(self, p0, p1, t0, t1):
        if self.gate >= len(self.windows): return False
        w = self.windows[self.gate]; sign = self.signs[self.gate]
        q0, z0 = w.world_to_local(p0, t0); q1, z1 = w.world_to_local(p1, t1)
        a, b = sign*z0, sign*z1
        if not self.crossed and a >= 0. and b < 0.:
            alpha = abs(a)/(abs(a)+abs(b)); q = (1-alpha)*q0+alpha*q1
            # Actual opening membership is task counting, not a safety margin.
            if not w.aperture.contains(q):
                self.violation = 'missed_current_opening'
                return False
            self.crossings.append(dict(gate=self.gate, time=float(t0+alpha*(t1-t0)), q=q.tolist(),
                                       margin=float(w.aperture.boundary_distance(q)-self.radius)))
            self.crossed = True
        elif self.crossed and a < 0. and b >= 0.:
            self.violation = 'reversed_before_body_exit'
            self.crossed = False
        if self.crossed and b <= -(self.radius+self.thickness):
            self.exits.append(dict(gate=self.gate, time=float(t1), signed_distance=float(b)))
            self.gate += 1; self.crossed = False
            return True
        return False
