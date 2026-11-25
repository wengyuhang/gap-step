"""Keep physical timeline continuity; no per-gate or approximate global deadline."""
import numpy as np
from convex_timevarying_window.event_free_terminal_mpc.reference import Reference as Parent


class Reference(Parent):
    def candidate_reason(self,candidate,gate,phase):
        with self.lock:
            reason=None;head,old=self.state_at(candidate.made_at,3)
            if candidate.first_gate!=gate or phase!='approach':reason='stale_gate_stage'
            proposed=np.concatenate([candidate.sample(candidate.made_at,d) for d in range(3)])
            if old is not None and np.max(abs(head-proposed))>1e-5:reason='head_timeline_changed'
            self.contract_log.append(dict(time=candidate.made_at,value_score_s=candidate.value_score_s,reason=reason))
            return reason
