"""Allow local tradeoffs, but never postpone the stored complete race plan."""
import numpy as np
from convex_timevarying_window.wang_free_state_mpc.reference import Reference as Base


class Reference(Base):
    def __init__(self,start):
        super().__init__(start)
        self.contract_log=[]

    def candidate_reason(self,candidate,gate,phase):
        with self.lock:
            reason=None
            head,old=self.state_at(candidate.made_at,3)
            if candidate.first_gate!=gate or phase!='approach':reason='stale_gate_stage'
            proposed=np.concatenate([candidate.sample(candidate.made_at,d) for d in range(3)])
            if old is not None:
                if np.max(abs(head-proposed))>1e-5:reason='head_timeline_changed'
                elif candidate.nominal_finish_s>old.nominal_finish_s+1e-8:reason='delayed_race_finish'
            self.contract_log.append(dict(time=candidate.made_at,
                old_finish_s=old.nominal_finish_s if old else None,
                new_finish_s=candidate.nominal_finish_s,reason=reason))
            return reason
