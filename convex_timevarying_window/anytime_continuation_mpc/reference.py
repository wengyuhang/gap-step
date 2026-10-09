import numpy as np
from convex_timevarying_window.race_to_go_mpc.reference import Reference as Parent


class Reference(Parent):
    def state_at(self,t,orders=4):
        with self.lock:
            plan=self.plan_at(t)
            if plan is not None and getattr(plan,'is_bootstrap',False) and t>plan.end_time:
                raise RuntimeError('bootstrap_reference_exhausted: no fabricated hover tail')
            return super().state_at(t,orders)

    def candidate_reason(self,candidate,gate,phase):
        with self.lock:
            old=self.plan_at(candidate.made_at)
            if old is None or not getattr(old,'is_bootstrap',False):
                return super().candidate_reason(candidate,gate,phase)
            reason=None;head,_=self.state_at(candidate.made_at,3)
            if candidate.first_gate!=gate or phase!='approach':reason='stale_gate_stage'
            proposed=np.concatenate([candidate.sample(candidate.made_at,d) for d in range(3)])
            if np.max(abs(head-proposed))>1e-5:reason='head_timeline_changed'
            self.contract_log.append(dict(time=candidate.made_at,bootstrap_transition=True,
                old_finish_s=None,new_finish_s=candidate.nominal_finish_s,reason=reason))
            return reason
