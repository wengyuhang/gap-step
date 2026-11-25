"""The same event identity is compared even when the horizon advances."""
from convex_timevarying_window.wang_free_state_mpc.reference import Reference as BaseReference
from .contracts import comparison,TOLERANCE_S


class Reference(BaseReference):
    def __init__(self,start):
        super().__init__(start)
        self.contract_log=[]

    def candidate_reason(self,candidate,gate,phase):
        with self.lock:
            reason=super().candidate_reason(candidate,gate,phase)
            incumbent=self.plan_at(candidate.made_at)
            common=comparison(incumbent,candidate)
            if reason is None:
                delayed=next((r for r in common if r['delay_s']>TOLERANCE_S+1e-9),None)
                if delayed is not None: reason=f"delayed_shared_passage:{delayed['gate']}"
            self.contract_log.append(dict(candidate_made_at=candidate.made_at,
                incumbent_first_gate=incumbent.first_gate if incumbent is not None else None,
                candidate_first_gate=candidate.first_gate,common=common,reason=reason))
            return reason
