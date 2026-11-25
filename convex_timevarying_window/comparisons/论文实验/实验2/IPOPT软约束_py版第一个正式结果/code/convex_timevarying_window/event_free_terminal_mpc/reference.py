"""Free local endpoints are not hover setpoints after reference exhaustion."""
import numpy as np
from convex_timevarying_window.wang_progress_mpc.reference import Reference as BaseReference


class Reference(BaseReference):
    def __init__(self,start,gate_count):
        super().__init__(start)
        self.total_gates=gate_count

    def state_at(self,t,orders=4):
        with self.lock:
            plan=self.plan_at(t)
            if plan is None:return np.r_[self.start,np.zeros(3*(orders-1))],None
            local=plan.first_gate+plan.gate_count<self.total_gates
            if local and t>plan.end_time+1e-9:
                raise RuntimeError('free_terminal_reference_exhausted: no fabricated hover tail')
            return np.concatenate([plan.sample(t,d) if local or d==0 or t<plan.end_time
                                   else np.zeros(3) for d in range(orders)]),plan
