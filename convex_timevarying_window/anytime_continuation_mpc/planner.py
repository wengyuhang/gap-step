from dataclasses import replace
import time
import numpy as np
from convex_timevarying_window.center_continuation_mpc.planner import Planner as Parent,Config
from convex_timevarying_window.event_free_terminal_mpc.planner import Planner as Local
from convex_timevarying_window.wang_free_state_mpc.trajectory import arrival_jerk_for_middle


class Planner(Parent):
    def __init__(self,track,cfg=Config(),*,build_indices=None):
        super().__init__(track,cfg,build_indices=build_indices)
        self.bootstrap=Local(track,replace(cfg,horizon=2,free_points=False,solver_seconds=.055),build_indices=[0])

    def solve(self,state,now,first):
        if self.admitted is None and first==0:
            plan=self.bootstrap.solve(state,now,first)
            plan.is_bootstrap=True;plan.nominal_finish_s=None
            plan.race_selection=dict(selected='center',center_valid=False,free_valid=False,
                center_finish_s=None,free_finish_s=None,center_reason='local_bootstrap_no_full_race_score',
                bootstrap_valid=bool(plan.event_valid),free_reason='not_run')
            return plan
        plan=super().solve(state,now,first);plan.is_bootstrap=False
        return plan

    def initial(self,state,now,first):
        old=self.admitted
        if old is None or not getattr(old,'is_bootstrap',False):return super().initial(state,now,first)
        # The short bootstrap has fewer gates than the center suffix. Never
        # index it as if it already contained a full-course trajectory.
        self.admitted=None
        try:x=super().initial(state,now,first)
        finally:self.admitted=old
        if first in self.search:return x
        gates=len(self.ids(first));vi,ai,ji,qi=self.offsets(gates)
        head=np.asarray(state[:9]).reshape(3,3);stamp=now
        for k,wi in enumerate(self.ids(first)):
            # Warm only the imminent crossing. The bootstrap's unexecuted
            # second-gate timing is not a cost-to-go estimate or a deadline.
            if k>=1:break
            old_k=wi-old.first_gate
            if not 0<=old_k<old.gate_count:break
            end=float(old.crossing_times[old_k]);T=end-stamp
            if T<self.cfg.min_leg_seconds:break
            target=self.windows[wi].state_at(end)[0]
            velocity=old.sample(end,1);acceleration=old.sample(end,2)
            tail=np.array([target,velocity,acceleration])
            x[k]=np.log(T);x[vi+3*k:vi+3*k+3]=velocity/self.cfg.speed
            x[ai+3*k:ai+3*k+3]=acceleration/self.cfg.acceleration
            x[ji+3*k:ji+3*k+3]=arrival_jerk_for_middle(head,tail,old.sample(stamp+.5*T),T)/self.cfg.jerk
            x[qi+2*k:qi+2*k+2]=0.
            head=tail;stamp=end
        return x
