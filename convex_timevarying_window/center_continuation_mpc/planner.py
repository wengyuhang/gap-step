"""Condensed joint center-suffix approximation, not an exact base-policy rollout."""
from dataclasses import dataclass
import numpy as np
from convex_timevarying_window.race_to_go_mpc.planner import Planner as Parent,Config as ParentConfig
from convex_timevarying_window.wang_free_state_mpc.trajectory import arrival_jerk_for_middle


@dataclass(frozen=True)
class Config(ParentConfig):
    free_prefix_gates: int = 2
    seed_speed: float = 8.


class Planner(Parent):
    def _build(self,first):
        solver,ev,lo,hi,lbg,ubg=super()._build(first)
        gates=len(self.ids(first));qi=self.offsets(gates)[3]
        # Beyond the short prefix, only center crossings are allowed. The tail
        # state/times remain coupled to the prefix, so its cost is not constant.
        lo[qi+2*self.cfg.free_prefix_gates:qi+2*gates]=0.
        hi[qi+2*self.cfg.free_prefix_gates:qi+2*gates]=0.
        return solver,ev,lo,hi,lbg,ubg

    def initial(self,state,now,first):
        if self.admitted is not None or first in self.search:
            return super().initial(state,now,first)
        gates=len(self.ids(first));legs=gates+1;vi,ai,ji,qi=self.offsets(gates)
        x=np.zeros(4*legs+8*gates);head=np.asarray(state[:9]).reshape(3,3).copy();stamp=now
        for k in range(legs):
            terminal=k==gates
            target=self.track.goal if terminal else self.windows[first+k].state_at(stamp)[0]
            distance=np.linalg.norm(target-head[0])
            T=max(.8,distance/self.cfg.seed_speed,np.sqrt(3*distance/self.cfg.acceleration))
            if terminal:
                velocity=np.zeros(3)
            else:
                w=self.windows[first+k];target=w.state_at(stamp+T)[0]
                nxt=self.windows[first+k+1].center0 if first+k+1<len(self.windows) else self.track.goal
                direction=nxt-head[0]
                velocity=direction/max(np.linalg.norm(direction),1e-9)*(.45*self.cfg.speed)
            tail=np.array([target,velocity,np.zeros(3)])
            x[k]=np.log(T)
            if not terminal:x[vi+3*k:vi+3*k+3]=velocity/self.cfg.speed
            x[ji+3*k:ji+3*k+3]=arrival_jerk_for_middle(head,tail,.5*(head[0]+target),T)/self.cfg.jerk
            head=tail;stamp+=T
        return x
