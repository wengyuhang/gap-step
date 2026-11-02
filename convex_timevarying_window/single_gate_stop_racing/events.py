"""Fixed-plane crossing position queried against the gate at the event time."""
import numpy as np
from convex_timevarying_window.simple_two_gate_soft_only_mpc.execution import Progress as Parent


class Progress(Parent):
    def update(self,p0,p1,t0,t1):
        if self.gate>=len(self.windows):return False
        w=self.windows[self.gate];sign=self.signs[self.gate]
        _,z0=w.world_to_local(p0,t0);_,z1=w.world_to_local(p1,t1)
        a,b=sign*z0,sign*z1
        if not self.crossed and a>=0. and b<0.:
            alpha=abs(a)/(abs(a)+abs(b));at=float(t0+alpha*(t1-t0))
            # Interpolate only body motion, never the known nonlinear gate motion.
            pos=(1-alpha)*np.asarray(p0)+alpha*np.asarray(p1)
            q,_=w.world_to_local(pos,at)
            if not w.aperture.contains(q):
                self.violation='missed_current_opening';return False
            self.crossings.append(dict(gate=self.gate,time=at,q=q.tolist(),
                margin=float(w.aperture.boundary_distance(q)-self.radius)))
            self.crossed=True
        elif self.crossed and a<0. and b>=0.:
            self.violation='reversed_before_body_exit';self.crossed=False
        if self.crossed and b<=-(self.radius+self.thickness):
            self.exits.append(dict(gate=self.gate,time=float(t1),signed_distance=float(b)))
            self.gate+=1;self.crossed=False;return True
        return False
