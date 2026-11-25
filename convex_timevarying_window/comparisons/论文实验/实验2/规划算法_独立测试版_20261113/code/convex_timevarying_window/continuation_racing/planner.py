"""Receding racing with a rest continuation and reference-consistent switching.

No gate index, motion multiplier, phase, or saved-run-dependent branches.
The terminal point is in the inter-gate leg, outside every frame's swept ball.
This is sufficient for a stationary nominal terminal point, not a tracking or
continuous trajectory certificate.
"""
import casadi as ca
import math
import numpy as np
from convex_timevarying_window.receding_gate_racing.planner import Planner as BasePlanner


class Planner(BasePlanner):
    discard_stale_seed = True
    warm_start_duals = True
    preserve_unexecuted_durations = True

    def seed_incumbent(self):
        return getattr(self,'admitted',None)

    def commit(self, plan):
        self.admitted=plan

    def swept_balls(self):
        for w in self.frames:
            extent=w.aperture.radius if w.aperture.kind=='circle' else np.linalg.norm(w.aperture.physical_vertices,axis=1).max()
            motion=np.linalg.norm(w.motion.translation_amplitude) if w.motion.translation_enabled else 0.
            yield np.asarray(w.center0),extent+motion+self.cfg.radius+self.cfg.frame_thickness+self.cfg.tracking_margin

    def horizon_ids(self, first):
        ids=super().horizon_ids(first)
        if ids[-1] < len(self.windows):ids.append(len(self.windows))
        return ids

    def terminal_position(self, first):
        last=min(first+self.cfg.horizon-1,len(self.windows))
        if last>=len(self.windows)-1:return np.asarray(self.track.goal)
        start=np.asarray(self.windows[last].center0);direction=self.windows[last+1].center0-start
        # Project the midpoint onto safe intervals on the inter-gate leg.
        # Each swept ball excludes a quadratic interval; no time sampling or
        # gate-specific offsets are involved in selecting a holdable point.
        intervals=[(.0,1.)];aa=float(direction@direction)
        if aa<1e-12:raise ValueError('Degenerate inter-gate leg has no terminal search direction')
        for center,radius in self.swept_balls():
            offset=start-center;bb=2*float(offset@direction);cc=float(offset@offset)-(radius+.05)**2
            disc=bb*bb-4*aa*cc
            if disc<=0:continue
            left=(-bb-np.sqrt(disc))/(2*aa);right=(-bb+np.sqrt(disc))/(2*aa)
            remaining=[]
            for lo,hi in intervals:
                if hi<=left or lo>=right:remaining.append((lo,hi))
                else:
                    if lo<left:remaining.append((lo,min(hi,left)))
                    if hi>right:remaining.append((max(lo,right),hi))
            intervals=remaining
        if self.cfg.workspace_lower is not None:
            for axis in range(3):
                lo=self.cfg.workspace_lower[axis]+self.workspace_inset()
                hi=self.cfg.workspace_upper[axis]-self.workspace_inset()
                if abs(direction[axis])<1e-12:
                    if not lo<=start[axis]<=hi:intervals=[]
                else:
                    a,b=sorted(((lo-start[axis])/direction[axis],(hi-start[axis])/direction[axis]))
                    intervals=[(max(l,a),min(h,b)) for l,h in intervals if max(l,a)<=min(h,b)]
        if not intervals:raise ValueError('No holdable terminal point on inter-gate leg; not a global infeasibility proof')
        fraction=min((np.clip(.5,lo,hi) for lo,hi in intervals),key=lambda s:abs(s-.5))
        return start+fraction*direction

    def workspace_inset(self):
        return self.cfg.tracking_margin

    def initial_duration(self, distance, k, wi):
        nominal=super().initial_duration(distance,k,wi)
        if wi==len(self.windows):
            return max(nominal,math.sqrt(6*distance/self.cfg.acceleration),
                       (60*distance/self.cfg.jerk)**(1/3))
        return nominal

    def tilt_expression(self, a, demanded, thrust):
        # Tilt is determined by the demanded thrust, not by a different
        # acceleration signal. Scale the zero-bound constraint by gravity^2.
        return (ca.sumsqr(demanded[:2])-(math.tan(math.radians(42))*thrust[2])**2)/9.8066**2

    def segment_frames(self, first, wi, k):
        if wi==len(self.windows):
            last=min(first+self.cfg.horizon-1,len(self.windows)-1)
            return range(max(0,last),min(len(self.windows),last+2))
        return super().segment_frames(first,wi,k)

    def terminal_constraints(self, con, p):
        for center,radius in self.swept_balls():
            con(ca.sumsqr(p-ca.DM(center)),radius**2,np.inf)

    def prefix_clearance(self, plan, now, duration=.30):
        clearance=super().prefix_clearance(plan,now,duration)
        if self.cfg.workspace_lower is not None:
            for t in np.arange(now,min(now+duration,plan.end_time)+1e-6,.01):
                p=plan.sample(t)
                clearance=min(clearance,float(np.min(p-np.asarray(self.cfg.workspace_lower))),
                              float(np.min(np.asarray(self.cfg.workspace_upper)-p)))
        return clearance
