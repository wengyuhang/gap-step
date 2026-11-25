"""Observed ordered gate events, independent of TOGT and predicted events."""
import numpy as np

class Progress:
    def __init__(self,track,cfg,signs,finite_entry_only=False):
        self.track,self.cfg,self.signs=track,cfg,signs
        self.finite_entry_only=finite_entry_only
        self.first=0;self.phase=0;self.violation=None;self.crossings=[];self.centers=[]

    def update(self,p0,t0,p1,t1):
        if self.first>=len(self.track.order):return False
        w=self.track.windows[self.track.order[self.first]];sign=self.signs[self.first]
        q0,z0=w.world_to_local(p0,t0);q1,z1=w.world_to_local(p1,t1)
        a,b=sign*z0,sign*z1
        slab=self.cfg.radius+self.cfg.error_margin+self.cfg.frame_thickness
        # A rotating gate's infinite supporting plane may sweep across the
        # vehicle while its finite opening is many metres away.  That is not
        # an aperture-entry event and must not commit the passage state.
        if self.phase==0 and a<=-slab<b:
            if w.aperture.contains(q1):self.phase=1
            else:
                # A vehicle crossing the old, fixed plane outside the opening
                # is a bypass.  A distant moving plane sweeping over the
                # vehicle is not, and does not start a passage commitment.
                _,frozen_z1=w.world_to_local(p1,t0)
                radius=w.aperture.radius if w.aperture.kind=='circle' else np.linalg.norm(w.aperture.physical_vertices,axis=1).max()
                nearby=np.linalg.norm(q1)<=radius+slab
                if sign*frozen_z1>-slab and (nearby or not self.finite_entry_only):self.violation='outside_aperture'
        if self.phase:
            if not w.aperture.contains(q1):self.violation='outside_aperture'
            if b<=-slab or (self.phase==2 and b<0):self.violation='forbidden_retreat'
            if a<=0<b:
                f=-a/(b-a);tc=t0+f*(t1-t0);p=(1-f)*np.asarray(p0)+f*np.asarray(p1)
                q,z=w.world_to_local(p,tc)
                self.centers.append(dict(gate=self.first,time=tc,position=p.tolist(),local_point=q.tolist(),plane_residual=z))
                self.phase=2
            if self.phase==2 and b>slab and self.violation is None:
                self.crossings.append(dict(gate=self.first,time=t1,position=np.asarray(p1).tolist(),local_point=q1.tolist()))
                self.first+=1;self.phase=0;return True
        return False
