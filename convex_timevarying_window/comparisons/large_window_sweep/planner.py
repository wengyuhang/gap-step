"""Same optimizer/objective; replace obsolete +/-2m box by aperture bounds."""
import numpy as np
from convex_timevarying_window.simple_two_gate_soft_only_mpc.planner import Planner as Current


def crossing_bounds(aperture,free_points):
    if not free_points:return np.zeros(2),np.zeros(2)
    if aperture.kind=='circle':return np.full(2,-aperture.radius),np.full(2,aperture.radius)
    return aperture.physical_vertices.min(axis=0),aperture.physical_vertices.max(axis=0)


class Planner(Current):
    def _build(self,first):
        solver,evaluate,lo,hi,lbg,ubg=super()._build(first)
        ids=self.ids(first);qi=self.offsets(len(ids))[3]
        lo=lo.copy();hi=hi.copy()
        for k,wi in enumerate(ids):
            lower,upper=crossing_bounds(self.windows[wi].aperture,self.cfg.free_points)
            lo[qi+2*k:qi+2*k+2]=lower;hi[qi+2*k:qi+2*k+2]=upper
        return solver,evaluate,lo,hi,lbg,ubg
