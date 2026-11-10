"""Same two-piece P/V-MINCO/time-only problem, circle map and pendulum query."""
from dataclasses import dataclass
import tempfile
import casadi as ca
import numpy as np
from convex_timevarying_window.comparisons.single_gate_swing.planner import Config as ParentConfig,Planner as Parent
from convex_timevarying_window.dynamic_passage_minco.planner import Stats
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import minco_coefficients
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from .motion import symbolic_pose,R0
from .safety import passage

@dataclass(frozen=True)
class Config(ParentConfig):
    frame_offset: float=.085

class Planner(Parent):
    def __init__(self,track,cfg=Config()):
        self.track,self.cfg=track,cfg;self.windows=list(track.windows);self.rotation=R0
        self.signs=[float(np.sign((track.start-self.windows[0].center0)@R0[:,2]))];self.admitted=None
        self._jit_dir=tempfile.TemporaryDirectory(prefix='pendulum_minco_');self._build();self.solvers=[(Stats(),)]
    def _build(self):
        cfg=self.cfg;w=self.windows[0];self.nvar=7
        x=ca.MX.sym('x',7);par=ca.MX.sym('par',13+2*cfg.witness_slots);times=ca.exp(x[:2]);cross=par[0]+times[0]
        uv=2*w.aperture.radius*x[2:4]/(1+ca.sumsqr(x[2:4]))
        c,R=symbolic_pose(w,cross);knot=c+R[:,:2]@uv
        C=minco_coefficients(ca.reshape(par[1:13],3,4),ca.horzcat(ca.DM(self.track.goal),ca.DM.zeros(3,3)),knot,x[4:7],times)
        def state(piece,u,derivative=0):return C[8*piece:8*piece+8,:].T@basis(u,derivative)/times[piece]**derivative
        constraints=self.extra_constraints(state,times,C)
        radius=cfg.radius+cfg.frame_thickness+cfg.passage_margin+cfg.witness_slack
        frame_radius=w.aperture.radius+cfg.frame_offset
        for piece in range(2):
            for j in range(cfg.witness_slots):
                offset=ca.fmin(par[13+piece*cfg.witness_slots+j],times[piece]);u=1-offset/times[0] if piece==0 else offset/times[1]
                t=cross-offset if piece==0 else cross+offset;cc,_=symbolic_pose(w,t);q=ca.DM(R0).T@(state(piece,u)-cc)
                rho=ca.sqrt(ca.sumsqr(q[:2])+1e-14)
                constraints.append(ca.fmax(frame_radius-rho,0.)**2+q[2]**2-radius**2)
        constraints.append(-self.signs[0]*ca.dot(R[:,2],state(0,1.,1))-.1)
        g=ca.vertcat(*constraints);cost=ca.sum1(times)/1000.
        options={'cse':True}
        if cfg.compile_objective:options.update(jit=True,compiler='shell',jit_options={'flags':[cfg.compiler_optimization],'directory':self._jit_dir.name+'/'})
        self.function=ca.Function('pendulum_minco',[x,par],[cost,ca.gradient(cost,x),g,ca.jacobian(g,x),C,ca.MX(0.)],options)
        self.lower=np.r_[np.log([.3,.3]),[-np.inf,-np.inf] if cfg.free_points else [0.,0.],np.full(3,-np.inf)]
        self.upper=np.r_[np.log([15.,15.]),[np.inf,np.inf] if cfg.free_points else [0.,0.],np.full(3,np.inf)]
    def local_passage(self,plan,window,piece):
        return passage(plan,window,self.cfg.radius+self.cfg.frame_thickness+self.cfg.passage_margin,self.cfg.frame_offset,piece)
