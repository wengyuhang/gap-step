"""Unchanged local MINCO mechanism with native GenericQuad actuator bounds."""
import casadi as ca
import numpy as np
from dataclasses import replace
import hashlib
from pathlib import Path
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from ..planner import Single as BaseSingle, Planner as BasePlanner, route_signs, parking_points
from .model import flat_map, GenericQuad


class Single(BaseSingle):
    def __init__(self,track,cfg,sign,model):
        self.native_model=model
        super().__init__(track,cfg,sign)

    def _build(self):
        cfg=self.cfg
        self.cfg=replace(cfg,compile_objective=False)
        self._build_expression()
        self.cfg=cfg
        if not cfg.compile_objective:return
        # Content-addressed, embedded native code: repeat development flights
        # need not recompile the same seven symbolic maps. Never cache plans.
        from .. import planner as parent,geometry,point_mapping
        from . import model
        signature=hashlib.sha256()
        for path in (Path(__file__),Path(parent.__file__),Path(geometry.__file__),
                     Path(point_mapping.__file__),Path(model.__file__)):
            signature.update(path.read_bytes())
        for path in getattr(self,'cache_sources',()):signature.update(Path(path).read_bytes())
        signature.update(repr((cfg,self.native_model,self.signs,self.track.goal.tolist(),
            self.windows[0].center0.tolist(),repr(self.windows[0].motion),
            repr(self.windows[0].aperture))).encode())
        cache=Path(__file__).parent/'runtime'/'planner_cache'
        cache.mkdir(parents=True,exist_ok=True)
        target=cache/(signature.hexdigest()+'.casadi')
        if target.exists():self.function=ca.Function.load(str(target));return
        original=self.function;inputs=[original.mx_in(i) for i in range(original.n_in())]
        self.function=ca.Function('single_stop_eval',inputs,original(*inputs),dict(cse=True,
            jit=True,jit_serialize='embed',compiler='shell',jit_options=dict(
                flags=[cfg.compiler_optimization],directory=self._jit_dir.name+'/')))
        self.function.save(str(target))

    def _build_expression(self):
        super()._build()

    def thrust_constraints(self,state,times):
        result=[];flat=flat_map(self.native_model)
        for piece in range(2):
            for u in np.linspace(0.,1.,self.cfg.nodes_per_piece+1):
                _,rates,pwm=flat(*[state(piece,float(u),d) for d in (1,2,3,4)])
                # Direct motor-PWM API bypasses SimpleFlight's rate setpoint
                # controller. An inherited 2.5 rad/s cap is not a native
                # physical limit; actuator allocation below bounds torque.
                result.extend(ca.vertsplit(pwm))
                result.extend(ca.vertsplit(1-pwm))
                pos=state(piece,float(u))
                result.extend(ca.vertsplit((pos-ca.DM(self.cfg.workspace_lower))/30))
                result.extend(ca.vertsplit((ca.DM(self.cfg.workspace_upper)-pos)/30))
        return result


class Planner(BasePlanner):
    def __init__(self,track,cfg,model=GenericQuad()):
        self.track,self.cfg=track,cfg;self.windows=list(track.windows);self.signs=route_signs(track)
        self.parking=parking_points(track,cfg);self.admitted=None;self.locals=[None]*len(self.windows)
        self.solvers=[]
        self.native_model=model

    def prepare(self,index):
        if self.locals[index] is None:
            w=self.windows[index]
            local=SCWindowTrack('native_'+w.name,self.track.start.copy(),self.parking[index],(w,),(0,))
            self.locals[index]=Single(local,self.cfg,self.signs[index],self.native_model)
        return self.locals[index]

    def solve(self,state,now,first):
        self.prepare(first)
        return super().solve(state,now,first)
