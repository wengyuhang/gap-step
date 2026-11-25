"""Trajectory-only single-shooting CTBR MPC with the same execution model.

All predicted states are obtained by nonlinear integration of the optimized
inputs. No independent shooting states, gate terms, path retiming or fallback.
"""
from dataclasses import dataclass,asdict
import hashlib
from pathlib import Path
import time
import casadi as ca
import numpy as np
from scipy.optimize import minimize
from .aligned_controller import MPC as Parent,Config as ParentConfig,next_control_tick
from .model import MASS,G,KF,FMIN,FMAX,RATE_MAX,quaternion_error
from convex_timevarying_window.single_gate_stop_racing import px4_model


@dataclass(frozen=True)
class Config(ParentConfig):
    solver_type: str = 'L-BFGS-B'
    control_weight: float = 1.
    max_solve_ms: float = 60.
    max_iterations: int = 30


class BudgetExpired(Exception):pass


class MPC(Parent):
    def __init__(self,cfg=None):
        self.cfg=Config() if cfg is None else cfg;cfg=self.cfg;n=cfg.steps
        self.controls=None;self.predicted=None;self.last_time=None;self.last_execution_time=None;self.logs=[]
        self.previous=np.array([MASS*G,0.,0.,0.]);self.lower=np.r_[4*FMIN,-RATE_MAX];self.upper=np.r_[4*FMAX,RATE_MAX]
        self.observer=None;self.observer_time=None;self.observer_physical=None
        self.deliveries=[];self.session=None;self.compute_sim_s=0.
        from .model import functions,MIXER
        self.rhs,_,self.aero=functions();self.mixer_inverse=np.linalg.inv(MIXER)
        _,step,_,_=px4_model.dynamics()
        u=ca.MX.sym('u',4,n);x=ca.MX.sym('initial',33);p=ca.MX.sym('reference',17,n+1)
        states=ca.horzcat(x,step.mapaccum(n)(x,u,cfg.dt))
        weights=ca.DM(np.r_[np.full(3,cfg.position_weight),np.full(3,cfg.velocity_weight),
            np.full(3,cfg.attitude_weight),np.full(3,cfg.rate_weight)])
        scale=ca.DM(np.r_[4*FMAX,RATE_MAX]);cost=0.
        for k in range(n+1):
            error=ca.vertcat(states[:6,k]-p[:6,k],quaternion_error(states[6:10,k],p[6:10,k]),states[10:13,k]-p[10:13,k])
            cost+=ca.dot(weights,error**2)
            if k<n:
                force_error=KF*states[13:17,k]**2-p[13:17,k]
                control_ref=ca.vertcat(ca.sum1(p[13:17,k]),p[10:13,k])
                cost+=cfg.thrust_weight*ca.sumsqr(force_error)+cfg.control_weight*ca.sumsqr((u[:,k]-control_ref)/scale)
        objective=ca.Function('shooting_tracking_cost',[u,x,p],[cost/1000.,ca.gradient(cost/1000.,u)])
        rollout=ca.Function('shooting_tracking_rollout',[u,x],[states])
        source=Path(__file__)
        signature=hashlib.sha256(source.read_bytes()+Path(px4_model.__file__).read_bytes()+repr(cfg).encode()).hexdigest()
        self.tick,self.step,_,_,self.objective,self.rollout=px4_model.native_dynamics((objective,rollout),signature)
        self.uscale=np.r_[4*FMAX,RATE_MAX]
        self.metadata=dict(implementation='execution_aligned_single_shooting_ctbr_mpc',model_states=33,
            input='collective_N_and_body_rates_FLU_rad_s',output='same_CTBR_input',solver='L-BFGS-B',
            prediction_states='nonlinear rollout; no multiple-shooting equality defects',
            physical_input_box=True,body_rate_state_hard_constraint=False,reference_retiming=False,
            gate_constraints=False,source_signature=signature[:16],config=asdict(cfg))

    def command(self,state,now,reference,*,execution_time=None):
        began=time.perf_counter();cfg=self.cfg;n=cfg.steps
        state=np.asarray(state,float)
        if state.shape not in ((17,),(33,)) or not np.isfinite(state).all():raise ValueError('Expected finite state')
        state=state.copy();norm=np.linalg.norm(state[6:10])
        if norm<.5:raise ValueError('Invalid measured quaternion')
        state[6:10]/=norm
        sim_start=now if self.session is None else float(self.session.clock.value)-30.
        if execution_time is None:execution_time=now if self.session is None else max(now,next_control_tick(sim_start+self.compute_sim_s))
        execution_time=now+max(0,int(round((execution_time-now)/px4_model.DT)))*px4_model.DT
        x=state if len(state)==33 else self.observe(state,now)
        x=self.propagate(x,now,execution_time)
        refs,ff=self.references(reference,execution_time,x[6:10]);parameters=np.c_[refs,ff].T
        if self.controls is None or execution_time<self.last_execution_time:
            controls=np.c_[ff[:-1].sum(axis=1),refs[:-1,10:13]]
        else:
            offset=(execution_time-self.last_execution_time)/cfg.dt
            controls=np.column_stack([np.interp(np.arange(n)+offset,np.arange(n),self.controls[:,i]) for i in range(4)])
        controls=np.clip(controls,self.lower,self.upper)
        best=controls.copy();best_cost=np.inf;calls=0
        def objective(vector):
            nonlocal best,best_cost,calls
            if calls and 1000*(time.perf_counter()-began)>=cfg.max_solve_ms:raise BudgetExpired()
            trial=vector.reshape(n,4)*self.uscale
            value,gradient=self.objective(trial.T,x,parameters);value=float(value);calls+=1
            if np.isfinite(value) and value<best_cost:best_cost=value;best=trial.copy()
            return value,(np.asarray(gradient).T*self.uscale).ravel()
        initial=objective((controls/self.uscale).ravel())[0]
        try:
            solved=minimize(objective,(controls/self.uscale).ravel(),method='L-BFGS-B',jac=True,
                bounds=list(zip(np.tile(self.lower/self.uscale,n),np.tile(self.upper/self.uscale,n))),
                options=dict(maxiter=cfg.max_iterations,maxls=12,maxcor=10,ftol=1e-8,gtol=1e-5))
            status=str(solved.message);converged=bool(solved.success)
        except BudgetExpired:status='wall_budget_feasible_iterate';converged=False
        if not np.isfinite(best_cost):raise RuntimeError('No finite MPC objective')
        self.controls=np.clip(best,self.lower,self.upper);self.predicted=np.asarray(self.rollout(self.controls.T,x)).T
        command=self.controls[0].copy();self.previous=command;self.last_time=now;self.last_execution_time=execution_time
        if self.session is not None:self.compute_sim_s=max(0.,float(self.session.clock.value)-30.-sim_start)
        self.logs.append(dict(time=float(now),execution_time_s=float(execution_time),state_age_at_compute_start_s=float(sim_start-now),
            compute_sim_s=self.compute_sim_s,cycle_ms=1000*(time.perf_counter()-began),solver_status=0,
            optimization_status=status,converged=converged,
            evaluations=calls,initial_cost=1000*initial,final_cost=1000*best_cost,
            controller=self.metadata['implementation'],command=command.tolist(),state_prediction_s=float(execution_time-now),
            estimated_rate_integral=x[17:20].tolist(),position_error_m=float(np.linalg.norm(state[:3]-reference([now])[0,:3]))))
        return command.copy()
