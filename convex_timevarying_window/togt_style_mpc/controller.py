"""Independent acados SQP-RTI rotor NMPC, exported as collective/body rates.

Architecture follows TOGT's flight interface, not an exact author reproduction.
No gate geometry, reference retiming, PX4 internal states or safety filter.
"""
from dataclasses import dataclass, asdict
import hashlib
import fcntl
from pathlib import Path
import time
import casadi as ca
import numpy as np
from .model import (expressions, functions, quaternion_error, MASS, G, MIXER,
                    X500_INERTIA, KF, FMIN, FMAX, RATE_MAX)
from convex_timevarying_window.checked_soft_mpc_racing.mpc import reference_states


@dataclass(frozen=True)
class Config:
    steps: int = 25
    dt: float = .04
    update_period: float = .01
    position_weight: float = 2000.
    velocity_weight: float = 30.
    attitude_weight: float = 20.
    rate_weight: float = .3
    thrust_weight: float = .02
    # CTBR extraction from the first FUTURE shooting node, not the fixed x0.
    rate_preview_s: float = .04
    state_prediction_s: float = 0.


class MPC:
    def __init__(self, cfg=None):
        from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
        self.cfg = Config() if cfg is None else cfg
        cfg = self.cfg
        if cfg.steps < 2 or cfg.dt <= 0 or not 0 < cfg.rate_preview_s <= cfg.dt:
            raise ValueError('Require positive horizon and first-interval rate preview')
        self.rhs, self.step, self.aero = functions()
        self.controls = None; self.predicted = None; self.last_time = None; self.logs = []
        self.previous = np.array([MASS*G, 0., 0., 0.])
        self.mixer_inverse = np.linalg.inv(MIXER)
        x,u,rhs,_ = expressions()
        model = AcadosModel(); model.name = 'togt_style_x500'
        model.x, model.u = x,u
        model.xdot = ca.SX.sym('xdot',13)
        model.f_expl_expr = rhs; model.f_impl_expr = model.xdot-rhs
        model.p = ca.SX.sym('reference',17)
        error = ca.vertcat(x[:6]-model.p[:6], quaternion_error(x[6:10],model.p[6:10]),
                          x[10:]-model.p[10:13])
        model.cost_y_expr = ca.vertcat(error,u-model.p[13:])
        model.cost_y_expr_e = error
        ocp = AcadosOcp(); ocp.model = model
        ocp.parameter_values = np.r_[np.zeros(6),1.,np.zeros(6),np.full(4,MASS*G/4)]
        q = np.r_[np.full(3,cfg.position_weight),np.full(3,cfg.velocity_weight),
                  np.full(3,cfg.attitude_weight),np.full(3,cfg.rate_weight)]
        ocp.cost.cost_type = ocp.cost.cost_type_e = 'NONLINEAR_LS'
        ocp.cost.W = np.diag(np.r_[q,np.full(4,cfg.thrust_weight)])
        ocp.cost.W_e = np.diag(q)
        ocp.cost.yref = np.zeros(16); ocp.cost.yref_e = np.zeros(12)
        ocp.constraints.idxbu = np.arange(4)
        ocp.constraints.lbu = np.full(4,FMIN); ocp.constraints.ubu = np.full(4,FMAX)
        ocp.constraints.idxbx = ocp.constraints.idxbx_e = np.arange(10,13)
        ocp.constraints.lbx = ocp.constraints.lbx_e = -RATE_MAX.copy()
        ocp.constraints.ubx = ocp.constraints.ubx_e = RATE_MAX.copy()
        ocp.constraints.x0 = ocp.parameter_values[:13].copy()
        opts = ocp.solver_options
        opts.N_horizon = cfg.steps; opts.tf = cfg.steps*cfg.dt
        opts.nlp_solver_type = 'SQP_RTI'; opts.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        opts.qp_solver_cond_N = 5; opts.hessian_approx = 'GAUSS_NEWTON'
        opts.integrator_type = 'ERK'; opts.sim_method_num_stages = 4; opts.sim_method_num_steps = 4
        opts.qp_solver_iter_max = 50; opts.print_level = 0
        opts.levenberg_marquardt = 1e-6
        source = Path(__file__); model_source = source.with_name('model.py')
        signature = hashlib.sha256(source.read_bytes()+model_source.read_bytes()+repr(cfg).encode()
                                  +repr((MASS,X500_INERTIA.tolist(),MIXER.tolist())).encode()).hexdigest()[:16]
        build = source.parent/'native'/'build'/signature; build.mkdir(parents=True,exist_ok=True)
        ocp.code_export_directory = str(build/'code')
        # acados' generated Makefile may clean the .so on each construction.
        # Serialize first build and reuse it, so test/flight processes cannot
        # remove each other's library while loading the same model signature.
        with (build/'build.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            ready=(build/'code'/'libacados_ocp_solver_togt_style_x500.so').exists()
            self.solver = AcadosOcpSolver(ocp,json_file=str(build/'ocp.json'),verbose=False,
                                          build=not ready,generate=not ready)
        # Match the paper's discrete stage/terminal weights, avoiding an
        # accidental 1/dt multiplier on the terminal cost in acados defaults.
        for k in range(cfg.steps+1): self.solver.cost_set(k,'scaling',np.array([1.]))
        self.metadata = dict(implementation='independent_togt_style_acados_rotor_nmpc',
            author_source_used=False, model_states=13, input='four_rotor_thrust_N',
            output='collective_N_and_body_rates_FLU_rad_s', config=asdict(cfg),
            solver='acados_0.5.4_SQP_RTI_HPIPM', source_signature=signature,
            motor_lag_in_prediction=False, px4_inner_loop_in_prediction=False,
            gate_constraints=False, reference_retiming=False)

    def references(self, reference, now, initial_quaternion):
        cfg = self.cfg
        # Add two nodes for the angular-acceleration feedforward difference.
        times = now+np.arange(cfg.steps+3)*cfg.dt
        refs17, ctbr = reference_states(reference(times),cfg.dt,initial_quaternion)
        states = refs17[:cfg.steps+1,:13]
        alpha = np.gradient(refs17[:,10:13],cfg.dt,axis=0)[:cfg.steps+1]
        total = KF*np.sum(refs17[:cfg.steps+1,13:17]**2,axis=1)
        thrusts=[]
        for state,acc,collective in zip(states,alpha,total):
            omega=state[10:13]
            torque = X500_INERTIA*acc+np.cross(omega,X500_INERTIA*omega)
            initial = np.full(4,collective/4)
            torque -= np.asarray(self.aero(state,initial)).ravel()
            thrusts.append(self.mixer_inverse@np.r_[collective,torque])
        return states,np.asarray(thrusts)

    def command(self,state,now,reference):
        began=time.perf_counter(); cfg=self.cfg
        state=np.asarray(state,float)
        if state.shape not in ((13,),(17,)) or not np.isfinite(state).all():
            raise ValueError('Expected finite physical state[13] or telemetry[17]')
        x=state[:13].copy()
        norm=np.linalg.norm(x[6:10])
        if norm < .5: raise ValueError('Invalid measured quaternion')
        x[6:10]/=norm
        refs,ff=self.references(reference,now,x[6:10])
        # Shift BOTH states and controls: RTI is multiple shooting. Re-rolling
        # saturated open-loop controls from every new measurement discards the
        # optimized state seed and can move the linearization far off-course.
        # Defects of the shifted state seed are handled by the shooting QP.
        # The reference clock is never shifted.
        if self.controls is None or now < self.last_time:
            controls=np.clip(ff[:-1],FMIN,FMAX)
            seeds=refs.copy()
        else:
            offset=max(0.,(now-self.last_time)/cfg.dt); knots=np.arange(cfg.steps)
            controls=np.column_stack([np.interp(knots+offset,knots,self.controls[:,j]) for j in range(4)])
            state_knots=np.arange(cfg.steps+1)
            seeds=np.column_stack([np.interp(state_knots+offset,state_knots,self.predicted[:,j])
                                   for j in range(13)])
            seeds[:,6:10]/=np.linalg.norm(seeds[:,6:10],axis=1)[:,None]
        seeds[0]=x
        for k in range(cfg.steps):
            self.solver.set(k,'x',seeds[k]); self.solver.set(k,'u',controls[k])
            self.solver.set(k,'p',np.r_[refs[k],ff[k]])
        self.solver.set(cfg.steps,'x',seeds[-1])
        self.solver.set(cfg.steps,'p',np.r_[refs[-1],ff[-1]])
        self.solver.set(0,'lbx',x); self.solver.set(0,'ubx',x)
        status=int(self.solver.solve())
        if status != 0:
            self.logs.append(dict(time=float(now),cycle_ms=1000*(time.perf_counter()-began),
                solver_status=status, failure='rotor_nmpc_qp_failed'))
            raise RuntimeError(f'rotor NMPC failed, acados status={status}')
        controls=np.array([self.solver.get(k,'u') for k in range(cfg.steps)])
        predicted=np.array([self.solver.get(k,'x') for k in range(cfg.steps+1)])
        if not np.isfinite(controls).all() or not np.isfinite(predicted).all():
            raise RuntimeError('Nonfinite rotor NMPC result')
        if controls.min()<FMIN-1e-5 or controls.max()>FMAX+1e-5:
            raise RuntimeError('Rotor NMPC input bound residual exceeded')
        fraction=cfg.rate_preview_s/cfg.dt
        rate=(1-fraction)*predicted[0,10:13]+fraction*predicted[1,10:13]
        command=np.r_[controls[0].sum(),rate]
        self.controls=controls; self.predicted=predicted; self.last_time=now; self.previous=command
        self.logs.append(dict(time=float(now),cycle_ms=1000*(time.perf_counter()-began),
            controller=self.metadata['implementation'],solver_status=status,
            solver_ms=1000*float(self.solver.get_stats('time_tot')),
            rotor_thrust_N=controls[0].tolist(),command=command.tolist(),
            shooting_defect_max=float(max(np.max(np.abs(predicted[k+1]-np.asarray(
                self.step(predicted[k],controls[k],cfg.dt)).ravel())) for k in range(cfg.steps))),
            qp_iterations=np.asarray(self.solver.get_stats('qp_iter')).tolist(),
            position_error_m=float(np.linalg.norm(x[:3]-refs[0,:3])),
            rate_preview_s=cfg.rate_preview_s))
        self.logs[-1]['cycle_ms']=1000*(time.perf_counter()-began)
        return command.copy()
