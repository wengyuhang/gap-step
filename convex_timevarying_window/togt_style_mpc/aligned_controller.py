"""CTBR NMPC: optimize the input actually sent to PX4, with its execution chain.

Fixed reference clock. No gate logic, smoothing penalty, fallback, or retiming.
PX4 hidden memory is propagated using measured states and sent-command history.
"""
from dataclasses import dataclass, asdict
from pathlib import Path
import fcntl
import hashlib
import time
import casadi as ca
import numpy as np
from .controller import MPC as RotorMPC, Config as RotorConfig
from .model import quaternion_error, functions, MASS, G, KF, FMIN, FMAX, RATE_MAX, MIXER
from convex_timevarying_window.single_gate_stop_racing import px4_model


@dataclass(frozen=True)
class Config(RotorConfig):
    # Accept each fresh 8/12 ms measurement; don't discard 8 ms updates because
    # a nominal 10 ms period hasn't elapsed. Transport still runs at 100 Hz.
    update_period: float = .004
    rate_preview_s: float = 0.
    state_prediction_s: float = 0.
    solver_type: str = 'SQP_RTI'
    sqp_max_iterations: int = 15
    control_weight: float = 0.


def next_control_tick(when):
    """A packet sent after this simulation stamp reaches the next PID tick."""
    index=np.floor(float(when)/px4_model.DT+1e-9)
    return float((index+1)*px4_model.DT)


class MPC:
    references = RotorMPC.references

    def __init__(self, cfg=None):
        from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
        self.cfg=Config() if cfg is None else cfg; cfg=self.cfg
        if cfg.dt != .04: raise ValueError('PX4 execution nodes require ten 4 ms ticks')
        self.controls=None; self.predicted=None; self.last_time=None; self.last_execution_time=None; self.logs=[]
        self.previous=np.array([MASS*G,0.,0.,0.]); self.lower=np.r_[4*FMIN,-RATE_MAX]; self.upper=np.r_[4*FMAX,RATE_MAX]
        self.tick,self.step,_,_=px4_model.native_dynamics()
        self.rhs,_,self.aero=functions(); self.mixer_inverse=np.linalg.inv(MIXER)
        self.observer=None; self.observer_time=None; self.observer_physical=None
        self.deliveries=[]; self.session=None; self.compute_sim_s=0.
        _,symbolic_step,_,_=px4_model.dynamics()
        x,u=ca.MX.sym('x',33),ca.MX.sym('u',4)
        model=AcadosModel(); model.name='px4_aligned_ctbr'
        model.x=x; model.u=u; model.p=ca.MX.sym('reference',17)
        model.disc_dyn_expr=symbolic_step(x,u,cfg.dt)
        error=ca.vertcat(x[:6]-model.p[:6],quaternion_error(x[6:10],model.p[6:10]),x[10:13]-model.p[10:13])
        # Keep the original physical meaning of the .02 rotor-force cost.
        # Rotor force is now an actuator STATE, not a fictitious direct input.
        control_reference=ca.vertcat(ca.sum1(model.p[13:17]),model.p[10:13])
        input_error=(u-control_reference)/ca.DM(np.r_[4*FMAX,RATE_MAX])
        model.cost_y_expr=ca.vertcat(error,KF*x[13:17]**2-model.p[13:17],input_error)
        model.cost_y_expr_e=error
        ocp=AcadosOcp(); ocp.model=model
        ocp.parameter_values=np.r_[np.zeros(6),1.,np.zeros(6),np.full(4,MASS*G/4)]
        weights=np.r_[np.full(3,cfg.position_weight),np.full(3,cfg.velocity_weight),np.full(3,cfg.attitude_weight),np.full(3,cfg.rate_weight)]
        ocp.cost.cost_type=ocp.cost.cost_type_e='NONLINEAR_LS'
        ocp.cost.W=np.diag(np.r_[weights,np.full(4,cfg.thrust_weight),np.full(4,cfg.control_weight)]); ocp.cost.W_e=np.diag(weights)
        ocp.cost.yref=np.zeros(20); ocp.cost.yref_e=np.zeros(12)
        ocp.constraints.idxbu=np.arange(4); ocp.constraints.lbu=self.lower; ocp.constraints.ubu=self.upper
        ocp.constraints.idxbx=ocp.constraints.idxbx_e=np.arange(10,13)
        ocp.constraints.lbx=ocp.constraints.lbx_e=-RATE_MAX; ocp.constraints.ubx=ocp.constraints.ubx_e=RATE_MAX
        from convex_timevarying_window.checked_soft_mpc_racing.mpc import hover_state
        ocp.constraints.x0=px4_model.initial_state(hover_state())
        opts=ocp.solver_options; opts.N_horizon=cfg.steps; opts.tf=cfg.steps*cfg.dt
        opts.nlp_solver_type=cfg.solver_type; opts.integrator_type='DISCRETE'; opts.hessian_approx='GAUSS_NEWTON'
        if cfg.solver_type=='SQP':
            opts.globalization='MERIT_BACKTRACKING'
            opts.nlp_solver_max_iter=cfg.sqp_max_iterations
            opts.nlp_solver_tol_stat=opts.nlp_solver_tol_eq=opts.nlp_solver_tol_ineq=opts.nlp_solver_tol_comp=1e-3
        opts.qp_solver='PARTIAL_CONDENSING_HPIPM'; opts.qp_solver_cond_N=5
        opts.qp_solver_iter_max=50; opts.levenberg_marquardt=1e-6; opts.print_level=0
        paths=[Path(__file__),Path(px4_model.__file__),Path(__file__).with_name('controller.py'),Path(__file__).with_name('model.py')]
        signature=hashlib.sha256(b''.join(p.read_bytes() for p in paths)+repr(cfg).encode()).hexdigest()[:16]
        build=Path(__file__).parent/'native'/'build'/signature; build.mkdir(parents=True,exist_ok=True)
        ocp.code_export_directory=str(build/'code')
        with (build/'build.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            ready=(build/'code'/'libacados_ocp_solver_px4_aligned_ctbr.so').exists()
            self.solver=AcadosOcpSolver(ocp,json_file=str(build/'ocp.json'),verbose=False,generate=not ready,build=not ready)
        for k in range(cfg.steps+1): self.solver.cost_set(k,'scaling',np.array([1.]))
        self.metadata=dict(implementation='execution_aligned_ctbr_acados_nmpc',author_source_used=False,
            model_states=33,input='collective_N_and_body_rates_FLU_rad_s',output='same_CTBR_input',
            motor_lag_in_prediction=True,px4_inner_loop_in_prediction=True,
            config=asdict(cfg),source_signature=signature,reference_retiming=False,
            gate_constraints=False,derivative_order='gyro_lowpass_then_derivative_then_D_lowpass',solver_type=cfg.solver_type)

    def bind_session(self, session):
        self.session=session
        self.observer=None; self.observer_time=None; self.observer_physical=None; self.deliveries=[]
        session.command_sent=self.command_sent

    def command_sent(self, command, sim_time):
        # Physics can run at 1 ms while the PX4 gyro/PID clock remains 4 ms.
        # Adding 4 ms to an off-grid send stamp invents a nonexistent PID tick.
        apply_time=next_control_tick(sim_time)
        self.deliveries.append((apply_time,np.asarray(command).copy()))
        if self.logs:
            self.logs[-1].update(send_sim_time_s=float(sim_time),
                predicted_apply_time_error_s=float(apply_time-self.logs[-1]['execution_time_s']),
                predicted_first_pid_tick_s=apply_time)

    def applied_at(self, when):
        values=[u for t,u in self.deliveries if t<=when+1e-9]
        return values[-1] if values else np.array([MASS*G,0.,0.,0.])

    def propagate(self, x, begin, end):
        # Digital filters are always advanced by 4 ms, never re-clocked with
        # an arbitrary telemetry interval or with a 40 ms prediction node.
        result=x.copy()
        count=max(0,int(round((end-begin)/px4_model.DT)))
        for k in range(count):
            t=begin+k*px4_model.DT
            result=np.asarray(self.tick(result,self.applied_at(t),px4_model.DT)).ravel()
        return result

    def observe(self, state, now):
        if self.observer is None or now<self.observer_time:
            self.observer=px4_model.initial_state(state); self.observer_time=now
        else:
            old=self.observer_physical; begin=self.observer_time
            count=max(0,int(round((now-begin)/px4_model.DT)))
            for k in range(count):
                # Condition the controller-memory observer on measured motion;
                # don't integrate a drifting open-loop gyro over the flight.
                fraction=k/max(count,1)
                measured=old+fraction*(state-old)
                if np.dot(old[6:10],state[6:10])<0:
                    measured[6:10]=(1-fraction)*old[6:10]-fraction*state[6:10]
                measured[6:10]/=np.linalg.norm(measured[6:10])
                self.observer[:17]=measured
                self.observer=np.asarray(self.tick(self.observer,self.applied_at(begin+k*px4_model.DT),px4_model.DT)).ravel()
        self.observer[:17]=state; self.observer_time=now; self.observer_physical=state.copy()
        older=[entry for entry in self.deliveries if entry[0]<=now-.2]
        self.deliveries=older[-1:]+[entry for entry in self.deliveries if entry[0]>now-.2]
        return self.observer.copy()

    def command(self,state,now,reference,*,execution_time=None):
        began=time.perf_counter(); cfg=self.cfg; n=cfg.steps
        state=np.asarray(state,float)
        if state.shape not in ((17,),(33,)) or not np.isfinite(state).all() or np.linalg.norm(state[6:10])<.5:
            raise ValueError('Expected finite telemetry[17] or full model state[33]')
        state=state.copy(); state[6:10]/=np.linalg.norm(state[6:10])
        sim_start=now if self.session is None else float(self.session.clock.value)-30.
        if execution_time is None:
            execution_time=now if self.session is None else max(now,next_control_tick(sim_start+self.compute_sim_s))
        execution_time=now+max(0,int(round((execution_time-now)/px4_model.DT)))*px4_model.DT
        x=state if len(state)==33 else self.observe(state,now)
        x=self.propagate(x,now,execution_time)
        refs,ff=self.references(reference,execution_time,x[6:10])
        if self.controls is None or self.last_execution_time is None or execution_time<self.last_execution_time:
            controls=np.column_stack([ff[:-1].sum(axis=1),refs[1:,10:13]])
            controls=np.clip(controls,self.lower,self.upper)
            seeds=[x]
            for u in controls: seeds.append(np.asarray(self.step(seeds[-1],u,cfg.dt)).ravel())
            seeds=np.array(seeds)
        else:
            offset=(execution_time-self.last_execution_time)/cfg.dt
            controls=np.column_stack([np.interp(np.arange(n)+offset,np.arange(n),self.controls[:,j]) for j in range(4)])
            seeds=np.column_stack([np.interp(np.arange(n+1)+offset,np.arange(n+1),self.predicted[:,j]) for j in range(33)])
        seeds[0]=x
        for k in range(n+1):
            self.solver.set(k,'x',seeds[k]); self.solver.set(k,'p',np.r_[refs[k],ff[k]])
            if k<n: self.solver.set(k,'u',controls[k])
        self.solver.constraints_set(0,'lbx',x); self.solver.constraints_set(0,'ubx',x)
        status=self.solver.solve()
        if status: raise RuntimeError(f'aligned_ctbr_acados_status={status}')
        self.controls=np.array([self.solver.get(k,'u') for k in range(n)])
        self.predicted=np.array([self.solver.get(k,'x') for k in range(n+1)])
        raw_command=self.controls[0].copy()
        scale=np.r_[4*FMAX,RATE_MAX]
        violation=float(np.max(np.maximum(self.lower-raw_command,raw_command-self.upper)/scale))
        if not np.isfinite(raw_command).all() or violation>1e-3:
            raise RuntimeError(f'Invalid aligned CTBR solver output: command={raw_command.tolist()}, normalized_bound_violation={violation}')
        # Enforce the physical box exactly for tiny QP feasibility residuals.
        # This never changes the reference or accepts a large/invalid output.
        command=np.clip(raw_command,self.lower,self.upper)
        self.controls[0]=command
        self.previous=command; self.last_time=now; self.last_execution_time=execution_time
        if self.session is not None:
            self.compute_sim_s=max(0.,float(self.session.clock.value)-30.-sim_start)
        self.logs.append(dict(time=float(now),execution_time_s=float(execution_time),
            state_age_at_compute_start_s=float(sim_start-now),compute_sim_s=self.compute_sim_s,
            cycle_ms=1000*(time.perf_counter()-began),solver_status=int(status),
            controller='execution_aligned_ctbr_acados_nmpc',command=command.tolist(),
            raw_command=raw_command.tolist(),normalized_input_bound_violation=violation,
            input_projection_delta=float(np.linalg.norm(command-raw_command)),
            position_error_m=float(np.linalg.norm(state[:3]-reference([now])[0,:3])),
            state_prediction_s=float(execution_time-now),estimated_rate_integral=x[17:20].tolist()))
        return command.copy()
