"""Offline, controlled layer isolation; never used by a flight entry point.

Same frozen reference, controller, feedback clock and initial condition. The
only intervention is the plant/actuation layer. Numerical evidence, not Gazebo.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import casadi as ca
import numpy as np
from .controller import MPC, Config
from .model import expressions, KF, FMIN, FMAX
from convex_timevarying_window.checked_soft_mpc_racing.mpc import hover_state
from convex_timevarying_window.single_gate_stop_racing.px4_model import initial_state, native_dynamics, DT
from convex_timevarying_window.global_gate_minco_probe.phase_descent import load_plan
from convex_timevarying_window.global_gate_minco_probe.flight import Reference


def motor_plant():
    x, u, rhs, _ = expressions()
    physical = ca.Function('physical', [x, u], [rhs])
    z, target, h = ca.SX.sym('z', 17), ca.SX.sym('target', 4), ca.SX.sym('h')
    speed = ca.sqrt(target/KF)
    dz = ca.vertcat(physical(z[:13], KF*z[13:]**2),
                   (speed-z[13:])/ca.if_else(speed > z[13:], .0125, .025))
    f = ca.Function('motor_rhs', [z, target], [dz])
    k1=f(z,target); k2=f(z+h*k1/2,target); k3=f(z+h*k2/2,target); k4=f(z+h*k3,target)
    y=z+h*(k1+2*k2+2*k3+k4)/6
    y=ca.vertcat(y[:6], y[6:10]/ca.norm_2(y[6:10]), y[10:])
    return ca.Function('motor_tick', [z,target,h], [y])


def run(kind, ref, duration, outdir, rti_iterations=1, control_period=.01, actuation_delay=0., filtered_rate_feedback=False, filtered_d_term=False, control_times=None):
    if filtered_rate_feedback and kind != 'px4_ctbr':
        raise ValueError('filtered feedback diagnostic requires the PX4 plant')
    from scipy.signal import butter
    bb, aa = butter(2, 40., fs=1./DT)
    mpc=MPC(replace(Config(),update_period=control_period))
    if rti_iterations > 1:
        original_solve=mpc.solver.solve
        def repeated_solve():
            for _ in range(rti_iterations):
                status=original_solve()
                if status: return status
            return status
        mpc.solver.solve=repeated_solve
    physical=hover_state(ref([0.])[0,:3])
    if kind=='ideal_rotors': x=physical[:13]; tick=mpc.step
    elif kind=='motor_lag': x=physical; tick=motor_plant()
    else: x=initial_state(physical); tick=native_dynamics()[0]
    if filtered_d_term:
        if kind != 'px4_ctbr': raise ValueError('D-term diagnostic requires PX4 plant')
        # Compatibility flag: the production plant now implements this order.
        # Historical raw-D results retain their original model source hashes.
    rows=[]; details=[]; next_control=0. if control_times is None else float(control_times[0]); failure=None; t=0.; pending=[]; control_index=0
    applied=mpc.previous.copy() if kind=='px4_ctbr' else np.full(4,np.sum(KF*physical[13:]**2)/4)
    try:
        for k in range(round(duration/DT)):
            t=k*DT
            if t >= next_control-1e-9:
                observed=x[:13].copy()
                if filtered_rate_feedback:
                    delay0=x[10:13]-aa[1]*x[20:23]-aa[2]*x[23:26]
                    observed[10:13]=bb[0]*delay0+bb[1]*x[20:23]+bb[2]*x[23:26]
                command=mpc.command(observed, t, ref)
                control=command if kind=='px4_ctbr' else mpc.controls[0].copy()
                pending.append((t+actuation_delay,control.copy()))
                # Match the OCP's RK4, four substeps; unlike the production
                # diagnostic's single RK4, this isolates actual RTI defects.
                predicted=observed.copy()
                for _ in range(4):
                    predicted=np.asarray(mpc.step(predicted,mpc.controls[0],.01)).ravel()
                dp=mpc.predicted[1]-predicted
                details.append(dict(time=t, first_defect_position_m=float(np.linalg.norm(dp[:3])),
                    first_defect_rate_rad_s=float(np.linalg.norm(dp[10:])),
                    rotor=mpc.controls[0].tolist(), command=command.tolist(),
                    true_rate=x[10:13].tolist(), observed_rate=observed[10:13].tolist()))
                control_index+=1
                if control_times is not None and control_index < len(control_times):
                    next_control=float(control_times[control_index])
                else:
                    next_control+=mpc.cfg.update_period
            error=float(np.linalg.norm(x[:3]-ref([t])[0,:3]))
            while pending and pending[0][0]<=t+1e-9:
                _,applied=pending.pop(0)
            rows.append([t,*x[:13],*applied,error])
            x=np.asarray(tick(x,applied,DT)).ravel()
            if not np.isfinite(x).all() or np.linalg.norm(x[:3]-ref([t+DT])[0,:3])>3:
                raise RuntimeError('diagnostic_stop_error_over_3m')
    except Exception as exc:
        failure=repr(exc)
    rows=np.asarray(rows)
    result=dict(layer=kind, rti_iterations=rti_iterations, completed=failure is None, failure=failure, stop_s=t+DT,
        control_period_s=control_period,actuation_delay_s=actuation_delay,
        filtered_rate_feedback=filtered_rate_feedback,
        filtered_d_term=True,
        recorded_control_times=control_times is not None,
        reference_duration_s=duration-1., position_error_max_m=float(rows[:,-1].max()),
        position_error_rms_m=float(np.sqrt(np.mean(rows[:,-1]**2))),
        first_error_over_half_m_s=next((float(r[0]) for r in rows if r[-1]>.5),None),
        first_error_over_one_m_s=next((float(r[0]) for r in rows if r[-1]>1.),None),
        solver_failures=sum(r['solver_status']!=0 for r in mpc.logs),
        max_first_position_defect_m=max(r['first_defect_position_m'] for r in details),
        max_first_rate_defect_rad_s=max(r['first_defect_rate_rad_s'] for r in details),
        evidence='numerical isolation only; no collision or Gazebo claim',
        controller=mpc.metadata)
    np.savez_compressed(outdir/(kind+'.npz'),rows=rows)
    (outdir/(kind+'_details.json')).write_text(json.dumps(details))
    (outdir/(kind+'_mpc.json')).write_text(json.dumps(mpc.logs))
    (outdir/(kind+'_result.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--outdir',required=True,type=Path)
    p.add_argument('--plan-dir',required=True,type=Path)
    p.add_argument('--layers',nargs='+',default=['ideal_rotors','motor_lag','px4_ctbr'],
                   choices=['ideal_rotors','motor_lag','px4_ctbr'])
    p.add_argument('--rti-iterations',type=int,default=1)
    p.add_argument('--control-period',type=float,default=.01)
    p.add_argument('--actuation-delay',type=float,default=0.)
    p.add_argument('--filtered-rate-feedback',action='store_true',
                   help='Offline only: supply the plant 40 Hz low-pass rates to MPC instead of true rates')
    p.add_argument('--filtered-d-term',action='store_true',
                   help='Compatibility flag: base PX4 model now differentiates filtered gyro')
    p.add_argument('--timing-flight-dir',type=Path,
                   help='Use recorded successful MPC measurement times; extend uniform clock after recording ends')
    a=p.parse_args(); a.outdir.mkdir(parents=True,exist_ok=False)
    plan=load_plan(json.loads((a.plan_dir/'result.json').read_text()))
    ref=Reference(plan.sample(0),plan); ref.advance(0.)
    from convex_timevarying_window.single_gate_stop_racing import px4_model
    paths=[Path(__file__),Path(__file__).with_name('controller.py'),Path(__file__).with_name('model.py'),Path(px4_model.__file__),a.plan_dir/'result.json']
    control_times=None
    if a.timing_flight_dir:
        path=a.timing_flight_dir/'mpc_live.jsonl'; paths.append(path)
        logs=[json.loads(line) for line in path.read_text().splitlines()]
        control_times=np.array([r['time'] for r in logs if r['time']>=0 and r.get('solver_status')==0])
    digest=lambda: {str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    before=digest()
    (a.outdir/'protocol.json').write_text(json.dumps(dict(sources=before,dt=DT,
        control_target_dt=a.control_period,actuation_delay_s=a.actuation_delay,
        filtered_rate_feedback=a.filtered_rate_feedback,
        filtered_d_term=True,
        timing_flight_dir=str(a.timing_flight_dir) if a.timing_flight_dir else None,
        initial='hover at fixed reference start',layers=a.layers,
        rti_iterations=a.rti_iterations),indent=2))
    for layer in a.layers: run(layer,ref.sample,plan.end_time+1.,a.outdir,a.rti_iterations,
                              a.control_period,a.actuation_delay,a.filtered_rate_feedback,a.filtered_d_term,control_times)
    (a.outdir/'source_verification.json').write_text(json.dumps(dict(unchanged=before==digest(),sources=digest()),indent=2))


if __name__=='__main__': main()
