"""Direct CTBR publication on the existing dedicated PX4/Gazebo experiment.

The sender performs only CTBR safety projection + frame/thrust encoding.
No acceleration/PVA feedback controller is present in the execution chain.
"""
from dataclasses import asdict
import json
import subprocess
import threading
import time
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_dynamic_seven_window_gazebo.online_safe_mppi_px4_experiment import CONTAINER,PARTITION,CONTACT_TOPICS,set_parameter
from convex_dynamic_seven_window_gazebo.px4_togt_nmpc_experiment import parse_contacts
from convex_timevarying_window.receding_gate_racing.scenario import vary_track
from convex_timevarying_window.dynamic_gate_mppi_v3.ctbr_probe import Session
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import hover_state,MASS
from convex_timevarying_window.dynamic_gate_mppi_v3.diagnostic import pvaj_from_state
from .reference import Planner,ReferenceWorker
from convex_timevarying_window.three_phase_mppi.controller import LinearReference
from convex_timevarying_window.gs_mppi.progress import Progress
from .controller import Controller
from .experiment import parser,snapshot
from .safety import filter_current,SLEW_PER_SEND

def remaining_tape(commands,planned_at,now,dt):
    """Do not append a new tail merely because solving consumed flight time."""
    elapsed=max(0.,now-planned_at)
    skip=min(len(commands),int(np.ceil(max(0.,elapsed-1e-8)/dt)))
    return commands[skip:].copy(),skip

class Observations:
    """State and crossing phase share one timestamp; caller holds its lock."""
    def __init__(self,progress,geometry):
        self.progress,self.geometry=progress,geometry
        self.last=None
    def update(self,at,x):
        exited=False
        if self.last is not None:
            old_time,old=self.last
            if at<=old_time:return old_time,old.copy(),False
            clear=observed_clearance(self.geometry,old_time,old,at,x)
            if clear<=0:raise RuntimeError('observed_interval_clearance')
            exited=self.progress.update(old[:3],old_time,x[:3],at)
            if self.progress.violation:raise RuntimeError(self.progress.violation)
        self.last=(at,x.copy())
        return at,x.copy(),exited

def observed_clearance(geometry,t0,x0,t1,x1):
    """Bound the measured interval using its chord plus curvature envelope.

    No intermediate measured state is invented: a bounded acceleration A
    places the actual body within A*dt^2/8 of its endpoint chord. The chord
    itself is checked on <=5 ms intervals against the known moving gates.
    As elsewhere, the physical A uses the stated 12 m/s speed assumption.
    """
    dt=t1-t0
    if dt<=0:raise ValueError('observations must have increasing times')
    cfg=geometry.config
    acceleration=4*cfg.max_rotor_thrust/MASS+9.8066+4*8.06428e-5*1000/MASS*cfg.speed
    fractions=np.linspace(0.,1.,max(1,int(np.ceil(dt/.005)))+1)
    positions=x0[:3]+fractions[:,None]*(x1[:3]-x0[:3])
    velocities=np.tile((x1[:3]-x0[:3])/dt,(len(fractions),1))
    chord=geometry.interval_clearances(positions,velocities,t0+fractions*dt,acceleration_bound=0.)
    return float(chord.min()-acceleration*dt*dt/8.)

def run(args):
    if args.single:raise ValueError('PX4 entry uses the unmodified full seven-gate world')
    track,opt=build_track();vary_track(track,args.phase,0.,args.motion_rate,args.translation_scale,args.rotation_scale)
    ctrl=Controller(track,args.samples,args.steps,args.seed,racing_guidance=args.experimental_racing_guidance,coupled_racing=args.coupled_racing)
    if args.coupled_racing:
        from .racing_reference import Planner as RacingPlanner,ReferenceWorker as RacingWorker
        RacingPlanner(track,opt)
    else:Planner(track,opt,racing_guidance=args.experimental_racing_guidance)
    initial=hover_state(track.start);tick=time.perf_counter();ctrl.select(initial,0.,0,0,None)
    ctrl.mean=None;ctrl.prepare_gate=None;ctrl.prior_time=None;warmup=time.perf_counter()-tick
    worker=RacingWorker(track,opt) if args.coupled_racing else ReferenceWorker(track,opt,racing_guidance=args.experimental_racing_guidance)
    session=Session(args.outdir,track,consistent_velocity=True);hashes=snapshot(args.outdir)
    # The optional frozen comparison profile is the PX4/x500 profile used by
    # the 2026-09-23 center-motion checked and Wang runs. CTBR still bypasses
    # their PVA position/velocity controller during racing.
    parameters={'SENS_GPS0_DELAY':0.,'SENS_GPS1_DELAY':0.}
    if args.matched_px4_parameters:
        parameters={'MPC_XY_VEL_MAX':14.,'MPC_XY_CRUISE':12.,'MPC_Z_VEL_MAX_UP':7.,
                    'MPC_Z_VEL_MAX_DN':7.,'MPC_ACC_HOR':10.,'MPC_ACC_HOR_MAX':12.,
                    'MPC_ACC_UP_MAX':9.,'MPC_ACC_DOWN_MAX':9.,'MPC_JERK_AUTO':35.,
                    'MPC_JERK_MAX':45.,'MPC_TILTMAX_AIR':45.,
                    'MPC_XY_P':2.,'MPC_XY_VEL_P_ACC':3.5,'MPC_Z_P':2.,
                    'SENS_GPS0_DELAY':0.,'SENS_GPS1_DELAY':0.}
    for name,value in parameters.items():set_parameter(name,value)
    contacts=[]
    for i,topic in enumerate(CONTACT_TOPICS):
        path=args.outdir/f'gate_{i+1:02d}_contacts.pbtxt';stream=path.open('w')
        process=subprocess.Popen(['docker','exec','-e',f'GZ_PARTITION={PARTITION}',CONTAINER,'gz','topic','-e','-t',topic],stdout=stream,stderr=subprocess.DEVNULL)
        contacts.append((process,stream,path))
    # Gazebo's pressure sensor can go idle without an external transport
    # subscriber, leaving PX4 with a stale barometer and an arming denial.
    # Keep this simulator sensor active; it does not enter the controller.
    baro_topic='/world/convex_seven_dynamic_px4_togt/model/x500_0/link/base_link/sensor/air_pressure_sensor/air_pressure'
    baro=subprocess.Popen(['docker','exec','-e',f'GZ_PARTITION={PARTITION}',CONTAINER,
                           'gz','topic','-e','-t',baro_topic],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    progress=Progress(track,ctrl.engine.config,ctrl.engine.signs,finite_entry_only=True);lock=threading.Lock()
    observed=Observations(progress,ctrl.engine.geometry)
    active=None;events=[];last_u=np.array([MASS*9.8066,0.,0.,0.]);sender_checks=[]
    cycles=[];updates=[];rows=[];pending='initial';running=None;ref=None;complete=False;failure=None;now=0.
    live=(args.outdir/'cycles.jsonl').open('w',buffering=1)
    def sender(x,at):
        nonlocal last_u
        started=time.perf_counter()
        with lock:
            at,x,exited=observed.update(at,x)
            if exited:events.append('gate_exit')
            if active is None:raise RuntimeError('missing_direct_control_sequence')
            made,commands=active;index=max(0,int((at-made)/ctrl.dt))
            if index>=len(commands):raise RuntimeError('direct_control_sequence_expired')
            check,safe,attempts,fallback=filter_current(ctrl.engine,x,commands[index],at,progress.first,progress.phase,previous=last_u)
            if safe is None:raise RuntimeError('current_CTBR_filter_rejected_'+str(int(check[1])))
            last_u=safe.copy();sender_checks.append((at,1000*(time.perf_counter()-started),*check,attempts,int(fallback)))
            return last_u.copy()
    try:
        session.prepare();now,x,_=session.read()
        boot=ctrl.select(x,now,0,0,None,last_u)
        if boot is None:
            np.savez_compressed(args.outdir/'bootstrap_failed.npz',time=now,state=x)
            (args.outdir/'bootstrap_failure.json').write_text(json.dumps(ctrl.last,indent=2)+'\n')
            raise RuntimeError('no_bootstrap_CTBR')
        with lock:active=(now,ctrl.mean.copy())
        session.feedback=sender
        while now<args.duration:
            now,x,age=session.read()
            if session.failure:raise RuntimeError(session.failure)
            with lock:
                now,x,exited=observed.update(now,x)
                if exited:events.append('gate_exit')
                first,phase=progress.first,progress.phase;previous=last_u.copy()
                if events:pending='gate_exit';events.clear()
            for _,candidate in worker.take():
                accepted=candidate is not None and not isinstance(candidate,dict) and candidate.first==first
                updates.append(dict(time=now,reason=running,accepted=accepted,error=candidate if isinstance(candidate,dict) else None,
                                    solve_ms=None if isinstance(candidate,dict) or candidate is None else candidate.solve_ms))
                if accepted:
                    ref=candidate
                    # Preserve the actual accepted reference for causal replay;
                    # a later rerun of a budgeted solver need not return it.
                    reference_dir=args.outdir/'references';reference_dir.mkdir(exist_ok=True)
                    np.savez_compressed(reference_dir/f'{len(updates):04d}.npz',
                                        accepted_at=now,made_at=ref.made_at,first=ref.first,
                                        durations=ref.trajectory.durations,coefficients=ref.trajectory.coefficients,
                                        crossing_times=ref.crossing_times,local_points=ref.local_points)
                else:pending=pending or running
                running=None
            if pending and running is None:
                if worker.submit(len(updates),pvaj_from_state(ctrl.engine,x,previous),now,first):running=pending;pending=None
            tick=time.perf_counter();action=ctrl.select(x,now,first,phase,ref,previous)
            at,fresh,_=session.read()
            with lock:
                at,fresh,exited=observed.update(at,fresh)
                if exited:events.append('gate_exit')
                actual_first,actual_phase=progress.first,progress.phase
            accepted=False;check=None;current_check=None;future_warning=False
            if action is not None:
                # Full local direct tape is rechecked from fresh measured state
                # and fresh gate times after computation, not blindly published.
                tape,skipped=remaining_tape(ctrl.mean,now,at,ctrl.dt)
                poses=ctrl.engine.poses(at+np.arange(4*len(tape)+1)*.005)
                reference=ctrl.reference(ref,at,fresh)[:len(tape)+1]
                if len(tape):
                    stage_mode={'TRANSIT':0,'PREPARE':1,'CROSS':2}[ctrl.stage]
                    check,_,commands=ctrl.engine.rollout(fresh,tape[None],reference,poses,actual_first,actual_phase,stage_mode,gpu=False,previous_command=previous)
                    future_warning=bool(check[1]!=0)
                    # This is receding-horizon control, not a commitment to
                    # execute the entire remaining open-loop tape unchanged.
                    # Validate the actual new command interval from fresh state;
                    # the sender repeats this check at every publication.
                    current_check,first_control,activation_attempts,activation_fallback=filter_current(
                        ctrl.engine,fresh,tape[0],at,actual_first,actual_phase,stage_mode,
                        poses=poses[:5],reference=reference[:2],previous=previous)
                    if future_warning:commands=tape.copy()
                    if first_control is not None:commands[0]=first_control
                accepted=bool(current_check is not None and first_control is not None and current_check[1]==0 and 1000*(time.perf_counter()-tick)<100)
                if accepted:
                    with lock:active=(at,commands.copy())
                    # Pad only the next optimization's warm start; unverified
                    # padding is never published to the execution thread.
                    ctrl.mean=np.concatenate((commands,np.tile(commands[-1],(ctrl.steps-len(commands),1))))
                    ctrl.prior_time=at
            record=dict(ctrl.last,cycle_ms=1000*(time.perf_counter()-tick),accepted=accepted,activation=None if current_check is None else current_check.tolist(),
                        activation_filter_attempts=activation_attempts if current_check is not None else None,
                        activation_filter_fallback=activation_fallback if current_check is not None else None,
                        future_warning=future_warning,future_check=None if check is None else check.tolist())
            cycles.append(record);live.write(json.dumps(record)+'\n')
            if not accepted:
                if hasattr(ctrl,'failure_data'):np.savez_compressed(args.outdir/'failed_selection.npz',**ctrl.failure_data)
                raise RuntimeError('fresh_state_CTBR_not_admitted')
            if future_warning and ctrl.stage=='TRANSIT' and int(check[1])&1:
                # Release the TOGT objective during a detected transit hazard;
                # request a fresh reference without a 100 ms old-policy gate.
                ref=LinearReference(fresh,at);pending=pending or 'obstacle_recovery'
            if ctrl.last['avoiding']:pending=pending or 'obstacle_recovery'
            if ref is not None and ctrl.stage=='TRANSIT':
                r=ctrl.reference(ref,at,fresh)[0]
                if np.linalg.norm(fresh[:3]-r[:3])>.5 or np.linalg.norm(fresh[3:6]-r[3:6])>2:pending=pending or 'tracking_error'
            rows.append([at,*fresh])
            if actual_first==len(track.order) and np.linalg.norm(fresh[:3]-track.goal)<.55 and np.linalg.norm(fresh[3:6])<1:complete=True;break
        if not complete:failure='timeout'
    except Exception as exc:failure=repr(exc)
    finally:
        now,terminal,_=session.read();session.close();worker.close();live.close()
        baro.terminate()
        try:baro.wait(2)
        except subprocess.TimeoutExpired:baro.kill();baro.wait()
        for process,stream,path in contacts:
            process.terminate()
            try:process.wait(2)
            except subprocess.TimeoutExpired:process.kill();process.wait()
            stream.close()
    contact_info=[parse_contacts(path) for _,_,path in contacts]
    np.savez_compressed(args.outdir/'telemetry.npz',rows=np.asarray(rows),sender_checks=np.asarray(sender_checks))
    result=dict(method='direct_CTBR_MPPI_17state',completed=complete,failure=failure,last_time_s=now,flight_time_s=now if complete else None,
        crossings=progress.crossings,center_crossings=progress.centers,collision_detected=any(c['contact_pair_count'] for c in contact_info),contacts=contact_info,
        solves=cycles,reference_updates=updates,warmup_s=warmup,source_sha256=hashes,px4_parameters=parameters,
        config={**asdict(ctrl.engine.config),'horizon':ctrl.steps*ctrl.dt,'samples':ctrl.samples,'temperature':.4,
                'control_revision':'coupled_adaptive_transient_v6_20260926' if args.coupled_racing else ('experimental_corner_seed_a6p5_20260926' if args.experimental_racing_guidance else 'flow_smooth_v7_20260926'),
                'coupled_racing':args.coupled_racing,
                'autonomous_continuation':'next gate distance/velocity heuristic',
                'reference_jerk_feedforward_seed':args.coupled_racing,
                'reference_dynamics_preferences':{'speed':11.5,'acceleration':7.5,'response_lead_s':.15,'tilt_deg':40.,'jerk_energy_weight':.0006} if args.coupled_racing else None,
                'experimental_racing_guidance':args.experimental_racing_guidance,
                'seed_speed_limit_mps':.9*ctrl.engine.config.speed if args.experimental_racing_guidance else 9.,
                'seed_acceleration_limit_mps2':.9*ctrl.engine.config.acceleration if args.coupled_racing else 6.5,'corner_aware_exit_speed':args.experimental_racing_guidance,
                'togt_point_inset_m':ctrl.engine.config.radius+ctrl.engine.config.error_margin+.02+.10 if args.experimental_racing_guidance or args.coupled_racing else None,
                'reference_aligned_takeover_lead_s':.8 if args.experimental_racing_guidance or args.coupled_racing else None,
                'tilt_soft_reserve_deg':2.,'tilt_outward_lookahead_s':.18,'tilt_warning_weight':2000.,
                'cross_soft_clearance_reserve_m':ctrl.engine.config.error_margin,
                'cross_soft_clearance_weight':200.,
                'finite_aperture_entry_guard':True,
                'sampling_interpolation':'linear between 100 ms CTBR noise knots',
                'rate_change_cost':0.0008,'angular_acceleration_cost':0.0015,
                'slew_per_publication':SLEW_PER_SEND.tolist(),
                'heading':'TRANSIT follows current reference tangent; PREPARE/CROSS follow candidate path tangent; hold below 0.2 m/s horizontal tangent',
                'publication_projection_iterations':4,'workspace_lower':list(ctrl.engine.config.lower),'workspace_upper':list(ctrl.engine.config.upper)},
        phase=args.phase,phase_units='radians',layout=0.,motion_rate=args.motion_rate,
        translation_scale=args.translation_scale,rotation_scale=args.rotation_scale,
        origin_enu_m=session.origin.tolist(),goal_error_m=float(np.linalg.norm(terminal[:3]-track.goal)),goal_speed_mps=float(np.linalg.norm(terminal[3:6])),
        evidence='PX4/Gazebo direct CTBR; empirical prediction model and numerical safety projection; NOT a robust safety certificate')
    (args.outdir/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('completed','failure','last_time_s','crossings','collision_detected')},indent=2),flush=True)
    return result

if __name__=='__main__':run(parser().parse_args())
