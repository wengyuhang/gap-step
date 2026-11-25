"""Independent three-phase controller on the unchanged x500/PX4 scenario."""
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
from .reference import Planner,AsyncPlanner
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import hover_state,MASS
from convex_timevarying_window.dynamic_gate_mppi_v3.diagnostic import pvaj_from_state
from .controller import Controller,Progress,config
from .experiment import parser,snapshot
from .events import reason,computation_safe

class ReferenceWorker:
    """Reception at 1 ms wall intervals; no geometry or reference safety test."""
    def __init__(self,track,opt):
        self.worker=AsyncPlanner(track,opt);self.lock=threading.Lock();self.returned=[];self.stop=False
        self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()
    def loop(self):
        while not self.stop:
            with self.lock:
                r=self.worker.poll()
                if r is not None:self.returned.append(r)
            time.sleep(.001)
    def submit(self,identifier,head,now,first):
        with self.lock:return self.worker.submit(identifier,head,now,first)
    def take(self):
        with self.lock:r=self.returned;self.returned=[];return r
    def close(self):
        self.stop=True;self.thread.join(1);self.worker.close()

def run(args):
    track,opt=build_track();vary_track(track,args.phase,args.layout,args.motion_rate,args.translation_scale,args.rotation_scale)
    cfg=config(args.samples);ctrl=Controller(track,cfg,args.seed);Planner(track,opt)
    previous=np.array([MASS*9.8066,0,0,0.]);initial=hover_state(track.start)
    warmstart=time.perf_counter();ctrl.select(initial,0.,0,0,None,previous);ctrl.prepare_gate=0
    for _ in range(3):
        sel=ctrl.select(initial,0.,0,0,None,previous)
        if sel is not None:ctrl.active=sel
    ctrl.active=None;ctrl.prepare_gate=None;warmup=time.perf_counter()-warmstart
    worker=ReferenceWorker(track,opt);s=Session(args.outdir,track,consistent_velocity=True);hashes=snapshot(args.outdir)
    # Match the existing paper-baseline scenario, including its documented
    # simulator GPS timestamp correction. The old CTBR probe omitted these.
    parameters={'MPC_XY_VEL_MAX':14.,'MPC_XY_CRUISE':12.,'MPC_Z_VEL_MAX_UP':7.,
        'MPC_Z_VEL_MAX_DN':7.,'MPC_ACC_HOR':10.,'MPC_ACC_HOR_MAX':12.,'MPC_ACC_UP_MAX':9.,
        'MPC_ACC_DOWN_MAX':9.,'MPC_JERK_AUTO':35.,'MPC_JERK_MAX':45.,'MPC_TILTMAX_AIR':45.,
        'MPC_XY_P':2.,'MPC_XY_VEL_P_ACC':3.5,'MPC_Z_P':2.,'SENS_GPS0_DELAY':0.,'SENS_GPS1_DELAY':0.}
    for key,value in parameters.items():set_parameter(key,value)
    baro_stream=(args.outdir/'barometer.pbtxt').open('w')
    baro=subprocess.Popen(['docker','exec','-e',f'GZ_PARTITION={PARTITION}',CONTAINER,'gz','topic','-e','-t',
        '/world/convex_seven_dynamic_px4_togt/model/x500_0/link/base_link/sensor/air_pressure_sensor/air_pressure'],stdout=baro_stream,stderr=subprocess.DEVNULL)
    contacts=[]
    for i,topic in enumerate(CONTACT_TOPICS):
        path=args.outdir/f'gate_{i+1:02d}_contacts.pbtxt';stream=path.open('w')
        proc=subprocess.Popen(['docker','exec','-e',f'GZ_PARTITION={PARTITION}',CONTAINER,'gz','topic','-e','-t',topic],stdout=stream,stderr=subprocess.DEVNULL)
        contacts.append((proc,stream,path))
    progress=Progress(track,cfg,ctrl.engine.signs);ref=None;pending='initial';running=None
    rows=[];solves=[];updates=[];policies=[];failure=None;completed=False;now=0.;last_measurement=None
    command_lock=threading.Lock();previous=previous.copy();last_sent_time=-1.
    live=(args.outdir/'cycles.jsonl').open('w',buffering=1)
    def sender(x,at):
        nonlocal previous,last_sent_time
        with command_lock:
            if at<=last_sent_time+1e-9:return previous.copy()
            u=ctrl.action(x,at,previous);previous=u;last_sent_time=at;return u
    def consume_measurements():
        nonlocal last_measurement,pending
        for t,x in s.drain_measurements():
            if t<0:continue
            if last_measurement is not None:
                t0,x0=last_measurement
                if t>t0:
                    clear=ctrl.engine.geometry.interval_clearances(np.array([x0[:3],x[:3]]),np.array([x0[3:6],x[3:6]]),np.array([t0,t])).min()
                    if clear<=0:raise RuntimeError('actual_interval_clearance')
                    if progress.update(x0[:3],t0,x[:3],t):pending='gate_exit'
                    if progress.violation:raise RuntimeError(progress.violation)
            last_measurement=t,x
    try:
        s.prepare();now,state,_=s.read()
        sel=ctrl.select(state,now,progress.first,progress.phase,None,previous)
        bootstrap=dict(ctrl.last,accepted=sel is not None,bootstrap=True)
        solves.append(bootstrap);live.write(json.dumps(bootstrap)+'\n')
        if sel is None:raise RuntimeError('no_initial_policy')
        ctrl.active=sel;s.feedback=sender;next_control=now
        while now<args.duration:
            now,state,age=s.read()
            if s.failure:raise RuntimeError(s.failure)
            consume_measurements()
            for _,candidate in worker.take():
                accepted=candidate is not None and not isinstance(candidate,dict) and candidate.first==progress.first
                updates.append(dict(time=now,accepted=accepted,error=candidate if isinstance(candidate,dict) else None,
                    reason=running,solve_ms=None if isinstance(candidate,dict) or candidate is None else candidate.solve_ms))
                if accepted:ref=candidate
                else:pending=pending or running
                running=None
            # Scheduling before and after controller update: event-triggered
            # requests are not blocked by an obsolete reference or gate phase.
            with command_lock:last=previous.copy()
            if pending and running is None and ctrl.active is not None:
                prefix,_,_=ctrl.prefix(ctrl.active,state,now,progress.first,progress.phase,last)
                if computation_safe(prefix) and worker.submit(len(updates),pvaj_from_state(ctrl.engine,state,last),now,progress.first):running=pending;pending=None
            if now>=next_control:
                tick=time.perf_counter();sel=ctrl.select(state,now,progress.first,progress.phase,ref,last)
                at,actual,_=s.read();consume_measurements()
                accepted=False;recheck=None
                if sel is not None and sel.first==progress.first:
                    with command_lock:last=previous.copy()
                    # Recheck the full LOCAL selected feedback from fresh
                    # measured state, including motion during computation.
                    steps=max(10,int(np.ceil((sel.valid_until-at)/.01)))+(10 if sel.mode else 0)
                    poses=ctrl.engine.prepare(at,steps);refs=ctrl.reference_array(sel.reference,at,steps,actual)
                    m,predicted,commands=ctrl.engine.evaluate(actual,last,sel.at(at)[None],refs,poses,progress.first,progress.phase,sel.mode,False)
                    recheck=int(m[1]);accepted=bool(m[1]==0 and (time.perf_counter()-tick)<.1)
                    if accepted:
                        sel.valid_until=at+m[5]*.01
                        ctrl.active=sel;policies.append((at,sel.vector.copy(),sel.made_at,predicted.copy()))
                record=dict(ctrl.last,accepted=accepted,activation_check=recheck,cycle_ms=1000*(time.perf_counter()-tick))
                solves.append(record);live.write(json.dumps(record)+'\n')
                pending=pending or reason(ctrl.stage,avoiding=record['transit_avoidance'])
                if ref is not None and ctrl.stage=='TRANSIT':
                    r=ctrl.reference_array(ref,at,0,actual)[0]
                    pending=pending or reason(ctrl.stage,position_error=np.linalg.norm(actual[:3]-r[:3]),velocity_error=np.linalg.norm(actual[3:6]-r[3:6]))
                if not accepted:
                    m,prefix_states,_=ctrl.prefix(ctrl.active,actual,at,progress.first,progress.phase,last)
                    if m[1]!=0 or ctrl.active.valid_until-at<.1:
                        np.savez_compressed(args.outdir/'failed_prefix.npz',time=at,states=prefix_states,metrics=m)
                        raise RuntimeError('no_admissible_successor')
                next_control=now+.02
            rows.append([now,*state[:6],0.,0.,0.,*state[:3],progress.first,0.])
            if progress.first==len(track.order) and np.linalg.norm(state[:3]-track.goal)<.55 and np.linalg.norm(state[3:6])<1:completed=True;break
            time.sleep(.001)
        if not completed:failure='timeout'
    except Exception as error:failure=repr(error)
    finally:
        now,terminal,_=s.read();s.close();worker.close();live.close()
        baro.terminate()
        try:baro.wait(2)
        except subprocess.TimeoutExpired:baro.kill();baro.wait()
        baro_stream.close()
        for proc,stream,path in contacts:
            proc.terminate()
            try:proc.wait(2)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
            stream.close()
    contact_info=[parse_contacts(path) for _,_,path in contacts]
    np.savez_compressed(args.outdir/'policies.npz',**{f'{i}_{k}':v for i,(t,pol,made,states) in enumerate(policies)
        for k,v in (('time',t),('policy',pol),('made_at',made),('states',states))})
    np.savetxt(args.outdir/'telemetry.csv',rows,delimiter=',',header='t,x,y,z,vx,vy,vz,ax,ay,az,rx,ry,rz,gate,error',comments='')
    result=dict(method='three_phase_mppi',completed=completed,failure=failure,last_time_s=now,flight_time_s=now if completed else None,
        crossings=progress.crossings,collision_detected=any(c['contact_pair_count'] for c in contact_info),contacts=contact_info,
        solves=solves,reference_updates=updates,warmup_s=warmup,source_sha256=hashes,
        config={**asdict(cfg),'workspace_lower':list(cfg.lower),'workspace_upper':list(cfg.upper)},
        px4_parameters=parameters,
        phase=args.phase,layout=args.layout,motion_rate=args.motion_rate,translation_scale=args.translation_scale,rotation_scale=args.rotation_scale,
        origin_enu_m=s.origin.tolist(),goal_error_m=float(np.linalg.norm(terminal[:3]-track.goal)),goal_speed_mps=float(np.linalg.norm(terminal[3:6])),
        evidence='PX4 CTBR; empirical inner loop; sampled checks, not robust safety certificate')
    (args.outdir/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('completed','failure','last_time_s','crossings','collision_detected')},indent=2),flush=True)
    return result

if __name__=='__main__':run(parser().parse_args())
