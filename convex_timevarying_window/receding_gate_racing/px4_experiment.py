"""PX4 adapter for the uniform three-gate space/time planner.

The planner runs asynchronously. A timestamped PVA segment is streamed at
100 Hz; execution never pauses the world's external gate clock.
"""
from __future__ import annotations
import argparse
import csv
from dataclasses import asdict, replace
from datetime import datetime
import json
import hashlib
from pathlib import Path
import queue
import subprocess
import threading
import time
import numpy as np

from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_dynamic_seven_window_gazebo.online_safe_mppi_px4_experiment import (
    CONTAINER, CONTACT_TOPICS, MOTION_START_S, PARTITION, ClockReader, StallWatchdog,
    receive, set_parameter, mavutil,
)
from convex_dynamic_seven_window_gazebo.px4_togt_nmpc_experiment import (
    enu_to_ned, parse_contacts, send_gcs_heartbeat, send_setpoint,
)
from .planner import Config, Planner
from .experiment import advance_gate
from .scenario import vary_track, launch_variant

ROOT=Path(__file__).resolve().parents[2]
ASSETS=ROOT/'convex_dynamic_seven_window_gazebo'


def command_plan(active, pending, physical_time, tracking_delay):
    """Keep the physical reference clock distinct from the command preview."""
    command_time=physical_time+tracking_delay
    if pending is not None and command_time>=pending.made_at:return pending,command_time
    return active,command_time


def planning_anchor(active,gate,now,lead,horizon,window_count,anticipation):
    """Prepare the next horizon at a fixed future splice, before crossing."""
    if active is not None and gate<window_count:
        index=gate-active.first_gate
        count=min(horizon,window_count-active.first_gate)
        if 0<=index<count:
            crossing=active.crossing_times[index]
            if 0<=crossing-now<=anticipation:
                return gate+1,float(crossing+lead)
    return gate,now+lead


class Worker:
    def __init__(self, planner):
        self.planner=planner; self.requests=queue.Queue(maxsize=1); self.results=queue.Queue()
        self.stop=False
        self.thread=threading.Thread(target=self.run,daemon=True); self.thread.start()

    def run(self):
        while not self.stop:
            try: state,t,gate=self.requests.get(timeout=.1)
            except queue.Empty: continue
            started=time.perf_counter()
            try:
                plan=self.planner.solve(state,t,gate)
                clearance=self.planner.prefix_clearance(plan,t)
                elapsed=1000*(time.perf_counter()-started)
                self.results.put((plan,clearance,elapsed,None))
            except Exception as exc: self.results.put((None,None,1000*(time.perf_counter()-started),repr(exc)))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--launch',action='store_true')
    parser.add_argument('--max-flight-time',type=float,default=60.)
    parser.add_argument('--outdir',type=Path)
    parser.add_argument('--phase',type=float,default=0.)
    parser.add_argument('--layout',type=float,default=0.)
    parser.add_argument('--motion-rate',type=float,default=1.)
    parser.add_argument('--translation-scale',type=float,default=1.)
    parser.add_argument('--rotation-scale',type=float,default=1.)
    parser.add_argument('--center',action='store_true')
    crossing=parser.add_mutually_exclusive_group()
    crossing.add_argument('--local-sweep',action='store_true',help='time-local crossing constraints (now default)')
    crossing.add_argument('--legacy-crossing',action='store_true',help='reproduce the old worst-case speed constraint')
    parser.add_argument('--race',action='store_true',help='uniform higher-authority x500 configuration')
    parser.add_argument('--nested',action='store_true',help='centre incumbent followed by free-point refinement')
    parser.add_argument('--continuation',action='store_true',help='rest continuation, workspace tube and continuous reference switching')
    parser.add_argument('--tracking-preview',type=float,default=0.,help='identified end-to-end tracking delay, seconds; continuation only')
    parser.add_argument('--guarded',action='store_true',help='commit only a checked racing prefix plus analytic brake')
    parser.add_argument('--checked',action='store_true',help='joint local racing/rest trajectory with batched admissibility checks')
    parser.add_argument('--checked-soft',action='store_true',help='checked plus dynamic-frame soft clearance objective')
    parser.add_argument('--anticipatory',action='store_true',help='prepare the next checked horizon before crossing')
    parser.add_argument('--paper-polynomial',action='store_true',help='independent IROS 2023 time-uniform MINCO baseline; same PX4 transport only')
    args=parser.parse_args()
    if args.checked_soft:
        if any((args.guarded,args.paper_polynomial,args.nested,args.center,args.race,args.legacy_crossing,args.anticipatory)):
            parser.error('checked-soft is an objective-only variant; do not combine with other algorithm changes')
        args.checked=True
    if args.guarded and args.checked:parser.error('choose either guarded or checked')
    if args.guarded or args.checked:args.continuation=True
    if args.paper_polynomial and (args.continuation or args.nested or args.center or args.anticipatory):
        parser.error('paper baseline cannot be combined with proposed-method options')
    continuous_reference=args.continuation or args.paper_polynomial
    if args.anticipatory and not args.checked:parser.error('anticipatory shifting requires checked continuation')
    if not np.isfinite(args.tracking_preview) or not 0<=args.tracking_preview<=.2:
        parser.error('tracking preview must be finite and in [0, 0.2] seconds')
    if args.tracking_preview and not continuous_reference:parser.error('tracking preview requires continuous reference transport')
    config_type=Config
    if args.checked_soft:
        from convex_timevarying_window.checked_soft_racing.planner import Config as config_type
    cfg=replace(config_type(),free_points=not args.center); track,_=build_track()
    if args.local_sweep: cfg=replace(cfg,crossing_constraint='local_sweep')
    if args.legacy_crossing: cfg=replace(cfg,crossing_constraint='worst_case')
    # Arena geometry is an input from this Gazebo scene, not a gate-dependent
    # planner rule. Keep a metre from the visual side walls and back wall.
    cfg=replace(cfg,workspace_lower=(-28.,-30.,.65),workspace_upper=(28.,22.,12.))
    if args.race: cfg=replace(cfg,speed=14.,acceleration=9.5,jerk=35.,jerk_weight=.0003)
    if continuous_reference: cfg=replace(cfg,horizon=2)
    if args.paper_polynomial: cfg=replace(cfg,free_points=False,crossing_constraint='paper_center_equality')
    vary_track(track,args.phase,args.layout,args.motion_rate,args.translation_scale,args.rotation_scale)
    source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in Path(__file__).parent.glob('*.py')}
    began=time.perf_counter()
    if args.paper_polynomial:
        from 复现.polynomial_dynamic_racing_2023.planner import Planner as PaperPlanner
        planner=PaperPlanner(track,cfg)
        for p in (ROOT/'复现/polynomial_dynamic_racing_2023').glob('*.py'):
            source_hashes['paper_'+p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
    elif args.continuation:
        from convex_timevarying_window.continuation_racing.planner import Planner as ContinuationPlanner
        if args.guarded:
            from convex_timevarying_window.continuation_racing.guard import Planner as ContinuationPlanner
        if args.checked:
            from convex_timevarying_window.continuation_racing.guard import CheckedPlanner as ContinuationPlanner
        if args.checked_soft:
            from convex_timevarying_window.checked_soft_racing.planner import Planner as ContinuationPlanner
            for p in (ROOT/'convex_timevarying_window/checked_soft_racing').glob('*.py'):
                source_hashes['checked_soft_'+p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
        planner=ContinuationPlanner(track,cfg)
        source_hashes['continuation_planner.py']=hashlib.sha256((ROOT/'convex_timevarying_window/continuation_racing/planner.py').read_bytes()).hexdigest()
        if args.guarded or args.checked:source_hashes['continuation_guard.py']=hashlib.sha256((ROOT/'convex_timevarying_window/continuation_racing/guard.py').read_bytes()).hexdigest()
    elif args.nested and not args.center:
        from .nested import NestedPlanner
        planner=NestedPlanner(track,cfg)
    else: planner=Planner(track,cfg)
    build_s=time.perf_counter()-began
    run_dir=args.outdir or Path(__file__).parent/'results'/('px4_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    run_dir.mkdir(parents=True,exist_ok=False)
    if args.launch:
        launch_variant(track,ASSETS,run_dir)
    # Gazebo's lazy air-pressure publisher can stop despite the PX4 bridge
    # subscription. A read-only subscriber keeps the real sensor publishing;
    # no sensor values, arming checks, or PX4 parameters are changed.
    baro_log=(run_dir/'barometer.pbtxt').open('w')
    baro_reader=subprocess.Popen(['docker','exec','-e',f'GZ_PARTITION={PARTITION}',CONTAINER,
        'gz','topic','-e','-t','/world/convex_seven_dynamic_px4_togt/model/x500_0/link/base_link/sensor/air_pressure_sensor/air_pressure'],
        stdout=baro_log,stderr=subprocess.DEVNULL)
    parameters={'MPC_XY_VEL_MAX':14.,'MPC_XY_CRUISE':12.,'MPC_Z_VEL_MAX_UP':7.,
                'MPC_Z_VEL_MAX_DN':7.,'MPC_ACC_HOR':10.,'MPC_ACC_HOR_MAX':12.,
                'MPC_ACC_UP_MAX':9.,'MPC_ACC_DOWN_MAX':9.,'MPC_JERK_AUTO':35.,
                'MPC_JERK_MAX':45.,'MPC_TILTMAX_AIR':45.,
                'MPC_XY_P':2.,'MPC_XY_VEL_P_ACC':3.5,'MPC_Z_P':2.,
                'SENS_GPS0_DELAY':0.,'SENS_GPS1_DELAY':0.}
    if args.race: parameters['MPC_XY_VEL_MAX']=16.
    for k,v in parameters.items(): set_parameter(k,v)
    link=mavutil.mavlink_connection('udpin:0.0.0.0:14550',source_system=254,
                                    source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER)
    if link.wait_heartbeat(timeout=10) is None: raise RuntimeError('missing PX4 heartbeat')
    clock=ClockReader(); latest=None
    deadline=time.monotonic()+5
    while latest is None and time.monotonic()<deadline:
        latest=link.recv_match(type='LOCAL_POSITION_NED',blocking=True,timeout=.5)
    if latest is None: raise RuntimeError('missing local position')
    while clock.value is None and time.monotonic()<deadline: time.sleep(.01)
    if clock.value is None or clock.value>18: raise RuntimeError(f'preparation clock {clock.value}')
    origin=np.array([-16.-latest.y,4.-latest.x,-5.76+latest.z])
    start_ned=enu_to_ned(track.start,position=True,origin_enu=origin)
    processes=[(baro_reader,baro_log)]; paths=[]
    # Read-only keepalive, never passed to the controller. Prevent missing
    # ground-truth evidence caused by Gazebo's lazy odometry publisher.
    truth_reader=subprocess.Popen(['docker','exec','-e',f'GZ_PARTITION={PARTITION}',CONTAINER,
        'gz','topic','-e','-t','/model/x500_0/odometry_with_covariance'],
        stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    for i,topic in enumerate(CONTACT_TOPICS):
        path=run_dir/f'gate_{i+1:02d}_contacts.pbtxt'; f=path.open('w')
        proc=subprocess.Popen(['docker','exec','-e',f'GZ_PARTITION={PARTITION}',CONTAINER,
                               'gz','topic','-e','-t',topic],stdout=f,stderr=subprocess.DEVNULL)
        processes.append((proc,f)); paths.append(path)
    shared={'active':None,'pending':None,'hold':np.asarray(track.start).copy(),'stop':False}
    sending_ms=[]
    def sender():
        beat=0.
        while not shared['stop']:
            tick=time.perf_counter()
            t=(clock.value or 0)-MOTION_START_S
            if shared['pending'] is not None and t>=shared['pending'].made_at:
                shared['active']=shared['pending']; shared['pending']=None
            plan,command_time=command_plan(shared['active'],shared['pending'],t,args.tracking_preview)
            if plan is not None and 0<=t and (command_time<=plan.end_time or continuous_reference):
                p,v,a,j=[plan.sample(command_time,d) for d in range(4)]
                if continuous_reference and command_time>=plan.end_time: v=np.zeros(3); a=np.zeros(3); j=np.zeros(3)
                u=a+cfg.response_time*j
                send_setpoint(link,enu_to_ned(p,position=True,origin_enu=origin),
                    enu_to_ned(v,position=False,origin_enu=origin),
                    enu_to_ned(u,position=False,origin_enu=origin),0.)
            else: send_setpoint(link,enu_to_ned(shared['hold'],position=True,origin_enu=origin))
            if time.monotonic()-beat>.4: send_gcs_heartbeat(link); beat=time.monotonic()
            sending_ms.append(1000*(time.perf_counter()-tick))
            time.sleep(max(.0005,.01-(time.perf_counter()-tick)))
    thread=threading.Thread(target=sender,daemon=True); thread.start()
    worker=Worker(planner)
    rows=[]; solves=[]; crossings=[]; gate=0; finished=False; failure=None
    live_trace=(run_dir/'telemetry_live.csv').open('w')
    live_writer=csv.writer(live_trace)
    live_writer.writerow(['t','x','y','z','vx','vy','vz','ax','ay','az','rx','ry','rz','gate','error']);live_trace.flush()
    live_solves=(run_dir/'solves_live.jsonl').open('w')
    def record_row(row):
        rows.append(row);live_writer.writerow(row);live_trace.flush()
    now=0.; state=np.r_[track.start,np.zeros(6)]; last_p=state[:3].copy(); last_t=0.
    acceleration=np.zeros(3); last_v=np.zeros(3); velocity_t=None
    watchdog=StallWatchdog(); next_request=0.; last_row=-1.; completed=None
    deferred=None
    try:
        time.sleep(1.25)
        link.mav.set_mode_send(1,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,6<<16)
        arm_deadline=time.monotonic()+12; armed=False; last_arm=0.
        while not armed and time.monotonic()<arm_deadline:
            if time.monotonic()-last_arm>1:
                link.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
                last_arm=time.monotonic()
            msg=link.recv_match(blocking=False)
            if msg is not None:
                if msg.get_type()=='HEARTBEAT': armed=bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif msg.get_type()=='LOCAL_POSITION_NED': latest=msg
            time.sleep(.002)
        if not armed: raise RuntimeError('arm failed')
        while clock.value<MOTION_START_S:
            latest=receive(link,latest); time.sleep(.002)
        while clock.value<MOTION_START_S+args.max_flight_time:
            if watchdog.stalled(clock.value): failure='clock_stalled'; break
            latest=receive(link,latest); now=clock.value-MOTION_START_S
            pos=np.array([latest.y,latest.x,-latest.z])+origin
            vel=np.array([latest.vy,latest.vx,-latest.vz])
            if velocity_t is not None and now-velocity_t>=.018:
                raw=(vel-last_v)/(now-velocity_t)
                raw*=min(1.,cfg.acceleration/max(np.linalg.norm(raw),1e-9))
                acceleration=.8*acceleration+.2*raw
                last_v=vel.copy(); velocity_t=now
            elif velocity_t is None: last_v=vel.copy(); velocity_t=now
            state=np.r_[pos,vel,acceleration]
            if not np.all(np.isfinite(state)) or np.linalg.norm(vel)>4*cfg.speed or np.linalg.norm(pos-track.start)>1e4:
                failure='simulation_state_diverged';break
            passed,cross=advance_gate(planner,gate,last_p,pos,last_t,now)
            if passed:
                gate+=1; crossings.append(cross); next_request=now
                print('PX4 pass',gate,round(now,3),flush=True)
            last_p=pos.copy(); last_t=now
            if deferred is not None:
                prepared,record=deferred
                if now>prepared.made_at-args.tracking_preview or prepared.first_gate<gate:
                    deferred=None
                elif prepared.first_gate==gate:
                    planner.commit(prepared);shared['pending']=prepared
                    record['accepted']=True;record['admitted_later_at']=now
                    live_solves.write(json.dumps(dict(record,admission_update=True))+'\n');live_solves.flush()
                    deferred=None
            while not worker.results.empty():
                candidate,clearance,elapsed,error=worker.results.get()
                accepted=(candidate is not None and candidate.first_gate==gate and
                          candidate.violation<.003 and clearance>=cfg.tracking_margin and elapsed<100 and
                          now<candidate.made_at+.04)
                if continuous_reference and candidate is not None: accepted=accepted and now<=candidate.made_at-args.tracking_preview
                solves.append({'time':now,'cycle_ms':elapsed,'accepted':accepted,'gate':candidate.first_gate if candidate is not None and args.anticipatory else gate,
                               'observed_gate':gate,
                               'violation':candidate.violation if candidate else None,
                               'status':candidate.status if candidate else error,'prefix_clearance':clearance})
                if continuous_reference and candidate is not None:
                    solves[-1].update(made_at=candidate.made_at,durations=candidate.durations.tolist(),
                                      variables=candidate.x.tolist())
                    if candidate.violation<.003:solves[-1]['coefficients']=candidate.coefficients.tolist()
                    if candidate.violation<.003 and hasattr(candidate,'brake'):
                        solves[-1]['brake']={'made_at':candidate.brake.made_at,'durations':candidate.brake.durations.tolist(),
                                             'coefficients':candidate.brake.coefficients.tolist(),'clearance':candidate.clearance}
                if (args.anticipatory and candidate is not None and candidate.first_gate==gate+1 and
                    candidate.violation<.003 and clearance>=cfg.tracking_margin and elapsed<100 and
                    now<=candidate.made_at-args.tracking_preview):
                    if deferred is None or candidate.end_time<=deferred[0].end_time:
                        deferred=(candidate,solves[-1]);solves[-1]['deferred']=True
                live_solves.write(json.dumps(solves[-1])+'\n');live_solves.flush()
                if accepted:
                    if continuous_reference:planner.commit(candidate)
                    if now<candidate.made_at: shared['pending']=candidate
                    else: shared['active']=candidate
            active=shared['active']
            near_cross=(active is not None and gate<len(planner.windows) and active.first_gate==gate
                        and active.crossing_times[0]-now<.25
                        and (not continuous_reference or active.crossing_times[0]>=now))
            if args.guarded or args.checked:near_cross=False
            if (now>=next_request and not near_cross and worker.requests.empty()
                and (not args.anticipatory or shared['pending'] is None)):
                # Account for the computation/transport delay before applying a plan.
                lead=.10+args.tracking_preview
                target_gate,target_time=gate,now+lead
                if args.anticipatory:
                    target_gate,target_time=planning_anchor(active,gate,now,lead,cfg.horizon,len(planner.windows),4*cfg.replan_period)
                future=state.copy()
                if active is not None and continuous_reference:
                    future=np.concatenate([active.sample(target_time,d) for d in range(3)])
                elif active is not None:
                    future[:3]=active.sample(now+lead)+pos-active.sample(now)+(vel-active.sample(now,1))*lead
                    future[3:6]=active.sample(now+lead,1)+vel-active.sample(now,1)
                    future[6:9]=active.sample(now+lead,2)
                else: future[:3]+=vel*lead+.5*acceleration*lead**2; future[3:6]+=acceleration*lead
                worker.requests.put_nowait((future,target_time,target_gate)); next_request=now+cfg.replan_period
            if active is not None and now>active.end_time+.05 and gate<len(planner.windows) and not continuous_reference:
                failure='plan_expired'; break
            ref=active.sample(now) if active is not None else np.asarray(track.start)
            if now-last_row>=.018:
                record_row([now,*pos,*vel,*acceleration,*ref,gate,float(np.linalg.norm(pos-ref))]); last_row=now
            if gate==len(planner.windows) and np.linalg.norm(pos-track.goal)<.55 and np.linalg.norm(vel)<1.:
                record_row([now,*pos,*vel,*acceleration,*ref,gate,float(np.linalg.norm(pos-ref))])
                finished=True; completed=now; break
            time.sleep(.001)
        if not finished and failure is None: failure='timeout'
    except Exception as exc:
        failure=repr(exc)
    finally:
        shared['active']=None; shared['pending']=None; shared['hold']=state[:3].copy()
        worker.stop=True; worker.thread.join(timeout=1.)
        time.sleep(.5); shared['stop']=True; thread.join(timeout=1.)
        live_trace.close();live_solves.close()
        clock.close()
        truth_reader.terminate()
        for proc,f in processes:
            proc.terminate()
            try: proc.wait(timeout=2)
            except subprocess.TimeoutExpired: proc.kill()
            f.close()
    contacts=[parse_contacts(path) for path in paths]
    collision=any(c['contact_pair_count']>0 for c in contacts)
    subprocess.run(['docker','exec',CONTAINER,'/opt/px4-gazebo/bin/px4-logger','stop'],capture_output=True)
    log_files=subprocess.run(['docker','exec',CONTAINER,'bash','-lc',
                              'ls -t /opt/px4-gazebo/log/*/*.ulg'],capture_output=True,text=True)
    if log_files.returncode==0 and log_files.stdout.strip():
        source=log_files.stdout.splitlines()[0]
        subprocess.run(['docker','cp',f'{CONTAINER}:{source}',str(run_dir/'flight.ulg')],check=True)
    # Retain exact parser output too; different historical parsers use different keys.
    with (run_dir/'telemetry.csv').open('w') as f:
        writer=csv.writer(f); writer.writerow(['t','x','y','z','vx','vy','vz','ax','ay','az','rx','ry','rz','gate','error'])
        writer.writerows(rows)
    result={'completed':finished,'failure':failure,'flight_time_s':completed,'last_time_s':now,
            'collision_detected':collision,'contacts':contacts,'crossings':crossings,
            'config':asdict(cfg),'parameters':parameters,'solves':solves,'build_s':build_s,
            'phase':args.phase,'layout':args.layout,'motion_rate':args.motion_rate,'source_sha256':source_hashes,
            'translation_scale':args.translation_scale,'rotation_scale':args.rotation_scale,'numpy_version':np.__version__,
            'origin_enu_m':origin.tolist(),'race_configuration':args.race,
            'nested_refinement':args.nested,'selection_counts':getattr(planner,'selection_counts',None),
            'continuation':args.continuation,
            'method':('checked_soft_racing' if args.checked_soft else
                      'paper_polynomial_2023_independent' if args.paper_polynomial else 'proposed'),
            'continuous_reference_transport':continuous_reference,
            'tracking_preview_s':args.tracking_preview,
            'guarded':args.guarded,
            'checked':args.checked,
            'checked_soft':args.checked_soft,
            'anticipatory':args.anticipatory,
            'goal_error_m':float(np.linalg.norm(state[:3]-track.goal)),
            'goal_speed_mps':float(np.linalg.norm(state[3:6])),
            'sender_max_ms':max(sending_ms) if sending_ms else None,
            'evidence':'PX4 telemetry and Gazebo contacts, not a continuous certificate'}
    (run_dir/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='solves'},indent=2),flush=True)
    print(run_dir,flush=True)


if __name__=='__main__': main()
