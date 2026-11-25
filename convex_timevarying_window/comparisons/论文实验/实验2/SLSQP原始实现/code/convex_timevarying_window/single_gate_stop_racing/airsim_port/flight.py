"""Native AirSim paused-step closed-loop flight, never trajectory replay.

The packaged engine uses wall timers for continueForTime, not exact tick counts.
Native timestamps, rather than the requested step, define all trajectory times.
"""
import argparse
import hashlib
import shutil
import sys
from dataclasses import asdict
import json
import math
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
from ..events import Progress
from ..planner import Config,route_signs
from ..scenario import workspace_bounds,manifest,Case
from .model import GenericQuad,ENU_TO_NED,FRD_TO_FLU,ned_to_enu,enu_to_ned,body_envelope,planning_seed_parameters
from .rpc import Client
from .scene import Scene,pose
from .mpc import MPC
from .planner import Planner


def values(d):
    return np.array([d[k] for k in ('x_val','y_val','z_val')])


def physical_state(raw,origin):
    k=raw['kinematics_estimated'];q=k['orientation']
    rn=Rotation.from_quat([q[k] for k in ('x_val','y_val','z_val','w_val')]).as_matrix()
    r=ENU_TO_NED.T@rn@FRD_TO_FLU
    quat=Rotation.from_matrix(r).as_quat()[[3,0,1,2]]
    return np.r_[ned_to_enu(values(k['position']),origin),
        ENU_TO_NED.T@values(k['linear_velocity']),quat,
        FRD_TO_FLU@values(k['angular_velocity'])]


def climb_reference(head,tail,duration):
    # Zero-VAJ seventh-degree smooth transition, used only before race t=0.
    coefficients=np.array([0,0,0,0,35,-84,70,-20.])
    delta=tail-head
    def reference(times,order=0):
        tt=np.asarray(times);u=np.clip(tt/duration,0,1)
        c=np.array([math.factorial(k)/math.factorial(k-order)*coefficients[k]
                    if k>=order else 0. for k in range(8)])
        f=sum(c[k]*u**(k-order) for k in range(order,8))/duration**order
        if order:f=np.where((tt<0)|(tt>duration),0,f)
        return (head if order==0 else np.zeros(3))+f[:,None]*delta
    return reference


def plan_reference(plan):
    def reference(times,order=0):
        result=plan.sample_many(times,order)
        if order:result[np.asarray(times)>plan.end_time]=0.
        return result
    return reference


def camera(client,state,scene,gate,heading):
    velocity=state[3:6].copy();velocity[2]=0
    if np.linalg.norm(velocity)>.5:
        heading=.85*heading+.15*velocity/np.linalg.norm(velocity)
    heading/=max(np.linalg.norm(heading),1e-9)
    position=state[:3]-6*heading+np.array([0,0,2.3])
    look=state[:3]+4*heading-position
    forward=look/np.linalg.norm(look)
    right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right)
    down=np.cross(forward,right)
    renu=np.column_stack((forward,right,down))
    client.call('simSetObjectPose','ExternalCamera',pose(enu_to_ned(position,scene.origin),ENU_TO_NED@renu),True)
    # Also align a native image camera to the same world pose for evidence.
    rn=ENU_TO_NED@Rotation.from_quat(state[6:10][[1,2,3,0]]).as_matrix()@FRD_TO_FLU
    relative=rn.T@ENU_TO_NED@(position-state[:3])
    client.call('simSetCameraPose','back_center',pose(relative,rn.T@ENU_TO_NED@renu),'RaceDrone',False)
    return heading


def save_image(client,path):
    response=client.call('simGetImages',[dict(camera_name='back_center',image_type=0,
        pixels_as_float=False,compress=True)],'RaceDrone',False)[0]
    data=response['image_data_uint8']
    if response['width'] and data:
        path.write_bytes(bytes(data))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--outdir',required=True)
    parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--aperture-scale',type=float,default=1.)
    parser.add_argument('--spacing-scale',type=float,default=1.)
    parser.add_argument('--seed-speed',type=float)
    parser.add_argument('--seed-tail-scale',type=float)
    parser.add_argument('--arrival-seed-speed',type=float,default=6.5)
    parser.add_argument('--max-race-seconds',type=float,default=150.)
    args=parser.parse_args()
    out=Path(args.outdir);out.mkdir(parents=True,exist_ok=False)
    c=Client(timeout=60.);c.call('simPause',True);c.call('reset');c.call('simPause',True)
    clock_speed=float(json.loads((Path(__file__).parent/'settings.json').read_text())['ClockSpeed'])
    c.call('enableApiControl',True,'RaceDrone');c.call('armDisarm',True,'RaceDrone')
    scene=Scene(c,args.aperture_scale,args.spacing_scale);scene.build();scene.update(0.)
    scenario=manifest(Case(4.,4.,0.))
    scenario.update(aperture_scale=args.aperture_scale,additional_horizontal_spacing_scale=args.spacing_scale,
        anchors=[w.center0.tolist() for w in scene.track.windows],
        horizontal_spacing_scale=2.*args.spacing_scale,
        first_gate_sideways_shift_m=scenario['first_gate_sideways_shift_m']*args.spacing_scale,
        first_gate_shift_world=(np.asarray(scenario['first_gate_shift_world'])*args.spacing_scale).tolist(),
        workspace_lower=workspace_bounds(2.*args.spacing_scale)[0],
        workspace_upper=workspace_bounds(2.*args.spacing_scale)[1],
        scene_adaptation='Horizontal anchor offsets from the unchanged start uniformly scaled; height, orientation and 4Hz/4m motion unchanged',
        aperture_scaling='Only aperture local coordinates scaled; frame rod radius unchanged',
        scaled_apertures=[dict(name=w.name,kind=w.aperture.kind,radius=w.aperture.radius,
            vertices=None if w.aperture.physical_vertices is None else w.aperture.physical_vertices.tolist())
            for w in scene.track.windows])
    c.call('simRunConsoleCommand','ke CameraDirector setMode 3')
    env=c.call('simGetGroundTruthEnvironment','RaceDrone')
    model=GenericQuad(air_density=float(env['air_density']))
    radius,envelope=body_envelope(c)
    radius=float(np.ceil(radius*1000)/1000)
    envelope['planning_radius_m']=radius
    print('Native body envelope radius',radius,flush=True)
    print('Compiling native PWM tracking MPC',flush=True);mpc=MPC(model=model)
    mpc.motor=np.zeros(4)
    seed_parameters=planning_seed_parameters(model)
    if args.seed_speed is not None:
        if args.seed_speed<=0:raise ValueError('seed speed must be positive')
        seed_parameters['speed']=args.seed_speed
    if args.seed_tail_scale is not None:
        if args.seed_tail_scale<=0:raise ValueError('seed tail scale must be positive')
        seed_parameters['initial_tail_scale']=args.seed_tail_scale
    lo,hi=workspace_bounds(2.*args.spacing_scale);cfg=Config(radius=radius,workspace_lower=lo,workspace_upper=hi,
        passage_margin=.04,compiler_optimization='-O1',arrival_seed_speed=args.arrival_seed_speed,**seed_parameters)
    planner_class=Planner
    print('Native enclosing sphere radius',radius,'margin',cfg.passage_margin,flush=True)
    planner=None
    root=Path(__file__).resolve().parents[3]
    source_hashes={}
    for module in list(sys.modules.values()):
        file=getattr(module,'__file__',None)
        if not file:continue
        path=Path(file).resolve()
        if path.suffix!='.py' or not path.is_relative_to(root):continue
        relative=str(path.relative_to(root));source_hashes[relative]=hashlib.sha256(path.read_bytes()).hexdigest()
        destination=out/'source_snapshot'/relative;destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(path,destination)
    (out/'protocol.json').write_text(json.dumps(dict(source_sha256=source_hashes,
        scenario=scenario,vehicle=asdict(model),planner=asdict(cfg),
        clock_speed=clock_speed,body_envelope=envelope),indent=2))
    if not args.prepare_only:
        planner=planner_class(scene.track,cfg,model)
        for i in range(7):
            began=time.perf_counter();planner.prepare(i)
            print('Compiled gate',i+1,'seconds',time.perf_counter()-began,flush=True)
    logs=[];plans=[];attempts=[];contacts=[];timings=[]
    result=dict(status='running',simulation_mode='native_physics_paused_step_not_realtime',
        scenario=scenario,vehicle_parameters=asdict(model),
        prediction='GenericQuad rigid body, quadratic drag, 5ms PWM filter; direct native motor API',
        geometry='Native Cylinder sleeves; circles circumscribed by 24 sides',
        body_envelope=envelope,
        planning_config=asdict(cfg),
        clock_speed=clock_speed,requested_native_near_step_s=.003*clock_speed,
        safety_margin_basis='4cm development allowance: observed preparation tracking 7.2mm, nominal near-pose midpoint error 15.1mm at ClockSpeed 0.1; not a closed-loop bound',
        body_geometry='bounding_sphere_local_enclosure',
        collision_certificate=False,trajectory_replay=False)
    (out/'result.json').write_text(json.dumps(result,indent=2))
    raw=c.call('getMultirotorState','RaceDrone');base=raw['timestamp']
    head=physical_state(raw,scene.origin)[:3];ref=climb_reference(head,scene.track.start,7.)
    heading=np.array([0.,1.,0.]);phase='preparation';status='uncompleted';race_base=None
    command_at=-np.inf;old_motor_task=None;gate_updates=np.zeros(7);near=None
    progress=Progress(list(scene.track.windows),route_signs(scene.track),cfg.radius,cfg.frame_thickness)
    plan=None;request=True;retry_at=0.;last_print=-np.inf;last_image=-np.inf
    try:
        while True:
            raw=c.call('getMultirotorState','RaceDrone')
            t=(raw['timestamp']-base)*1e-9
            now=t if race_base is None else (raw['timestamp']-race_base)*1e-9
            state=physical_state(raw,scene.origin)
            # Native getMultirotorState leaves collision default-initialized!
            # simGetCollisionInfo reads the actual Pawn collision latch.
            collision=c.call('simGetCollisionInfo','RaceDrone')
            if collision['has_collided'] and collision['time_stamp']>(race_base if race_base is not None else base):
                # Ground contact only during physical takeoff is not a frame hit.
                if phase=='race' or not collision['object_name'].startswith('Ground'):
                    contacts.append(dict(time=now,phase=phase,collision=collision));status='collision';break
            if phase=='preparation' and t>7.:
                if np.linalg.norm(state[:3]-scene.track.start)<.15 and np.linalg.norm(state[3:6])<.3:
                    if args.prepare_only:status='native_takeoff_hover_passed';break
                    phase='race';race_base=raw['timestamp'];now=0.;command_at=-np.inf
                    mpc.controls=None;mpc.last_time=None;mpc.motor_time=0.
                    scene.update(0.);gate_updates[:]=0.
                    print('Race clock started',flush=True)
                    last_print=-np.inf;last_image=-np.inf
                elif t>20.:status='preparation_tracking_failed';break
            if phase=='race':
                exited=progress.observe(state[:3],now)
                if progress.violation:status=progress.violation;break
                if exited:
                    print('Whole body exited gate',progress.gate,'at',now,flush=True)
                    request=progress.gate<7;retry_at=now
                if request and now>=retry_at:
                    if plan is None:headstate=np.r_[state[:6],ENU_TO_NED.T@values(raw['kinematics_estimated']['linear_acceleration']),np.zeros(3)]
                    else:headstate=np.concatenate([plan.sample(now,d) for d in range(4)])
                    started=time.perf_counter()
                    try:
                        candidate=planner.solve(headstate,now,progress.gate)
                        plan=candidate;planner.commit(plan);ref=plan_reference(plan);request=False
                        plans.append(dict(made_at=plan.made_at,first_gate=plan.first_gate,durations=plan.durations.tolist(),
                            coefficients=plan.coefficients.tolist(),solve_ms=plan.solve_ms,violation=plan.violation))
                        print('Installed plan gate',progress.gate+1,'T',plan.durations,'ms',plan.solve_ms,flush=True)
                    except RuntimeError as e:
                        attempts.append(dict(time=now,gate=progress.gate,error=str(e),wall_ms=1000*(time.perf_counter()-started),
                            stats=planner.locals[progress.gate].solvers[0][0].value))
                        retry_at=now+.2
                        if plan is None:
                            ref=climb_reference(scene.track.start,scene.track.start,1.)
                if progress.gate==7 and np.linalg.norm(state[:3]-scene.track.goal)<.55 and np.linalg.norm(state[3:6])<1.:
                    status='completed';break
                if now>args.max_race_seconds:status='race_timeout';break
                # Near finite frames are updated each observed native step. Distant
                # frames update at 24ms: their stale visuals cannot be confused
                # with a continuously animated, perfectly synchronized scene.
                distances=[np.linalg.norm(state[:3]-w.center0) for w in scene.track.windows]
                near={i for i,d in enumerate(distances) if d<8.}
                update={i for i in range(7) if i in near or now-gate_updates[i]>=.024}
                # Midpoint gate pose halves zero-order-hold phase error.
                midpoint=.0015*clock_speed if near else .015*clock_speed
                began=time.perf_counter();scene.update(now+midpoint,update)
                timings.append(dict(time=now,wall_ms=1000*(time.perf_counter()-began),gates=sorted(update)))
                for i in update:gate_updates[i]=now+midpoint
            if now-command_at>=.04-1e-6:
                u=mpc.command(state,now,ref)
                old_motor_task=c.send('moveByMotorPWMs',*map(float,u),1.,'RaceDrone')
                command_at=now
                heading=camera(c,state,scene,progress.gate,heading)
            logs.append(dict(time=now,phase=phase,timestamp=raw['timestamp'],state=state.tolist(),
                acceleration_enu=(ENU_TO_NED.T@values(raw['kinematics_estimated']['linear_acceleration'])).tolist(),
                gate=progress.gate,collision=collision,reference=ref(np.array([now]),0)[0].tolist()))
            if now-last_image>.25:
                save_image(c,out/f'view_{phase}_{now:08.3f}.png');last_image=now
            if now-last_print>=1.:
                print(phase,'t',round(now,3),'gate',progress.gate,'position',np.round(state[:3],2),
                    'error',round(np.linalg.norm(state[:3]-ref(np.array([now]),0)[0]),3),flush=True)
                (out/'progress.json').write_text(json.dumps(dict(phase=phase,time=now,gate=progress.gate,
                    position=state[:3].tolist(),wall_time=time.time()),indent=2));last_print=now
            # continueForTime is a wall timer; ClockSpeed scales the native
            # physics timestep, not the course's 4Hz simulation-time motion.
            c.call('simContinueForTime',(.003 if near else .03) if phase=='race' else .042/clock_speed)
            # Completed/cancelled async command replies are no longer needed.
            c.responses={k:v for k,v in c.responses.items() if k>=c.serial-200}
    except KeyboardInterrupt:
        status='development_interrupted'
    except Exception as e:
        status='execution_exception';result['exception']=repr(e)
        raise
    finally:
        c.call('simPause',True)
        result.update(status=status,phase=phase,completed=status=='completed',
            source_changes=[p for p,h in source_hashes.items() if not (root/p).exists()
                or hashlib.sha256((root/p).read_bytes()).hexdigest()!=h],
            complete_race_time_s=now if status=='completed' else None,
            last_time_s=now,body_exits=progress.exits,crossings=progress.crossings,
            contacts=contacts,plans=len(plans),planning_failures=len(attempts),
            requested_near_gate_step_s=.003*clock_speed,far_gate_update_s=.024,
            native_step_is_exact=False,
            maximum_observed_race_step_s=max((
                (b['timestamp']-a['timestamp'])*1e-9 for a,b in zip(logs,logs[1:])
                if a['phase']==b['phase']=='race'),default=None),
            actual_timestep_s='Recorded native timestamps, expected 3ms; verify gaps before accepting result')
        for name,data in [('result',result),('flight',logs),('plans',plans),('planning_attempts',attempts),
                          ('mpc',mpc.logs),('gate_updates',timings)]:
            (out/(name+'.json')).write_text(json.dumps(data,indent=2))
        save_image(c,out/'final_view.png')
        print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
