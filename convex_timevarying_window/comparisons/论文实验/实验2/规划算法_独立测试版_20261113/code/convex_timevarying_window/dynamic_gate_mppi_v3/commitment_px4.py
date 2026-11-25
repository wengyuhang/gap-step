"""Experimental closed-loop CTBR passage runner; independent of old motors."""
import argparse
import copy
from dataclasses import replace,asdict
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import threading
import time
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_dynamic_seven_window_gazebo.online_safe_mppi_px4_experiment import CONTAINER,PARTITION,CONTACT_TOPICS
from convex_dynamic_seven_window_gazebo.px4_togt_nmpc_experiment import parse_contacts
from convex_timevarying_window.receding_gate_racing.scenario import vary_track
from .ctbr_probe import Session
from .commitment_gpu import CommitmentController
from .feedback_maneuver import FeedbackManeuver
from .config import Config
from .geometry import GateProgress,Geometry
from .reference import Planner,AsyncPlanner
from .scheduler import Scheduler
from .diagnostic import pvaj_from_state
from .passage_commitment import activation_phase_matches
from .execution_prefix import prefix_violation,feedback_prefix,prefix_task_violation


def run(args):
    track,opt=build_track();vary_track(track,args.phase)
    scene=track
    if args.gates==1:track=replace(track,windows=(track.windows[track.order[0]],),order=(0,))
    cfg=replace(Config(),horizon=2.5,samples=args.samples,axis_workspace_bounds=True)
    # Feedback needs correction authority; reserve is algorithm-wide, not
    # gate-specific. Physical acceptance limits remain cfg, unchanged.
    planning_cfg=replace(cfg,acceleration=cfg.acceleration-1.,tilt=cfg.tilt-4.,
                         feedback_geometry_reserve=.1,staged_entry=args.staged_entry,
                         terminal_velocity_cost=args.terminal_velocity_cost,passage_event_tail=True)
    ctrl=CommitmentController(track,planning_cfg,args.seed);planner=Planner(track,opt);worker=AsyncPlanner(track,opt)
    actual_geometry=Geometry(track,cfg)
    s=Session(args.outdir,scene,consistent_velocity=True);hashes={}
    for source in Path(__file__).parent.rglob('*'):
        rel=source.relative_to(Path(__file__).parent)
        if source.suffix not in ('.py','.cpp') or any(x in rel.parts for x in ('results','build','install','log')):continue
        target=args.outdir/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,target);hashes[str(rel)]=hashlib.sha256(source.read_bytes()).hexdigest()
    contacts=[]
    for i,topic in enumerate(CONTACT_TOPICS):
        path=args.outdir/f'gate_{i+1:02d}_contacts.pbtxt';stream=path.open('w')
        proc=subprocess.Popen(['docker','exec','-e',f'GZ_PARTITION={PARTITION}',CONTAINER,'gz','topic','-e','-t',topic],stdout=stream,stderr=subprocess.DEVNULL)
        contacts.append((proc,stream,path))
    progress=GateProgress(track,cfg);scheduler=Scheduler(cfg);ref=None;pending=[]
    active=None;solves=[];updates=[];crossings=[];rows=[];activations=[];policy_history=[];watchdog=[];failure=None;completed=False
    live=(args.outdir/'solves_live.jsonl').open('w',buffering=1)
    now=0.;verified_until=0.;initial_reference_ms=None
    sent_command=None;sent_at=None
    planning_lock=threading.RLock()
    def receive_reference(x,at):
        nonlocal ref
        with planning_lock:
            returned=worker.poll()
            if returned is None:return
            _,candidate=returned;req=scheduler.running;accepted=False;validation={}
            if req is not None and candidate is not None and not isinstance(candidate,dict):
                err=ctrl.tracking_error(x,at,candidate)
                validation={**err,'solve_ms':candidate.solve_ms,'elapsed':at-req.started}
                accepted=scheduler.finish(req,at,progress.first,err['position_error'],err['velocity_error'],0.)
            else:scheduler.failed(req)
            if accepted:ref=candidate
            updates.append(dict(time=at,accepted=accepted,validation=validation,error=candidate if isinstance(candidate,dict) else None))

    def send_feedback(x,at):
        nonlocal active,verified_until,sent_command,sent_at
        # Receive completed TOGT work at feedback rate. Waiting for the next
        # 50--70 ms GPU cycle could discard a result actually ready at 75 ms.
        receive_reference(x,at)
        while pending and pending[0].made_at<=at:
            selection=pending.pop(0)
            phase_ok=bool(activation_phase_matches(progress,x,at,selection.expected_first,selection.expected_phase))
            accepted=phase_ok and ctrl.publish(selection,x,at,progress.first)
            errors=FeedbackManeuver(selection,progress.first).errors(x,at)
            activations.append(dict(time=at,anchor=selection.made_at,accepted=accepted,phase_match=phase_ok,errors=errors))
            if accepted:policy_history.append(selection)
            if accepted:active=ctrl.maneuver
        if active is None:
            command=ctrl.rollout.feedback(x,track.start,np.zeros(3),np.zeros(3))
            sent_command=command;sent_at=at
            return command
        if at>verified_until:raise RuntimeError(f'feedback_revalidation_lease_expired: now={at:.6f}, verified_until={verified_until:.6f}')
        if verified_until-at<.04:
            # Short-horizon execution validation must not wait for a long
            # GPU solve. Recheck the EXISTING feedback, not an unchecked
            # held command, with time left before the previous lease ends.
            tick=time.perf_counter();states,times=feedback_prefix(ctrl.rollout,active,x,at,previous=sent_command)
            bad=prefix_violation(ctrl.rollout,states,times,cfg,actual_geometry)
            bad=bad or prefix_task_violation(track,progress,states,times,cfg)
            elapsed=1000*(time.perf_counter()-tick)
            watchdog.append(dict(time=at,cycle_ms=elapsed,violation=bad))
            if bad is not None or elapsed>=40:raise RuntimeError(f'feedback_watchdog_{bad or "deadline"}')
            verified_until=max(verified_until,at+.1)
        dt=.01 if sent_at is None else max(0.,min(.02,at-sent_at))
        command=active.command(x,at,sent_command,dt)
        sent_command=command;sent_at=at
        return command
    try:
        s.prepare();s.feedback=send_feedback
        now,x,_=s.read();last=x.copy();last_t=now
        s.drain_measurements()
        initial_tick=time.perf_counter()
        ref=planner.solve(pvaj_from_state(ctrl.rollout,x,send_feedback(x,now)),now,0)
        initial_reference_ms=1000*(time.perf_counter()-initial_tick)
        next_control=now
        while now<args.duration:
            now,x,age=s.read()
            if s.failure:failure=s.failure;break
            if np.any(x[:3]<cfg.lower) or np.any(x[:3]>cfg.upper):failure='workspace';break
            tilt=np.degrees(np.arccos(np.clip(1-2*(x[7]**2+x[8]**2),-1,1)))
            if tilt>cfg.tilt:failure='actual_tilt_limit';break
            if np.linalg.norm(x[3:6])>cfg.speed:failure='actual_speed_limit';break
            # Check every received position sample. The optimization loop is
            # NOT the geometry sampling clock; joining only its endpoints
            # created 50--70 ms artificial sweep margins at the gate.
            samples=[(t,z) for t,z in s.drain_measurements() if t>last_t]
            if samples:
                times=np.array([last_t,*[v[0] for v in samples]])
                states=np.array([last,*[v[1] for v in samples]])
                clear=actual_geometry.interval_clearances(states[:,:3],states[:,3:6],times)[0]
                for i,(t,z) in enumerate(samples):
                    if clear[i]<=0:failure='actual_interval_clearance';break
                    if progress.update(last[:3],last_t,z[:3],t,True):
                        crossings.append(dict(window_index=progress.first-1,time=t,position=z[:3].tolist()))
                        scheduler.gate_passed(progress.first);print('CTBR PASS',progress.first,t,flush=True)
                    last=z.copy();last_t=t
                    if progress.violation:failure=progress.violation;break
                if failure:break
                now,x=float(times[-1]),states[-1]
            if now>=next_control:
                tick=time.perf_counter();lead=.08;anchor=now+lead;prediction=x.copy();pred=copy.copy(progress)
                prefix=[prediction.copy()];queued_policies=tuple(pending)
                policy=active
                previous=sent_command
                for k in range(20):
                    at=now+.005*k
                    for queued in queued_policies:
                        if queued.made_at<=at:policy=FeedbackManeuver(queued,progress.first)
                    u=ctrl.rollout.feedback(prediction,track.start,np.zeros(3),np.zeros(3)) if policy is None else policy.command(prediction,at,previous,.005)
                    old=prediction;prediction=ctrl.rollout.integrate(prediction,u,.005)
                    pred.update(old[:3],at,prediction[:3],at+.005,True)
                    prefix.append(prediction.copy())
                    previous=u
                    if k==15:anchor_state=prediction.copy();anchor_progress=copy.copy(pred);anchor_input=u.copy()
                if pred.violation:
                    np.savez_compressed(args.outdir/'rejected_prefix.npz',states=prefix,times=now+np.arange(21)*.005)
                    failure='pending_'+pred.violation;break
                bad=prefix_violation(ctrl.rollout,prefix,now+np.arange(21)*.005,cfg,actual_geometry)
                if bad is not None:
                    np.savez_compressed(args.outdir/'rejected_prefix.npz',states=prefix,times=now+np.arange(21)*.005)
                    failure='execution_prefix_'+bad;break
                # Renew only after checking the actual feedback being executed,
                # even when the long-horizon optimizer finds no new candidate.
                verified_until=max(verified_until,now+.1)
                prediction,pred,u=anchor_state,anchor_progress,anchor_input
                risk=ctrl.reference_risk(x,now,progress.first,ref,progress.crossing)
                warning=ctrl.reference_warning(x,now,progress.first,ref,progress.crossing)
                if warning:scheduler.flag_prediction_warning()
                errors=ctrl.tracking_error(x,now,ref)
                if errors['tracking_failed']:scheduler.flag_tracking_error()
                selection=ctrl.select(prediction,anchor,pred.first,pred.crossing,risk,ref,u)
                scheduler.set_risk(ctrl.last_diagnostics.get('avoidance',True))
                if ctrl.last_diagnostics.get('replan_requested'):scheduler.flag_prediction_warning()
                returned_time,actual,_=s.read();ms=1000*(time.perf_counter()-tick)
                accepted=bool(selection is not None and returned_time<anchor and ms<100 and s.failure is None)
                record=dict(time=now,accepted=accepted,late=bool(returned_time>=anchor),gate=progress.first,
                            phase=progress.crossing,verified_until=verified_until,reference_warning=warning,**ctrl.last_diagnostics)
                record['cycle_ms']=ms;solves.append(record);live.write(json.dumps(record)+'\n')
                if accepted:
                    selection.expected_first=pred.first;selection.expected_phase=pred.crossing
                    pending.append(selection)
                    # Full-state rollouts from predicted anchor admitted a
                    # successor; cover handover only, not a long blind cache.
                    verified_until=max(verified_until,anchor+.1)
                elif active is None:failure='no_initial_admissible_policy';break
                next_control=now+.02
                now,x,_=s.read()
            with planning_lock:
                if active is not None and worker.ready and scheduler.running is None:
                    now,x,_=s.read()
                    cache=active.selection;i=max(0,int(np.floor((now-cache.made_at)/.01)))
                    if i+10<len(cache.states):
                        times=cache.made_at+np.arange(i,len(cache.states))*.01
                        req=scheduler.observe(now,times,cache.clearances[i:],not progress.computation_allowed(x[:3],now),progress.first)
                        if req is not None:worker.submit(len(updates),pvaj_from_state(ctrl.rollout,x,sent_command),req.anchor,req.first_gate)
            rows.append([now,*x[:6],0.,0.,0.,*x[:3],progress.first,0.])
            if progress.first==len(track.order):
                if args.gates==1 or (np.linalg.norm(x[:3]-track.goal)<.55 and np.linalg.norm(x[3:6])<1):completed=True;break
            time.sleep(.001)
        if not completed and failure is None:failure='timeout'
    except Exception as exc:failure=repr(exc)
    finally:
        now,terminal,_=s.read();s.close();worker.close();live.close()
        for proc,stream,path in contacts:
            proc.terminate()
            try:proc.wait(2)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
            stream.close()
    np.savez_compressed(args.outdir/'activated_policies.npz',**{
        f'{i}_{key}':value for i,sel in enumerate(policy_history) for key,value in
        (('time',sel.made_at),('states',sel.states),('controls',sel.controls))})
    contact_info=[parse_contacts(path) for _,_,path in contacts]
    np.savetxt(args.outdir/'telemetry.csv',rows,delimiter=',',header='t,x,y,z,vx,vy,vz,ax,ay,az,rx,ry,rz,gate,error',comments='')
    result=dict(method='ctbr_feedback_passage_experimental',completed=completed,failure=failure,
                crossings=crossings,last_time_s=now,flight_time_s=now if completed else None,
                collision_detected=any(c['contact_pair_count'] for c in contact_info),contacts=contact_info,
                solves=solves,reference_updates=updates,initial_reference_ms=initial_reference_ms,activations=activations,feedback_revalidations=watchdog,source_sha256=hashes,
                config={**asdict(cfg),'workspace_lower':list(cfg.lower),'workspace_upper':list(cfg.upper)},
                planning_config=asdict(planning_cfg),
                velocity_source='reported_velocity_plus_observed_position_correction_rate',
                phase=args.phase,layout=0.,motion_rate=1.,translation_scale=1.,rotation_scale=1.,
                origin_enu_m=s.origin.tolist(),goal_error_m=float(np.linalg.norm(terminal[:3]-track.goal)),
                goal_speed_mps=float(np.linalg.norm(terminal[3:6])),test_gates=args.gates,
                evidence='PX4 CTBR closed loop; empirical inner-loop model, not robust safety certificate')
    (args.outdir/'result.json').write_text(json.dumps(result,indent=2,default=lambda v:v.item() if isinstance(v,np.generic) else str(v))+'\n')
    print({k:result[k] for k in ('completed','failure','last_time_s','crossings','collision_detected')},flush=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--duration',type=float,default=60.);p.add_argument('--gates',type=int,choices=(1,7),default=7)
    p.add_argument('--phase',type=float,default=.4);p.add_argument('--seed',type=int,default=0)
    p.add_argument('--staged-entry',action='store_true');p.add_argument('--terminal-velocity-cost',action='store_true')
    p.add_argument('--samples',type=int,default=16384);run(p.parse_args())


if __name__=='__main__':main()
