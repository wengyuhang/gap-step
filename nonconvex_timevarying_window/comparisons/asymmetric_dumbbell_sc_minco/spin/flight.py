"""Fly the accepted stored spin reference using real MPC -> PX4 -> x500."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace
import numpy as np
from .scenario import build_case, local_coordinates
from . import gazebo_scene as scene
from ..run import save_json


WORLD_TRANSLATION = np.array([-16., 10., 0.])


def align_with_launch_origin(track, plan):
    """Rigidly translate the whole task, preserving motion and relative path.

    The existing PX4 launcher starts at x=-16,y=4. Put the airborne start
    above that launch pad; do not demand a lateral transfer during takeoff.
    """
    from dataclasses import replace
    for w in track.windows:
        w.center0=w.center0+WORLD_TRANSLATION
    track=type(track)(track.name,track.start+WORLD_TRANSLATION,
                      track.goal+WORLD_TRANSLATION,track.windows,track.order)
    coefficients=plan.coefficients.copy()
    coefficients[:,0,:]+=WORLD_TRANSLATION
    return track,replace(plan,coefficients=coefficients)


def audit(out, track, plan):
    from pyulog import ULog
    from scipy.spatial.transform import Rotation
    from convex_timevarying_window.three_phase_mppi.report import continuous_quaternions
    raw=json.loads((out/'result.json').read_text())
    log=ULog(str(out/'flight.ulg'))
    d=log.get_dataset('vehicle_local_position_groundtruth').data
    a=log.get_dataset('vehicle_attitude_groundtruth').data
    absolute=d['timestamp']*1e-6
    selected=(absolute>=30.) & (absolute<=30.+raw['last_time_s'])
    absolute=absolute[selected];times=absolute-30.
    if len(times)<3: raise RuntimeError('Insufficient actual flight ground truth')
    quats=continuous_quaternions(np.column_stack([a[k] for k in ('q[1]','q[2]','q[3]','q[0]')]))
    quats=np.column_stack([np.interp(absolute,a['timestamp']*1e-6,quats[:,i]) for i in range(4)])
    p=np.column_stack([d[k][selected] for k in ('x','y','z')])
    p+=Rotation.from_quat(quats).apply(np.tile([0.,0.,-.24],(len(times),1)))
    p=p[:,[1,0,2]]*[1,1,-1]
    w=track.windows[0];q=local_coordinates(w,p,times)
    sign=np.sign((track.start-w.center0)@w.state_at(0.)[1][:,2]);z=sign*q[:,2]
    events=[]
    for k in np.flatnonzero(z[:-1]*z[1:]<0):
        alpha=abs(z[k])/(abs(z[k])+abs(z[k+1]))
        t=float(times[k]+alpha*(times[k+1]-times[k]))
        point=(1-alpha)*p[k]+alpha*p[k+1]
        local,_=w.world_to_local(point,t)
        events.append(dict(time_s=t,forward=bool(z[k]>0),inside=w.aperture.contains(local),
                           local_point_m=local.tolist(),world_point_m=point.tolist()))
    once=len(events)==1 and events[0]['forward'] and events[0]['inside']
    h=raw['config']['radius']+raw['config']['frame_thickness']
    body_exit=bool(once and np.any((times>events[0]['time_s']) & (z<=-h)))
    error=np.linalg.norm(p-plan.sample_many(times),axis=1)
    goal_error=float(np.linalg.norm(p[-1]-track.goal))
    collision=raw['collision_detected']
    passed=bool(raw['completed'] and once and body_exit and goal_error<.55 and not collision)
    velocity=np.gradient(p,times,axis=0)
    clearance=np.hypot(w.aperture.boundary_distance(q[:,:2]),q[:,2])-h
    result=dict(verdict='collision' if collision else 'completed_no_contact' if passed else 'not_completed',
        passed=passed,completed=raw['completed'],failure=raw['failure'],collision_detected=collision,
        contacts=raw['contacts'],flight_time_s=raw['flight_time_s'] if passed else None,
        raw_completion_time_s=raw['flight_time_s'],nominal_time_s=float(plan.durations.sum()),
        crossings=events,body_exit_confirmed=body_exit,truth_goal_error_m=goal_error,
        tracking_error_max_m=float(error.max()),tracking_error_rms_m=float(np.sqrt(np.mean(error**2))),
        sampled_sphere_frame_clearance_m=float(clearance.min()),truth_step_median_s=float(np.median(np.diff(times))),
        peak_speed_mps=float(np.linalg.norm(velocity,axis=1).max()),
        evidence='actual PX4 CTBR MPC flight; ULog body centre and Gazebo contacts; not a certificate or pose replay')
    np.savez_compressed(out/'mechanism_trace.npz',times=times,positions=p,local_positions=q,
                        reference=plan.sample_many(times),tracking_error=error,clearance=clearance)
    save_json(out/'flight_audit.json',result)
    return result


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--plan-dir',type=Path,required=True)
    p.add_argument('--outdir',type=Path,required=True)
    args=p.parse_args();src=args.plan_dir.resolve();out=args.outdir.resolve()
    raw=json.loads((src/'result.json').read_text())
    if not raw['accepted']: raise ValueError('Need an accepted stored reference')
    if out.exists(): raise FileExistsError(out)
    track=build_case(raw['spin_hz'])
    from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
    data=np.load(src/'plan.npz')
    plan=Plan(0.,0,data['durations'],data['coefficients'],data['x'],0.,0.,'frozen_spin_reference',1)
    track,plan=align_with_launch_origin(track,plan)
    np.testing.assert_allclose(plan.sample(0.),track.start,atol=1e-7)
    np.testing.assert_allclose(plan.sample(float(plan.durations.sum())),track.goal,atol=1e-7)
    from convex_timevarying_window.simple_two_gate_soft_only_mpc import flight,px4
    from convex_timevarying_window.global_gate_minco_probe.flight import Reference
    from convex_timevarying_window.single_gate_stop_racing.planner import Config
    from convex_timevarying_window.single_gate_stop_racing.events import Progress
    from convex_timevarying_window.togt_style_mpc.aligned_session import Session
    from convex_timevarying_window.togt_style_mpc.shooting_controller import MPC
    from convex_timevarying_window.comparisons.single_gate_swing.contacts import parse_contacts

    class Worker:
        def __init__(self,track,cfg):
            self.geometry=SimpleNamespace(cfg=cfg,windows=track.windows,
                signs=[float(np.sign((track.start-w.center0)@w.state_at(0.)[1][:,2])) for w in track.windows])
            self.startup=dict(frozen_reference_source=str(src),new_planning=False)
            self.busy=False;self.metrics=[]
        def poll(self): return None
        def close(self): pass

    class Scheduler:
        def __init__(self,**kw):
            self.log=[];self.error_high=kw['error_high'];self.error_low=kw['error_low'];self.retry_delay=.2
        def observe(self,*args): pass
        def due(self,*args): return False

    class CheckedSession(Session):
        def convert(self,state):
            # PX4 map distortion is anchored at the actual launch/GPS origin,
            # not at this experiment's distant airborne race-start point.
            self.metric_anchor=np.array([-16.,4.,-5.76])
            return super().convert(state)
        def __init__(self,*a,**kw):
            super().__init__(*a,**kw)
            publishers={}
            for topic in px4.CONTACT_TOPICS:
                info=subprocess.run(['docker','exec','-e','GZ_PARTITION='+scene.PARTITION,
                    scene.CONTAINER,'gz','topic','-i','-t',topic],check=True,capture_output=True,text=True,timeout=8)
                publishers[topic]=info.stdout
                if 'gz.msgs.Contacts' not in info.stdout or 'Publishers [Address, Message Type]:' not in info.stdout:
                    self.close();raise RuntimeError('Missing contact publisher: '+topic)
            save_json(self.out/'contact_publishers.json',publishers)
        def prepare(self):
            super().prepare()
            _,state,_=self.read()
            error=float(np.linalg.norm(state[:3]-self.track.start))
            save_json(self.out/'race_start_check.json',dict(position_error_m=error,
                position_m=state[:3].tolist(),speed_mps=float(np.linalg.norm(state[3:6]))))
            if error>.25: raise RuntimeError('race_start_not_reached')

    flight.Worker=Worker;flight.Scheduler=Scheduler
    flight.Reference=lambda start,count:Reference(start,plan)
    flight.variant_config=lambda variant:Config(full_task_error_recovery=False)
    flight.build_case=lambda case:(track,None)
    flight.launch=scene.launch;flight.manifest=lambda case:scene.manifest(track)
    flight.MAX_FLIGHT_TIME=12
    px4.MPC=MPC;px4.Session=CheckedSession;px4.Progress=Progress
    px4.CONTACT_TOPICS=(scene.FRAME_TOPIC,scene.GROUND_TOPIC);px4.parse_contacts=parse_contacts
    sys.argv=[flight.__name__,'--method','ours','--frequency','3','--amplitude','2','--phase','0','--outdir',str(out)]
    frozen=hashlib.sha256((src/'plan.npz').read_bytes()).hexdigest()
    try: flight.main()
    finally:
        subprocess.run(['docker','exec','-e','GZ_PARTITION='+scene.PARTITION,scene.CONTAINER,
            'gz','service','-s','/world/convex_seven_dynamic_px4_togt/control','--reqtype','gz.msgs.WorldControl',
            '--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'],capture_output=True,timeout=5)
        if out.exists():
            shutil.copyfile(src/'plan.npz',out/'frozen_plan.npz')
            save_json(out/'reference_provenance.json',dict(source=str(src),sha256=frozen,
                unchanged=hashlib.sha256((src/'plan.npz').read_bytes()).hexdigest()==frozen,
                new_planning=False,retiming=False,pose_replay=False,
                rigid_world_translation_m=WORLD_TRANSLATION.tolist(),
                translation_applied_to_start_goal_gate_and_reference=True))
            for source in Path(__file__).parent.rglob('*'):
                if source.suffix in ('.py','.cc') or source.name=='CMakeLists.txt':
                    dst=out/'adapter_snapshot'/source.relative_to(Path(__file__).parent)
                    dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,dst)
        if (out/'result.json').exists():
            result=json.loads((out/'result.json').read_text())
            contacts=[parse_contacts(out/f'gate_{i:02d}_contacts.pbtxt',30.+result['last_time_s']) for i in (1,2)]
            result.update(method='component_sc_pv_minco_spin_frozen_mpc',spin_hz=raw['spin_hz'],
                contacts=contacts,collision_detected=any(c['contact_pair_count'] for c in contacts),
                planning_scheduler='disabled: accepted frozen single-gate reference',
                new_planning=False,closed_loop_certificate=False)
            save_json(out/'result.json',result)
            print('SPIN_FLIGHT_AUDIT',json.dumps(audit(out,track,plan)),flush=True)


if __name__=='__main__': main()
