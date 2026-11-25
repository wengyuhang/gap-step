"""One physical x500 flight for the frozen pendulum condition."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from . import scenario
from .motion import Case
from .planner import Config
from .worker import Worker
from convex_timevarying_window.comparisons.single_gate_swing.contacts import parse_contacts
from convex_timevarying_window.togt_style_mpc.shooting_controller import MPC
from convex_timevarying_window.comparisons.single_gate_constant_velocity.flight import Scheduler


def main():
    p=argparse.ArgumentParser();p.add_argument('--angle',type=float,required=True);p.add_argument('--method',choices=['ours','center','high_mpc_author'],required=True);p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    out=a.outdir.resolve();case=Case(a.angle);track,_=scenario.build_case(case)
    from convex_timevarying_window.simple_two_gate_soft_only_mpc import flight,px4
    from convex_timevarying_window.togt_style_mpc.aligned_session import Session
    from convex_timevarying_window.single_gate_stop_racing.events import Progress
    flight.Worker=Worker;flight.Scheduler=Scheduler;px4.MPC=MPC
    if a.method=='high_mpc_author':
        from . import high_mpc_adapter as author
        author.MPC.window=track.windows[0];flight.Worker=author.Worker;flight.Scheduler=author.Scheduler;px4.MPC=author.MPC
    flight.variant_config=lambda variant:Config(free_points=variant=='ours',full_task_error_recovery=False)
    flight.build_case=lambda unused:(track,Config());flight.launch=scenario.launch;flight.manifest=lambda unused:scenario.manifest(case);flight.MAX_FLIGHT_TIME=40
    px4.FINISH_POSITION_TOLERANCE=.50
    class CheckedSession(Session):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs);publishers={}
            for topic in (scenario.GATE_TOPIC,scenario.GROUND_TOPIC):
                info=subprocess.run(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition','convex_seven_togt_experiment','gz','topic','-i','-t',topic],check=True,capture_output=True,text=True,timeout=8)
                publishers[topic]=info.stdout
                if 'Publishers [Address, Message Type]:' not in info.stdout or 'gz.msgs.Contacts' not in info.stdout:
                    self.close();raise RuntimeError('contact publisher missing:'+topic)
            (self.out/'contact_publishers.json').write_text(json.dumps(publishers,indent=2)+'\n')
            self.pose_stream=(self.out/'gate_pose.pbtxt').open('w')
            self.pose_reader=subprocess.Popen(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition','convex_seven_togt_experiment','gz','topic','-e','-t','/world/convex_seven_dynamic_px4_togt/pose/info'],stdout=self.pose_stream,stderr=subprocess.DEVNULL)
        def close(self):
            try:super().close()
            finally:
                if hasattr(self,'pose_reader'):
                    self.pose_reader.terminate()
                    try:self.pose_reader.wait(timeout=2)
                    except subprocess.TimeoutExpired:self.pose_reader.kill();self.pose_reader.wait()
                    self.pose_stream.close()
    px4.Session=CheckedSession;px4.Progress=Progress;px4.CONTACT_TOPICS=(scenario.GATE_TOPIC,scenario.GROUND_TOPIC);px4.parse_contacts=parse_contacts
    sys.argv=[flight.__name__,'--method','ours' if a.method=='ours' else 'center','--frequency','1','--amplitude','0','--phase','0','--outdir',str(out)]
    try:flight.main()
    finally:
        subprocess.run(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition','convex_seven_togt_experiment','gz','service','-s','/world/convex_seven_dynamic_px4_togt/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'],capture_output=True,timeout=5)
        if (out/'result.json').exists():
            raw=json.loads((out/'result.json').read_text());contacts=[parse_contacts(out/f'gate_{i:02d}_contacts.pbtxt',30+raw['last_time_s']) for i in (1,2)]
            raw.update(case=asdict(case),method='pendulum_'+a.method,comparison_variant=a.method,contacts=contacts,collision_detected=any(c['contact_pair_count'] for c in contacts),
                physics_step_s=scenario.PHYSICS_STEP,local_safety='shared cubic pendulum table and local Bernstein circle-outside enclosure' if a.method!='high_mpc_author' else 'author controller, no added safety filter',
                task_deadline_s=40.,online_finish_position_m=.50,truth_finish_position_m=.55,size_matching='propeller ratio, full-body ratio not verified',closed_loop_certificate=False)
            (out/'result.json').write_text(json.dumps(raw,indent=2)+'\n')
            from convex_timevarying_window.comparisons.single_gate_swing.audit import audit
            result=audit(out,track=track,condition={'angle_deg':a.angle,'peak_gate_speed_mps':scenario.manifest(case)['maximum_table_speed_mps']})
            if a.method=='high_mpc_author' and result['verdict']=='online_no_accepted_plan':result.update(verdict='not_completed',no_separate_trajectory_plan_required=True)
            trace=dict(np.load(out/'mechanism_trace.npz'));times=trace['times'];window=track.windows[0]
            rotations=np.array([window.state_at(float(t))[1] for t in times])
            trace['local_positions_initial_basis']=trace['local_positions'].copy()
            trace['local_positions']=np.einsum('ni,nij->nj',trace['positions']-trace['gate_centers'],rotations)
            trace['relative_velocity_local']=np.einsum('ni,nij->nj',trace['velocity']-trace['gate_velocity'],rotations)
            np.savez_compressed(out/'mechanism_trace.npz',**trace)
            for event in result['crossings']:
                t=event['time_s'];position=np.array([np.interp(t,times,trace['positions'][:,k]) for k in range(3)])
                event['point_local_m']=window.world_to_local(position,t)[0].tolist()
                velocity=np.array([np.interp(t,times,trace['velocity'][:,k]) for k in range(3)])
                event['relative_lateral_speed_mps']=float((velocity-window.motion.translation(t)[1])@window.state_at(t)[1][:,1])
            result['local_frame_definition']='instantaneous moving gate frame; circular aperture rotation invariant'
            from .pose_audit import audit as pose_audit
            try:result['gate_pose_audit']=pose_audit(out,track.windows[0],raw['last_time_s'])
            except Exception as exc:
                result.update(passed=False,verdict='evidence_failure',flight_time_s=None,gate_pose_error=repr(exc))
                (out/'comparison_audit.json').write_text(json.dumps(result,indent=2)+'\n')
                raise
            (out/'comparison_audit.json').write_text(json.dumps(result,indent=2)+'\n')
            print('AUDIT',json.dumps(result),flush=True)

if __name__=='__main__':main()
