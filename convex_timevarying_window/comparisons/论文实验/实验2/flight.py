"""Full-course reference execution; no replanning for free/centre methods."""
import argparse,json,subprocess,sys
from dataclasses import asdict,replace
from pathlib import Path
import numpy as np
from types import SimpleNamespace
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
from convex_timevarying_window.global_gate_minco_probe.flight import Reference as FrozenReference,Worker as FrozenWorker,Scheduler as DisabledScheduler,Progress
from convex_timevarying_window.single_gate_stop_racing.planner import Config
from convex_timevarying_window.single_gate_stop_racing.scheduler import Scheduler as ExitScheduler
from convex_timevarying_window.togt_style_mpc.shooting_controller import MPC
from convex_timevarying_window.togt_style_mpc.aligned_session import Session
from convex_timevarying_window.comparisons.single_gate_swing.contacts import parse_contacts
from . import scenario
from .scenario import Case


def main():
    p=argparse.ArgumentParser();p.add_argument('--layout',required=True);p.add_argument('--level',required=True);p.add_argument('--method',choices=('ours','center','wang'),required=True);p.add_argument('--plan-dir',type=Path);p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    case=Case(a.layout,a.level);track,_=scenario.build_case(case);out=a.outdir.resolve();plan=None;source=None
    from convex_timevarying_window.simple_two_gate_soft_only_mpc import flight,px4
    if a.method!='wang':
        if a.plan_dir is None:raise ValueError('global methods require full plan')
        source=json.loads((a.plan_dir/'result.json').read_text());assert source['found']
        assert source['case']==asdict(case) and source['method']==a.method
        plan=Plan(0.,0,np.array(source['durations']),np.array(source['coefficients']),np.array(source['variables']),1000*source['solve_s'],0.,'frozen_global_qualified',7)
        flight.Worker=FrozenWorker;flight.Scheduler=DisabledScheduler;flight.Reference=lambda start,count:FrozenReference(start,plan)
        flight.variant_config=lambda variant:Config(**source['config'])
    else:
        from .worker import Worker
        from .baseline import Config as WangConfig
        flight.Worker=Worker;flight.Scheduler=ExitScheduler
        flight.variant_config=lambda variant:WangConfig()
    old_replace=px4.replace
    def correct_horizon(cfg,**kwargs):
        kwargs['horizon']=1 if a.method=='wang' else 7
        return replace(cfg,**kwargs)
    px4.replace=correct_horizon
    flight.build_case=lambda unused:(track,None);flight.launch=scenario.launch;flight.manifest=lambda unused:scenario.manifest(case);flight.MAX_FLIGHT_TIME=180
    px4.WORKSPACE_LOWER=(-100.,-120.,.65);px4.WORKSPACE_UPPER=(150.,120.,20.)
    px4.MPC=MPC;px4.Progress=Progress;px4.CONTACT_TOPICS=scenario.contact_topics(track);px4.parse_contacts=parse_contacts
    class CheckedSession(Session):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs);publishers={}
            for topic in scenario.contact_topics(track):
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
    px4.Session=CheckedSession
    sys.argv=[flight.__name__,'--method','ours' if a.method=='ours' else 'center','--frequency','1','--amplitude','0','--phase','0','--outdir',str(out)]
    try:flight.main()
    finally:
        px4.replace=old_replace
        subprocess.run(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition','convex_seven_togt_experiment','gz','service','-s','/world/convex_seven_dynamic_px4_togt/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'],capture_output=True,timeout=5)
        if (out/'result.json').exists():
            raw=json.loads((out/'result.json').read_text());contacts=[parse_contacts(out/f'gate_{i:02d}_contacts.pbtxt',30+raw['last_time_s']) for i in range(1,9)]
            raw.update(experiment_case=asdict(case),experiment_variant=a.method,method='mixed_seven_'+a.method,contacts=contacts,collision_detected=any(c['contact_pair_count'] for c in contacts),
                planning_scheduler='none_global_frozen' if a.method!='wang' else 'wang_original_local_body_exit',planning_horizon_windows=7 if a.method!='wang' else 1,
                intermediate_stop_points=False,return_to_start_required=True,tracking_error_replan_enabled=False,
                global_planning_s=source['solve_s'] if source else None,global_build_s=source['build_s'] if source else None,
                nominal_time_s=source['nominal_time_s'] if source else None,global_plan_directory=str(a.plan_dir) if source else None,closed_loop_certificate=False)
            (out/'result.json').write_text(json.dumps(raw,indent=2)+'\n')
            if raw['last_time_s']>0:
                from .audit import audit
                print('AUDIT',json.dumps(audit(out),ensure_ascii=False),flush=True)

if __name__=='__main__':main()
