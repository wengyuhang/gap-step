"""Frozen global TOGT reference, unchanged single-shooting MPC/PX4/x500."""
import argparse
import json
import subprocess
import sys
import time
from dataclasses import asdict,replace
from pathlib import Path
import numpy as np
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
from convex_timevarying_window.global_gate_minco_probe.flight import Reference,Worker,Scheduler,Progress
from convex_timevarying_window.single_gate_stop_racing.planner import Config
from convex_timevarying_window.togt_style_mpc.shooting_controller import MPC
from convex_timevarying_window.togt_style_mpc.aligned_session import Session
from convex_timevarying_window.comparisons.single_gate_swing.contacts import parse_contacts
from . import scenario

CONTAINER='convex_seven_togt_experiment';PARTITION='convex_seven_togt_experiment_partition'


def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def main():
    p=argparse.ArgumentParser();p.add_argument('--layout',required=True);p.add_argument('--method',choices=('dynamic_togt','dynamic_togt_center'),required=True);p.add_argument('--plan-dir',type=Path,required=True);p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    case=scenario.Case(a.layout,'static');track,_=scenario.build_case(case);source=json.loads((a.plan_dir/'result.json').read_text());out=a.outdir.resolve()
    assert source['found'] and source['case']==asdict(case) and source['method']==a.method
    plan=Plan(0.,0,np.array(source['durations']),np.array(source['coefficients']),np.array(source['variables']),source['solve_s']*1000,0.,'frozen_global_reference',7)
    from convex_timevarying_window.simple_two_gate_soft_only_mpc import flight,px4
    flight.Worker=Worker;flight.Scheduler=Scheduler;flight.Reference=lambda start,count:Reference(start,plan)
    flight.variant_config=lambda variant:Config(free_points=a.method=='dynamic_togt',full_task_error_recovery=False,frame_offset=.085,workspace_lower=(-100.,-120.,.65),workspace_upper=(150.,120.,20.))
    old_replace=px4.replace
    px4.replace=lambda cfg,**kw:replace(cfg,**dict(kw,horizon=7))
    flight.build_case=lambda unused:(track,None);flight.launch=scenario.launch;flight.manifest=lambda unused:scenario.manifest(case);flight.MAX_FLIGHT_TIME=180
    px4.WORKSPACE_LOWER=(-100.,-120.,.65);px4.WORKSPACE_UPPER=(150.,120.,20.)
    px4.MPC=MPC;px4.Progress=Progress;px4.CONTACT_TOPICS=scenario.contact_topics(track);px4.parse_contacts=parse_contacts

    class CheckedSession(Session):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs);self.native_recorders=[]
            for topic,name in [('/world/convex_seven_dynamic_px4_togt/model/x500_0/link/base_link/sensor/magnetometer_sensor/magnetometer','magnetometer_native.pbtxt'),('/world/convex_seven_dynamic_px4_togt/pose/info','gate_pose.pbtxt')]:
                stream=(self.out/name).open('w');reader=subprocess.Popen(['docker','exec','-e','GZ_PARTITION='+PARTITION,CONTAINER,'gz','topic','-e','-t',topic],stdout=stream,stderr=subprocess.DEVNULL)
                self.native_recorders.append((reader,stream))
            began=time.perf_counter();script="""import sys,json,subprocess
from concurrent.futures import ThreadPoolExecutor
def info(topic):
 p=subprocess.run(['gz','topic','-i','-t',topic],capture_output=True,text=True,check=True,timeout=8)
 return topic,p.stdout
with ThreadPoolExecutor(max_workers=8) as pool:
 print(json.dumps(dict(pool.map(info,json.loads(sys.argv[1])))))
"""
            data=subprocess.run(['docker','exec','-e','GZ_PARTITION='+PARTITION,CONTAINER,'python3','-c',script,json.dumps(scenario.contact_topics(track))],capture_output=True,text=True,check=True,timeout=15)
            publishers=json.loads(data.stdout)
            if len(publishers)!=8 or not all('Publishers [Address, Message Type]:' in v and 'gz.msgs.Contacts' in v for v in publishers.values()):raise RuntimeError('missing_native_contact_publisher')
            save(self.out/'publisher_check_timing.json',dict(elapsed_s=time.perf_counter()-began,channels=8,parallel=True))
            save(self.out/'contact_publishers.json',publishers)
            save(self.out/'contact_manifest.json',dict(sensor_topics=scenario.contact_topics(track),gate_sensor_groups=scenario.contact_groups(track),sensors=8,each_sensor_covers_entire_historic_sleeve_mesh=True))
        def prepare(self):
            super().prepare();at,state,age=self.read()
            ready=dict(state_time_s=float(at),position_m=state[:3].tolist(),start_error_m=float(np.linalg.norm(state[:3]-track.start)),speed_mps=float(np.linalg.norm(state[3:6])),age_s=float(age))
            ready['ready_at_common_race_epoch']=bool(at<=.1 and ready['start_error_m']<.55 and ready['speed_mps']<1.)
            save(self.out/'race_start_audit.json',ready)
            if not ready['ready_at_common_race_epoch']:raise RuntimeError('initialization_not_ready_at_race_epoch')
        def close(self):
            try:super().close()
            finally:
                for reader,stream in getattr(self,'native_recorders',[]):
                    reader.terminate()
                    try:reader.wait(timeout=2)
                    except subprocess.TimeoutExpired:reader.kill();reader.wait()
                    stream.close()
                self.native_recorders=[]

    px4.Session=CheckedSession
    sys.argv=[flight.__name__,'--method','ours' if a.method=='dynamic_togt' else 'center','--frequency','1','--amplitude','0','--phase','0','--outdir',str(out)]
    try:flight.main()
    finally:
        px4.replace=old_replace
        subprocess.run(['docker','exec','-e','GZ_PARTITION='+PARTITION,CONTAINER,'gz','service','-s','/world/convex_seven_dynamic_px4_togt/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'],capture_output=True,timeout=5)
        if (out/'result.json').exists():
            raw=json.loads((out/'result.json').read_text());contacts=scenario.parse_all_contacts(out,track,30+raw['last_time_s'])
            raw.update(experiment_case=asdict(case),experiment_variant=a.method,method='historic_togt_static_layout_'+a.method,
                contacts=contacts,collision_detected=any(c['contact_pair_count'] for c in contacts),planning_scheduler='none_global_frozen',planning_horizon_windows=7,
                intermediate_stop_points=False,return_to_start_required=True,tracking_error_replan_enabled=False,
                global_planning_s=source['solve_s'],global_build_s=source['build_s'],nominal_time_s=source['nominal_time_s'],global_plan_directory=str(a.plan_dir),closed_loop_certificate=False)
            save(out/'result.json',raw)
            if raw['last_time_s']>0:
                from .. import audit as independent
                independent.build_case=scenario.build_case;independent.Case=scenario.Case;independent.parse_all_contacts=scenario.parse_all_contacts
                result=independent.audit(out);print('AUDIT',json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
