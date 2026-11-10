"""Continue valid frozen tasks after diagnosed passive-sensor startup repair."""
import argparse,json,sys
from pathlib import Path
from . import run_four as engine
from .scenario import ROOT,HERE
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import digest


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);p.add_argument('--previous',type=Path,required=True);a=p.parse_args();old=a.previous.resolve()
    protocol=json.loads((old/'protocol.json').read_text());frozen=engine.sources();changed=[r for r,h in protocol['source_sha256'].items() if frozen.get(r)!=h]
    allowed=[str((HERE/'global_flight.py').relative_to(ROOT))]
    if changed!=allowed:raise RuntimeError('unexpected non-startup source changes: '+repr(changed))
    previous=json.loads((old/'summary.json').read_text());rows=[];retained=[];excluded=[]
    for r in previous:
        if r['verdict']=='evidence_failure':
            raw=json.loads((Path(r['directory'])/'result.json').read_text())
            if raw['last_time_s']!=0 or raw['solves'] or 'arm failed' not in str(raw['failure']):raise RuntimeError('cannot exclude an executed task')
            excluded.append(dict(directory=r['directory'],reason='unflown; actual magnetic sensor never published to PX4; passive native listener startup repair'))
            continue
        path=Path(r['directory']);names=['result.json']
        if r['verdict']!='planning_not_found':
            ready=json.loads((path/'race_start_audit.json').read_text());sensors=json.loads((path/'contact_manifest.json').read_text())
            if not ready['ready_at_common_race_epoch'] or sensors['sensors']!=20:raise RuntimeError('invalid earlier initialization/instrumentation')
            names+=['comparison_audit.json','flight.ulg','race_start_audit.json','contact_manifest.json','contact_publishers.json','gate_pose_audit.json',*['gate_%02d_contacts.pbtxt'%i for i in range(1,21)]]
        retained.append(dict(directory=str(path),sha256={n:digest(path/n) for n in names}))
        r.update(reused=True,reuse_reason='same frozen planner/controller/scene; valid common start and 20 physical contact sensors; additional passive sensor recorder does not change values or controls')
        rows.append(r)
    audit=dict(prior=str(old),prior_planning_algorithm_sources_unchanged=True,changed_execution_startup_only=changed,
        passive_listener_only=True,all_physical_scene_and_parameters_unchanged=True,retained=retained,excluded_unflown_facility_records=excluded,
        repaired='native Gazebo magnetometer lazy publisher first-data connection; no heading/arming bypass',methods_excluded=['wang'])
    original=engine.reuse
    engine.reuse=lambda _: (rows,audit,engine.sources())
    sys.argv=[engine.__name__,'--outdir',str(a.outdir.resolve()),'--reuse',str(old)]
    try:engine.main()
    finally:engine.reuse=original

if __name__=='__main__':main()
