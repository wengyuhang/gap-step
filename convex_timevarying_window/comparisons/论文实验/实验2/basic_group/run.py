"""Freeze the base-course 12 conditions; retain the already flown static TOGT pair."""
import argparse
import hashlib
import json
import os
import shutil
import tempfile
from pathlib import Path
from . import scenario
from .report import METHODS,report,figures
from ..run import execute
from ..static_calibration.run import sources as original_sources
from ..run_four import pair_check

MODULE='convex_timevarying_window.comparisons.论文实验.实验2.basic_group'
HERE=Path(__file__).resolve().parent;ROOT=scenario.ROOT
def read(p):return json.loads(p.read_text())
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def digest(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        while b:=f.read(4*1024**2):h.update(b)
    return h.hexdigest()
def sources():
    r=original_sources();r[str((HERE/'PROTOCOL.md').relative_to(ROOT))]=digest(HERE/'PROTOCOL.md');return r


def reuse(old):
    previous=read(old/'protocol.json');now=sources()
    changed=[r for r,h in previous['source_sha256'].items() if now.get(r)!=h]
    allowed={str((HERE.parent/'compact_calibration'/n).relative_to(ROOT)) for n in ['scenario.py','tests/test_compact.py']}
    if set(changed)-allowed:raise RuntimeError('old algorithm source changed: '+repr(changed))
    scene=scenario.manifest(scenario.Case('original','static'))
    if json.loads(json.dumps(scene))!=next(c for c in previous['cases'] if c['case']==dict(layout='original',level='static')):raise RuntimeError('retained static scene changed')
    validation=read(old/'validation.json');assert validation['raw_audit_passed']
    rows=[];evidence=[]
    for row in read(old/'summary.json'):
        if row['case']!=dict(layout='original',level='static') or row['variant'] not in ('dynamic_togt','dynamic_togt_center'):continue
        assert row['passed'] and not row['collision_detected']
        path=Path(row['directory']);raw=read(path/'result.json');plan=Path(raw['global_plan_directory'])
        expected=next(x for x in validation['checks'] if x['layout']=='original' and x['method']==row['variant'])
        assert digest(path/'result.json')==expected['result_sha256'] and digest(path/'flight.ulg')==expected['ulog_sha256']
        for name,sha in [('result.json',previous['plans']['original_'+row['variant']]['result_sha256']),('trajectory.npz',previous['plans']['original_'+row['variant']]['trajectory_sha256'])]:assert digest(plan/name)==sha
        with tempfile.TemporaryDirectory(prefix='base-course-world-') as tmp:
            track,_=scenario.build_case(scenario.Case('original','static'));world=scenario.write_world(track,scenario.ASSETS,Path(tmp));assert digest(world)==digest(path/'world.sdf')
        files={str(p.relative_to(path)):digest(p) for p in path.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
        evidence.append(dict(directory=str(path),sha256=files,plan_directory=str(plan),plan_sha256={n:digest(plan/n) for n in ['result.json','trajectory.npz','protocol.json']}))
        rows.append(dict(row,reused=True,reuse_reason='exact same static scene, world, reference, controller and preserved original evidence'))
    assert len(rows)==2
    return rows,dict(prior=str(old),changed_prior_entries=changed,allowed_changes='compact height-plane scene/test only; base static manifest and world exactly unchanged',retained=evidence)


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);p.add_argument('--reuse',type=Path,required=True);p.add_argument('--scene-probe',type=Path,required=True);a=p.parse_args()
    scenario.prepare_assets();pose=read(a.scene_probe/'gate_pose_audit.json');assert len(pose)==7 and all(r['passed'] for r in pose.values())
    assert read(a.scene_probe/'scene.json')==json.loads(json.dumps(scenario.manifest(scenario.Case('original','high'))))
    rows,inherited=reuse(a.reuse.resolve());out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False);frozen=sources()
    save(out/'reuse_audit.json',inherited);save(out/'protocol.json',dict(source_sha256=frozen,protocol=(HERE/'PROTOCOL.md').read_text(),cases=[scenario.manifest(scenario.Case('original',l)) for l in scenario.LEVELS],methods=METHODS,trials=1,total=12,scene_preflight=str(a.scene_probe.resolve()),scene_preflight_sha256={n:digest(a.scene_probe/n) for n in ['scene.json','gate_pose_audit.json','world.sdf']},no_outcome_based_reflight=True))
    for rel in frozen:
        target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,target)
    save(out/'summary.json',rows);report(out,rows)
    try:
        for i,level in enumerate(scenario.LEVELS):
            case=dict(layout='original',level=level)
            for method in (METHODS if i%2==0 else tuple(reversed(METHODS))):
                if sources()!=frozen:raise RuntimeError('frozen_source_changed')
                if any(r['case']==case and r['variant']==method for r in rows):print('RETAIN',case,method,flush=True);continue
                if shutil.disk_usage(out).free<1024**3:raise RuntimeError('disk_below_1GiB_stop_without_relaxation')
                name='original_'+level+'_'+method;planpath=out/'plans'/name
                save(out/'status.json',dict(state='planning',pid=os.getpid(),completed=len(rows),total=12,case=case,method=method,directory=str(planpath)))
                print('PLAN',name,flush=True);code=execute([MODULE+'.plan','--layout','original','--level',level,'--method',method,'--outdir',str(planpath)],out/(name+'_plan.log'),1000)
                if code:raise RuntimeError('planner_process_failed:'+name)
                pr=read(planpath/'result.json')
                if not pr['found']:
                    row=dict(case=case,variant=method,passed=False,verdict='planning_not_found',flight_time_s=None,collision_detected=False,directory=str(planpath),global_planning_s=pr['solve_s'],failure='four_starts_no_qualified_plan_not_proven_infeasible')
                else:
                    for attempt in range(2):
                        path=out/(name if attempt==0 else name+'_facility_retry')
                        save(out/'status.json',dict(state='flying',pid=os.getpid(),completed=len(rows),total=12,case=case,method=method,directory=str(path),attempt=attempt))
                        print('FLY',path.name,flush=True);code=execute([MODULE+'.flight','--layout','original','--level',level,'--method',method,'--plan-dir',str(planpath),'--outdir',str(path)],out/(path.name+'.log'),600)
                        raw=read(path/'result.json') if (path/'result.json').exists() else None
                        unflown=raw is not None and raw['last_time_s']==0 and not raw['solves'] and any(s in str(raw['failure']) for s in ('arm failed','no heartbeat','initial telemetry'))
                        if unflown and attempt==0:
                            save(path/'facility_retry_reason.json',dict(unflown=True,failure=raw['failure'],original_retained=True));continue
                        if code or not (path/'comparison_audit.json').exists():row=dict(case=case,variant=method,passed=False,verdict='evidence_failure',flight_time_s=None,collision_detected=bool(raw and raw.get('collision_detected')),directory=str(path),failure=raw['failure'] if raw else 'missing_evidence',process_returncode=code)
                        else:row=dict(read(path/'comparison_audit.json'),directory=str(path))
                        break
                rows.append(row);save(out/'summary.json',rows);report(out,rows);print('RESULT',name,row['verdict'],row.get('flight_time_s'),flush=True)
                if row['verdict']=='evidence_failure':raise RuntimeError('infrastructure_or_audit_failure:'+name)
            pair_check(out,rows,case,('ours','center'));pair_check(out,rows,case,('dynamic_togt','dynamic_togt_center'))
        if sources()!=frozen:raise RuntimeError('frozen_source_changed_at_end')
        from .validate import validate
        validate(out);figures(out,rows);report(out,rows,True)
        save(out/'status.json',dict(state='complete',pid=os.getpid(),completed=len(rows),total=12,sources_unchanged=True,raw_validation_passed=True));print('COMPLETE',len(rows),flush=True)
    except Exception as exc:
        save(out/'status.json',dict(state='stopped',pid=os.getpid(),completed=len(rows),total=12,error=repr(exc)));raise


if __name__=='__main__':main()
