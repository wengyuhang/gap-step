"""Four all-global variants; preserve previous current-method evidence, no Wang."""
import argparse,json,os,shutil
from pathlib import Path
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import save,digest
from .run import sources as original_sources,execute
from .scenario import LAYOUTS,LEVELS,Case,manifest,HERE,ROOT
from .report_four import report,METHODS
MODULE='convex_timevarying_window.comparisons.论文实验.实验2'

def sources():
    result=original_sources()
    for rel in ('convex_timevarying_window/togt/native/build/libtogt_analytic.so','convex_timevarying_window/togt/native/togt_analytic.cpp',
                'convex_dynamic_seven_window_gazebo/x500_togt/native/build/libtogt_x500.so','convex_dynamic_seven_window_gazebo/x500_togt/native/togt_x500.cpp'):
        result[rel]=digest(ROOT/rel)
    return result

def reuse(old):
    old=old.resolve();protocol=json.loads((old/'protocol.json').read_text());now=sources()
    changed=[rel for rel,sha in protocol['source_sha256'].items() if now.get(rel)!=sha]
    allowed={str((HERE/name).relative_to(ROOT)) for name in ('scenario.py','audit.py','tests/test_mixed.py')}
    if set(changed)-allowed:raise RuntimeError('prior algorithm source changed: '+repr(changed))
    # Geometry, motion, task and all planner source remain exact. Only the
    # contact instrumentation/audit is repaired; no old flight is reused.
    import ast
    rel=str((HERE/'scenario.py').relative_to(ROOT));previous=(old/'source_snapshot'/rel).read_text();current=(HERE/'scenario.py').read_text()
    def definitions(text):
        return {n.name:ast.get_source_segment(text,n) for n in ast.parse(text).body if isinstance(n,(ast.FunctionDef,ast.ClassDef))}
    before,after=definitions(previous),definitions(current)
    checked=('Case','Motion','Aperture','Window','pose','centers','local_coordinates','make_rotation','build_case','manifest')
    if any(before[n]!=after[n] for n in checked):raise RuntimeError('planning scene functions changed')
    for scene in protocol['cases']:
        if manifest(Case(**scene['case']))!=scene:raise RuntimeError('planning scene manifest changed')
    import tempfile,xml.etree.ElementTree as ET
    from . import scenario
    namespace=dict(vars(scenario));exec(before['write_world'],namespace)
    track,_=scenario.build_case(Case('slalom','high'))
    with tempfile.TemporaryDirectory(prefix='contact_sensor_parity_') as tmp:
        folder=Path(tmp);(folder/'old').mkdir();(folder/'new').mkdir()
        paths=[namespace['write_world'](track,scenario.ASSETS,folder/'old'),scenario.write_world(track,scenario.ASSETS,folder/'new')]
        trees=[ET.parse(p) for p in paths]
        for tree in trees:
            for link in tree.findall('world/model/link'):
                for sensor in list(link.findall('sensor')):link.remove(sensor)
        if ET.tostring(trees[0].getroot())!=ET.tostring(trees[1].getroot()):raise RuntimeError('physical scene changed beyond contact sensors')
    previous_rows=json.loads((old/'summary.json').read_text())
    rows=[r for r in previous_rows if r['variant'] in ('ours','center') and r['verdict']=='planning_not_found']
    # The in-flight optimization was allowed to finish after the old dispatcher
    # stopped; its fully written result is reused, never repeated for a new outcome.
    for d in sorted((old/'plans').iterdir()):
        f=d/'result.json'
        if not f.exists():continue
        raw=json.loads(f.read_text());key=(raw['case']['layout'],raw['case']['level'],raw['method'])
        if any((r['case']['layout'],r['case']['level'],r['variant'])==key for r in rows):continue
        if raw['method'] not in ('ours','center'):continue
        if raw['found']:continue  # A found but unexecuted plan is used below.
        rows.append(dict(case=raw['case'],variant=raw['method'],passed=False,verdict='planning_not_found',flight_time_s=None,
            collision_detected=False,directory=str(d),global_planning_s=raw['solve_s'],failure='current_four_seeds_no_qualified_plan_not_proven_infeasible'))
    evidence=[]
    for r in rows:
        d=Path(r['directory']);names=['result.json'] if r['verdict']=='planning_not_found' else ['result.json','comparison_audit.json','flight.ulg','contact_publishers.json','gate_pose_audit.json',*['gate_%02d_contacts.pbtxt'%i for i in range(1,9)]]
        sha={name:digest(d/name) for name in names};evidence.append(dict(directory=str(d),sha256=sha))
        r.update(reused=True,reuse_reason='all previously frozen algorithm/scene source entries unchanged; previously executed task or completed request retained')
    return rows,dict(prior=str(old),prior_planning_algorithm_sources_unchanged=True,changed_prior_entries=changed,
        added_files=sorted(set(now)-set(protocol['source_sha256'])),retained=evidence,excluded_methods=['wang'],
        unchanged_planning_scene_definitions=list(checked),all_scene_manifests_equal=True,physical_world_xml_equal_after_removing_contact_sensors=True,instrumentation_source_changes=changed,
        old_physical_records_not_reused=[dict(directory=r['directory'],reason='rectangle contact configuration covered only one edge; all physical tasks rerun under corrected complete instrumentation') for r in previous_rows if r['verdict']!='planning_not_found']),now

def pair_check(out,rows,case,variants):
    pair=[r for r in rows if r['case']==case and r['variant'] in variants]
    if len(pair)!=2 or any(r['verdict'] in ('planning_not_found','evidence_failure') for r in pair):return
    raw=[json.loads((Path(r['directory'])/'result.json').read_text()) for r in pair];cfg=[dict(r['config']) for r in raw]
    for c in cfg:c.pop('free_points')
    same=cfg[0]==cfg[1] and raw[0]['mpc_config']==raw[1]['mpc_config'] and raw[0]['parameters']==raw[1]['parameters'] and digest(Path(pair[0]['directory'])/'world.sdf')==digest(Path(pair[1]['directory'])/'world.sdf')
    save(out/f"pair_{case['layout']}_{case['level']}_{variants[0]}.json",dict(equivalent_except_free_points=same,directories=[r['directory'] for r in pair]))
    if not same:raise RuntimeError('pair_configuration_mismatch')

def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',required=True,type=Path);p.add_argument('--reuse',required=True,type=Path);a=p.parse_args()
    rows,audit,frozen=reuse(a.reuse);out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    save(out/'reuse_audit.json',audit);save(out/'summary.json',rows)
    save(out/'protocol.json',dict(source_sha256=frozen,protocol=(HERE/'FOUR_METHOD_PROTOCOL.md').read_text(),cases=[manifest(Case(l,d)) for l in LAYOUTS for d in LEVELS],methods=METHODS,trials=1,total=36))
    for rel in frozen:
        target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,target)
    report(out,rows)
    try:
        for i,(layout,level) in enumerate((l,d) for l in LAYOUTS for d in LEVELS):
            case=dict(layout=layout,level=level)
            for method in (METHODS if i%2==0 else tuple(reversed(METHODS))):
                if sources()!=frozen:raise RuntimeError('frozen_source_changed')
                if any(r['case']==case and r['variant']==method for r in rows):print('RETAIN',case,method,flush=True);continue
                name=f'{layout}_{level}_{method}';planpath=out/'plans'/name
                priorplan=a.reuse/'plans'/name
                if method in ('ours','center') and (priorplan/'result.json').exists():
                    old=json.loads((priorplan/'result.json').read_text());assert old['found'];planpath=priorplan.resolve();print('RETAIN_PLAN',name,flush=True)
                else:
                    save(out/'status.json',dict(state='planning',pid=os.getpid(),completed=len(rows),total=36,case=case,method=method,directory=str(planpath)))
                    print('PLAN',name,flush=True)
                    mod=MODULE+'.plan' if method in ('ours','center') else MODULE+'.togt_dynamic'
                    code=execute([mod,'--layout',layout,'--level',level,'--method',method,'--outdir',str(planpath)],out/(name+'_plan.log'),1000)
                    if code:raise RuntimeError('planner_process_failed:'+name)
                pr=json.loads((planpath/'result.json').read_text())
                if not pr['found']:
                    row=dict(case=case,variant=method,passed=False,verdict='planning_not_found',flight_time_s=None,collision_detected=False,directory=str(planpath),global_planning_s=pr['solve_s'],failure='four_starts_no_admitted_result_not_proven_infeasible')
                    rows.append(row);save(out/'summary.json',rows);report(out,rows);print('RESULT',name,row['verdict'],flush=True);continue
                for attempt in range(2):
                    path=out/(name if attempt==0 else name+'_facility_retry')
                    save(out/'status.json',dict(state='flying',pid=os.getpid(),completed=len(rows),total=36,case=case,method=method,directory=str(path),attempt=attempt))
                    print('FLY',path.name,flush=True)
                    code=execute([MODULE+'.global_flight','--layout',layout,'--level',level,'--method',method,'--plan-dir',str(planpath),'--outdir',str(path)],out/(path.name+'.log'),600)
                    raw=json.loads((path/'result.json').read_text()) if (path/'result.json').exists() else None
                    unflown=raw is not None and raw['last_time_s']==0 and not raw['solves'] and any(s in str(raw['failure']) for s in ('arm failed','no heartbeat','initial telemetry'))
                    if unflown and attempt==0:
                        save(path/'facility_retry_reason.json',dict(unflown=True,failure=raw['failure'],original_retained=True));print('FACILITY_RETRY',name,flush=True);continue
                    if code or not (path/'comparison_audit.json').exists():
                        row=dict(case=case,variant=method,passed=False,verdict='evidence_failure',flight_time_s=None,collision_detected=bool(raw and raw.get('collision_detected')),directory=str(path),failure=raw['failure'] if raw else 'missing_evidence',process_returncode=code)
                    else:row=json.loads((path/'comparison_audit.json').read_text());row['directory']=str(path)
                    break
                rows.append(row);save(out/'summary.json',rows);report(out,rows);print('RESULT',name,row['verdict'],row.get('flight_time_s'),flush=True)
                if row['verdict']=='evidence_failure':raise RuntimeError('infrastructure_or_audit_failure:'+name)
            pair_check(out,rows,case,('ours','center'));pair_check(out,rows,case,('dynamic_togt','dynamic_togt_center'))
        if sources()!=frozen:raise RuntimeError('frozen_source_changed_end')
        for r in audit['retained']:
            if {k:digest(Path(r['directory'])/k) for k in r['sha256']}!=r['sha256']:raise RuntimeError('retained_evidence_changed')
        report(out,rows,True);save(out/'status.json',dict(state='complete',pid=os.getpid(),completed=len(rows),total=36,sources_unchanged=True,reused_evidence_unchanged=True));print('COMPLETE',len(rows),flush=True)
    except Exception as exc:
        save(out/'status.json',dict(state='stopped',pid=os.getpid(),completed=len(rows),total=36,error=repr(exc)));raise

if __name__=='__main__':main()
