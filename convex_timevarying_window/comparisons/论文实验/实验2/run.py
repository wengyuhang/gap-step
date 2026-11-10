"""Frozen 9-condition/3-method matrix; global free/center planned before flight."""
import argparse,json,os,shutil,signal,subprocess,sys
from pathlib import Path
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import sources as base_sources,save,digest
from .scenario import LAYOUTS,LEVELS,Case,manifest,HERE,ROOT
from .report import report,figures
MODULE='convex_timevarying_window.comparisons.论文实验.实验2'

def sources():
    result=base_sources(ROOT)
    for name in ('assets/MixedGateMotion.cc','assets/CMakeLists.txt','assets/build/libMixedGateMotion.so','assets/circle_frame.stl'):
        p=HERE/name;result[str(p.relative_to(ROOT))]=digest(p)
    return result

def execute(args,log,timeout):
    with log.open('w') as f:
        with subprocess.Popen([sys.executable,'-u','-m',*args],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,
            env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',GZ_SHOW_GUI='0')) as p:
            try:return p.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
                raise RuntimeError('subprocess_timeout:'+str(log))

def pair_check(out,rows,case):
    pair=[r for r in rows if r['case']==case and r['variant'] in ('ours','center')]
    if len(pair)!=2 or any(r['verdict'] in ('planning_not_found','evidence_failure') for r in pair):return
    raw=[json.loads((Path(r['directory'])/'result.json').read_text()) for r in pair];cfg=[dict(r['config']) for r in raw]
    for c in cfg:c.pop('free_points')
    matching=cfg[0]==cfg[1] and raw[0]['parameters']==raw[1]['parameters'] and raw[0]['mpc_config']==raw[1]['mpc_config'] and digest(Path(pair[0]['directory'])/'world.sdf')==digest(Path(pair[1]['directory'])/'world.sdf')
    save(out/f"pair_{case['layout']}_{case['level']}.json",dict(equal_except_free_points=matching,directories=[r['directory'] for r in pair]))
    if not matching:raise RuntimeError('paired_configuration_mismatch')

def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args();out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    frozen=sources();rows=[]
    save(out/'protocol.json',dict(source_sha256=frozen,protocol=(HERE/'PROTOCOL.md').read_text(),cases=[manifest(Case(l,d)) for l in LAYOUTS for d in LEVELS],methods=['ours','center','wang'],trials_per_condition=1,total=27))
    for rel in frozen:
        target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,target)
    save(out/'summary.json',rows);report(out,rows);figures(out,rows)
    try:
        for i,(layout,level) in enumerate((l,d) for l in LAYOUTS for d in LEVELS):
            case=dict(layout=layout,level=level)
            order=('ours','center','wang') if i%2==0 else ('wang','center','ours')
            for method in order:
                if sources()!=frozen:raise RuntimeError('frozen_source_changed')
                name=f'{layout}_{level}_{method}';planpath=out/'plans'/name;plan_result=None
                if method!='wang':
                    save(out/'status.json',dict(state='planning',pid=os.getpid(),completed=len(rows),total=27,case=case,method=method,directory=str(planpath)))
                    print('PLAN',name,flush=True)
                    code=execute([MODULE+'.plan','--layout',layout,'--level',level,'--method',method,'--outdir',str(planpath)],out/(name+'_plan.log'),1000)
                    if code:raise RuntimeError('planning_process_failed:'+name)
                    plan_result=json.loads((planpath/'result.json').read_text())
                    if not plan_result['found']:
                        row=dict(case=case,variant=method,passed=False,verdict='planning_not_found',flight_time_s=None,collision_detected=False,
                            directory=str(planpath),global_planning_s=plan_result['solve_s'],failure='current_four_seeds_no_qualified_plan_not_proven_infeasible')
                        rows.append(row);save(out/'summary.json',rows);report(out,rows);print('RESULT',name,row['verdict'],flush=True);continue
                for attempt in range(2):
                    path=out/(name if attempt==0 else name+'_facility_retry')
                    save(out/'status.json',dict(state='flying',pid=os.getpid(),completed=len(rows),total=27,case=case,method=method,directory=str(path),attempt=attempt))
                    args=[MODULE+'.flight','--layout',layout,'--level',level,'--method',method,'--outdir',str(path)]
                    if method!='wang':args+=['--plan-dir',str(planpath)]
                    print('FLY',path.name,flush=True);code=execute(args,out/(path.name+'.log'),600)
                    raw=json.loads((path/'result.json').read_text()) if (path/'result.json').exists() else None
                    unflown=raw is not None and raw['last_time_s']==0 and not raw['solves'] and any(x in str(raw['failure']) for x in ('arm failed','no heartbeat','initial telemetry'))
                    if unflown and attempt==0:
                        save(path/'facility_retry_reason.json',dict(unflown=True,failure=raw['failure'],original_retained=True));print('FACILITY_RETRY',path.name,flush=True);continue
                    if code or not (path/'comparison_audit.json').exists():
                        row=dict(case=case,variant=method,passed=False,verdict='evidence_failure',flight_time_s=None,collision_detected=bool(raw and raw.get('collision_detected')),failure=raw['failure'] if raw else 'missing_evidence',directory=str(path),process_returncode=code)
                    else:row=json.loads((path/'comparison_audit.json').read_text());row['directory']=str(path)
                    break
                rows.append(row);save(out/'summary.json',rows);report(out,rows);print('RESULT',name,row['verdict'],row.get('flight_time_s'),flush=True)
                if row['verdict']=='evidence_failure':raise RuntimeError('infrastructure_or_audit_failure:'+name)
            pair_check(out,rows,case)
        if sources()!=frozen:raise RuntimeError('source_changed_at_end')
        report(out,rows,True);figures(out,rows);save(out/'status.json',dict(state='complete',pid=os.getpid(),completed=len(rows),total=27,sources_unchanged=True));print('COMPLETE',len(rows),flush=True)
    except Exception as exc:
        save(out/'status.json',dict(state='stopped',pid=os.getpid(),completed=len(rows),total=27,error=repr(exc)));raise

if __name__=='__main__':main()
