"""Frozen three-condition center-continuation comparison; every flight retained."""
import argparse
from dataclasses import asdict
import json,os,shutil,signal,subprocess,sys,time
from pathlib import Path
from convex_timevarying_window.comparisons.comprehensive_wang.run import sources as common_sources
from convex_timevarying_window.comparisons.comprehensive_wang.registry import Trial
from convex_timevarying_window.comparisons.comprehensive_wang.audit import audit
from convex_timevarying_window.comparisons.checked_soft_racing.adjudicate import assess
from 复现.polynomial_dynamic_racing_2023.all_versions_current_track.run import infrastructure_arm_failure
from 复现.polynomial_dynamic_racing_2023.px4_original_track_legacy.run import save,digest,ROOT


def sources():
    hashes=common_sources()
    for name in ('third_gate_time_mpc','event_free_terminal_mpc','event_triggered_mpc'):
        for p in (ROOT/'convex_timevarying_window'/name).rglob('*'):
            if 'results' not in p.parts and p.suffix in ('.py','.md'):
                hashes[str(p.relative_to(ROOT))]=digest(p)
    return hashes


def execute(trial,path,log_path,rho):
    module='convex_timevarying_window.third_gate_time_mpc.flight'
    command=[sys.executable,'-u','-m',module]
    if trial.method!='wang':command+=['--method',trial.method]
    command+=['--frequency',str(trial.frequency),'--amplitude',str(trial.amplitude),'--phase',str(trial.phase),'--outdir',str(path)]
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',GZ_SHOW_GUI='0',WANG_DEVELOPMENT_RHO=str(rho))
    start=time.time();error=None
    with log_path.open('w') as log:
        with subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True) as proc:
            try:code=proc.wait(timeout=900)
            except subprocess.TimeoutExpired:
                error='process_timeout';os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                code=proc.returncode
    row=dict(trial=asdict(trial),name=trial.name,pair=trial.pair,directory=str(path),wall_s=time.time()-start,
        returncode=code,process_error=error,race_passed=False,main_passed=False,infrastructure_failure=False)
    with log_path.with_suffix('.docker.log').open('w') as log:
        subprocess.run(['docker','logs','convex_seven_togt_experiment'],stdout=log,stderr=subprocess.STDOUT,timeout=15)
    if not (path/'result.json').exists():return dict(row,failure='missing_result',audit_error='missing_result')
    raw=json.loads((path/'result.json').read_text());row['failure']=raw.get('failure')
    row['infrastructure_failure']=infrastructure_arm_failure(raw)
    try:
        row['preparation_assessment']=assess(path)
        row['infrastructure_failure'] |= row['preparation_assessment']['setup_failure']
    except Exception as exc:row['preparation_assessment_error']=repr(exc)
    row['world_sha256']=digest(path/'world.sdf');row['mpc_config']=raw['mpc_config'];row['px4_parameters']=raw['parameters']
    row['scene_manifest']=json.loads((path/'scene.json').read_text())
    if not row['infrastructure_failure']:
        try:row.update(audit(path))
        except Exception as exc:row.update(audit_error=repr(exc))
    save(path/'comparison_audit.json',row)
    return row


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    hashes=sources();schedule=[Trial('third_gate',m,1.,2.,0.,0) for m in ('ours','center','no_value')]
    save(out/'protocol.json',dict(schedule=[asdict(t) for t in schedule],source_sha256=hashes,
        repeat_count=1,protocol=Path(__file__).with_name('README.md').read_text()))
    for rel in hashes:
        dst=out/'source_snapshot'/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,dst)
    rows=[];attempts=[]
    for trial in schedule:
        assert sources()==hashes,'Sources changed during frozen experiment'
        for attempt in (1,2):
            path=out/(trial.name+f'_attempt{attempt}')
            save(out/'status.json',dict(state='running',variant=trial.method,attempt=attempt))
            print('START',trial.method,attempt,flush=True)
            row=execute(trial,path,out/(path.name+'.log'),100)
            attempts.append(row);save(out/'attempts.json',attempts)
            if not row['infrastructure_failure']:break
        if (path/'result.json').exists():
            raw=json.loads((path/'result.json').read_text());solves=raw['solves']
            row.update(planning_calls=len(solves),accepted=sum(s['accepted'] for s in solves),
                converged=sum(s.get('optimizer_stats',{}).get('converged',False) for s in solves),
                planner_max_ms=max((s['cycle_ms'] for s in solves),default=0.),
                mpc_max_ms=max((s['cycle_ms'] for s in raw['mpc_cycles']),default=0.))
            from collections import Counter
            row['rejections']=dict(Counter(s['rejection_reason'] for s in solves if not s['accepted']))
            assert all(s.get('optimizer_stats',{}).get('converged',False) for s in solves if s['accepted'])
        rows.append(row);save(out/'summary.json',rows)
        print('RESULT',json.dumps({k:row.get(k) for k in ('name','race_passed','race_time_s','failure','accepted','converged','planner_max_ms')},ensure_ascii=False),flush=True)
    changes=[k for k,v in hashes.items() if digest(ROOT/k)!=v]
    snapshot_changes=[k for k,v in hashes.items() if digest(out/'source_snapshot'/k)!=v]
    pairs={}
    for row in rows[1:]:
        pairs[row['name']]={k:row.get(k)==rows[0].get(k) for k in ('world_sha256','scene_manifest','mpc_config','px4_parameters')}
    save(out/'verification.json',dict(source_changes=changes,snapshot_changes=snapshot_changes,matched=pairs))
    assert not changes and not snapshot_changes and all(all(r.values()) for r in pairs.values())
    lines=['# Third-window time estimate: hardest scene only','',
        '1 Hz / 2 m / initial phase. One trial each. Normal convergence is required, not best-iterate admission.',
        'Seven-gate success and strict return are separate. Sampled safety is not a certificate.','',
        '|Variant|Race passed|Race time/s|Body exits|Accepted/converged/total solves|Max planner/ms|Failure|',
        '|---|---|---|---|---|---|---|']
    for r in rows:
        lines.append(f"|{r['trial']['method']}|{r.get('race_passed')}|{r.get('race_time_s')}|{r.get('body_exit_count')}|{r.get('accepted')}/{r.get('converged')}/{r.get('planning_calls')}|{r.get('planner_max_ms')}|{r.get('failure')}|")
    lines+=['','Historical references, NOT matched ablations: old forced-center 24.428 s; full-course anytime 22.760 s.',
        'An online failure is not evidence that the moving course or the reach-time idea is impossible.']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    save(out/'status.json',dict(state='complete',trials=len(rows)))
    print('COMPLETE',out,flush=True)


if __name__=='__main__':main()
