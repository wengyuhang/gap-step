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
    for name in ('anytime_continuation_mpc','center_continuation_mpc','race_to_go_mpc','event_free_terminal_mpc','event_triggered_mpc'):
        for p in (ROOT/'convex_timevarying_window'/name).rglob('*'):
            if 'results' not in p.parts and p.suffix in ('.py','.md'):
                hashes[str(p.relative_to(ROOT))]=digest(p)
    return hashes


def execute(trial,path,log_path,rho):
    module='convex_timevarying_window.anytime_continuation_mpc.flight'
    command=[sys.executable,'-u','-m',module]
    if trial.method!='wang':command+=['--method',trial.method]
    command+=['--frequency',str(trial.frequency),'--amplitude',str(trial.amplitude),'--phase',str(trial.phase),'--outdir',str(path)]
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',GZ_SHOW_GUI='0',WANG_DEVELOPMENT_RHO=str(rho))
    start=time.time();error=None
    with log_path.open('w') as log:
        with subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True) as proc:
            try:code=proc.wait(timeout=300)
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
    parser=argparse.ArgumentParser();parser.add_argument('--outdir',type=Path,required=True);a=parser.parse_args()
    out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    conditions=[(1.,2.),(.7,2.),(1.,1.2)]
    schedule=[Trial('anytime',m,f,b,0.,0) for i,(f,b) in enumerate(conditions)
              for m in ('ours',)]
    hashes=sources()
    save(out/'protocol.json',dict(schedule=[asdict(t) for t in schedule],source_sha256=hashes,
         repeat_count=1,protocol=Path(__file__).with_name('README.md').read_text()))
    for rel in hashes:
        dest=out/'source_snapshot'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,dest)
    rows=[];attempts=[];checks=[]
    for trial in schedule:
        if sources()!=hashes:raise RuntimeError('Source changed during batch')
        if shutil.disk_usage(out).free<3*1024**3:raise RuntimeError('Insufficient free disk')
        for attempt in (1,2):
            path=out/(trial.name+f'_attempt{attempt}')
            save(out/'status.json',dict(state='running',done=len(rows),total=3,name=trial.name,attempt=attempt))
            print('START',len(rows)+1,trial.name,attempt,flush=True)
            row=execute(trial,path,out/(path.name+'.log'),100);row['attempt']=attempt
            attempts.append(row);save(out/'attempts.json',attempts)
            if not row['infrastructure_failure']:break
        if row.get('audit_error'):raise RuntimeError(row['audit_error'])
        raw=json.loads((path/'result.json').read_text())
        paired=[r for r in rows if r['pair']==trial.pair]
        match={}
        historical=ROOT/'convex_timevarying_window/comparisons/comprehensive_wang/results/phase_zero_v1_20261007'/('ablation_center_'+trial.pair+'_attempt1')
        old_raw=json.loads((historical/'result.json').read_text())
        match['historical_world']=digest(path/'world.sdf')==digest(historical/'world.sdf')
        match['historical_mpc']=raw['mpc_config']==old_raw['mpc_config']
        match['historical_px4']=raw['parameters']==old_raw['parameters']
        for previous in paired:
            for key in ('world_sha256','scene_manifest','mpc_config','px4_parameters'):
                match[key]=row[key]==previous[key]
        accepted=[s for s in raw['solves'] if s['accepted']]
        selections=[s['race_selection'] for s in accepted]
        match['center_candidate_envelope']=all(
            not s['center_valid'] or s['nominal_finish_s']<=s['center_finish_s']+1e-6
            for s in [dict(x,nominal_finish_s=y['nominal_finish_s']) for x,y in zip(selections,accepted)])
        finish=[s['nominal_finish_s'] for s in accepted if s['nominal_finish_s'] is not None]
        match['no_predicted_finish_regression']=all(b<=a+1e-8 for a,b in zip(finish,finish[1:]))
        for s in accepted:
            gates=len(s['durations'])//2-1;qi=4*(gates+1)+6*gates
            suffix=s['variables'][qi+2*raw['config']['free_prefix_gates']:qi+2*gates]
            match['center_suffix_'+str(s['time'])]=all(abs(q)<1e-9 for q in suffix)
            if s.get('is_bootstrap'):
                match['bootstrap_center_'+str(s['time'])]=all(abs(q)<1e-9 for q in s['variables'][qi:qi+2*gates])
        row.update(accepted=len(accepted),planning_calls=len(raw['solves']),
            free_selected=sum(s['selected']=='free' for s in selections),
            valid_center_comparisons=sum(s['center_valid'] for s in selections),
            planner_max_ms=max((s['cycle_ms'] for s in raw['solves']),default=0.),
            mpc_max_ms=max((s['cycle_ms'] for s in raw['mpc_cycles']),default=0.))
        checks.append(dict(name=trial.name,checks=match))
        if not all(match.values()):save(out/'checks.json',checks);raise RuntimeError('Mechanism audit failed')
        rows.append(row);save(out/'summary.json',rows);save(out/'checks.json',checks)
        print('RESULT',json.dumps({k:row.get(k) for k in ('name','race_passed','race_time_s','body_exit_count','failure','planner_max_ms')},ensure_ascii=False),flush=True)
    verify=dict(source_count=len(hashes),changed=[k for k,v in hashes.items() if digest(ROOT/k)!=v],
        snapshot_changed=[k for k,v in hashes.items() if digest(out/'source_snapshot'/k)!=v])
    save(out/'verification.json',verify)
    assert not verify['changed'] and not verify['snapshot_changed']
    old={(1.,2.):24.428,(.7,2.):27.640,(1.,1.2):24.432}
    prior={(1.,2.):29.388,(.7,2.):21.796,(1.,1.2):24.296}
    report=['# Anytime center continuation: three frozen PX4 flights','',
        'Each condition once. Historical center and prior continuation are separate algorithms, not same-code ablations.',
        'Unsafe/incomplete flights have no valid race time. Sampled geometry and contacts are not continuous certificates.','',
        '|Hz / m|Historical center / s|Prior free-prefix / s|Anytime free-prefix / s|',
        '|---|---:|---:|---:|']
    for r in rows:
        k=(r['trial']['frequency'],r['trial']['amplitude'])
        report.append(f"|{k[0]} / {k[1]}|{old[k]}|{prior[k]}|{r.get('race_time_s')}|")
    report+=['','## All outcomes','']
    for r in rows:
        report.append(f"- {r['name']}: passed={r['race_passed']}, exits={r.get('body_exit_count')}, failure={r.get('failure')}; planner/MPC max={r['planner_max_ms']:.3f}/{r['mpc_max_ms']:.3f}ms.")
    (out/'REPORT.md').write_text('\n'.join(report)+'\n')
    save(out/'status.json',dict(state='complete',done=3,total=3))
    print('COMPLETE',out,flush=True)


if __name__=='__main__':main()
