"""Frozen soft-only simulation sweep; increments of exactly 0.5 Hz / 0.5 m."""
import argparse
from dataclasses import asdict
import json, os, shutil, signal, subprocess, sys, time
from pathlib import Path
from convex_timevarying_window.simple_two_gate_mpc.run import sources as old_sources
from convex_timevarying_window.comparisons.comprehensive_wang.registry import Trial
from convex_timevarying_window.comparisons.comprehensive_wang.audit import audit
from convex_timevarying_window.comparisons.checked_soft_racing.adjudicate import assess
from 复现.polynomial_dynamic_racing_2023.all_versions_current_track.run import infrastructure_arm_failure
from 复现.polynomial_dynamic_racing_2023.px4_original_track_legacy.run import save, digest, ROOT
from convex_timevarying_window.comparisons.simple_difficulty_sweep.simulation_run import evaluate

ROUTES = {
    'joint': [(1.5,2.5),(2.,3.),(2.5,3.5),(3.,4.)],
    'frequency': [(1.5,2.),(2.,2.),(2.5,2.),(3.,2.)],
    'amplitude': [(1.,2.5),(1.,3.),(1.,3.5),(1.,4.)],
}
METHODS = ('center','ours')


def recover_contact_failure(row):
    """A damaged post-crash truth tail cannot erase a recorded physical collision."""
    path=Path(row['directory'])/'result.json'
    if not path.exists():return row
    raw=json.loads(path.read_text())
    row.update({k:raw.get(k) for k in ('completed','failure','flight_time_s','last_time_s','collision_detected')})
    contact_times=[c['first_contact_sim_time_s']-30 for c in raw.get('contacts',[])
                   if c.get('first_contact_sim_time_s') is not None
                   and 30<=c['first_contact_sim_time_s']<=30+raw['last_time_s']]
    if row.get('audit_error') and contact_times and raw.get('collision_detected'):
        row.update(contact_evidence_sufficient=True,full_truth_audit_available=False,
                   first_contact_race_time_s=min(contact_times),
                   body_exit_count=len({e['gate'] for e in raw.get('body_exits',[])}),
                   body_exit_count_source='online measured events; full truth tail unavailable',main_passed=False)
    return row


def finish_stalled_logger(proc,path):
    if not (path/'sender_failure.npz').exists():return
    listing=subprocess.run(['ps','--ppid',str(proc.pid),'-o','pid=,etimes=,args='],capture_output=True,text=True)
    expected='docker exec convex_seven_togt_experiment /opt/px4-gazebo/bin/px4-logger stop'
    for line in listing.stdout.splitlines():
        parts=line.split(maxsplit=2)
        if len(parts)==3 and parts[2]==expected and int(parts[1])>15:
            try:os.kill(int(parts[0]),signal.SIGTERM)
            except ProcessLookupError:continue
            save(path/'logger_cleanup_intervention.json',dict(pid=int(parts[0]),command=expected,
                 reason='Post-failure logger stop stalled; preserve incomplete ULog and physical contacts'))


def sources():
    hashes = old_sources()
    for directory in (Path(__file__).parent, ROOT/'convex_timevarying_window/comparisons/simple_difficulty_sweep'):
        for p in directory.rglob('*'):
            if 'results' not in p.parts and p.suffix in ('.py','.md'):
                hashes[str(p.relative_to(ROOT))] = digest(p)
    return hashes


def execute(trial,path,log_path,rho,*,module='convex_timevarying_window.simple_two_gate_soft_only_mpc.flight',audit_fn=None):
    command=[sys.executable,'-u','-m',module]
    if trial.method!='wang':command+=['--method',trial.method]
    command+=['--frequency',str(trial.frequency),'--amplitude',str(trial.amplitude),'--phase',str(trial.phase),'--outdir',str(path)]
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',GZ_SHOW_GUI='0',WANG_DEVELOPMENT_RHO=str(rho))
    start=time.time();error=None
    with log_path.open('w') as log:
        with subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True) as proc:
            try:
                while True:
                    try:code=proc.wait(timeout=5);break
                    except subprocess.TimeoutExpired:
                        finish_stalled_logger(proc,path)
                        if time.time()-start>=900:raise
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
        try:row.update((audit if audit_fn is None else audit_fn)(path))
        except Exception as exc:row.update(audit_error=repr(exc))
    row=recover_contact_failure(row)
    save(path/'comparison_audit.json',row)
    return row


def report(out, rows, boundaries):
    lines = ['# 取消名义轨迹准入检查：0.5步长递增实验', '',
             '新版本全部重飞，原七窗/初相/同一MPC/PX4，完整时间含返航。每条件每方一次，不推断成功率。',
             '仅物理接触判断碰撞；名义残差、耗时、球包络不否决成绩。未完成不是完成时间。', '',
             '|频率 Hz|幅度 m|自由完整时间 s|自由碰撞|自由结果|中心完整时间 s|中心碰撞|中心结果|',
             '|---:|---:|---:|---|---|---:|---|---|']
    pairs = {}
    for row in rows:
        t=row['trial']; pairs.setdefault((t['frequency'],t['amplitude']),{})[t['method']]=row
    for (f,a), pair in pairs.items():
        vals=[f,a]
        for method in ('ours','center'):
            r=pair.get(method,{})
            vals += [r.get('simulation_race_time_s'),r.get('collision_detected'),r.get('simulation_outcome','尚未运行')]
        lines.append('| '+' | '.join(map(str,vals))+' |')
    lines += ['','## 已观察到的失败档位','','```json',json.dumps(boundaries,indent=2,ensure_ascii=False),'```',
              '', '档位顺序不证明物理难度单调；高档成功和失败样本全部保留。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--resume',action='store_true');a=p.parse_args()
    out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=a.resume)
    hashes=sources()
    if not a.resume:
        save(out/'protocol.json',dict(routes=ROUTES,baseline=(1.,2.),source_sha256=hashes,
         protocol=Path(__file__).with_name('README.md').read_text(),repeat_count=1,phase=0.,
         primary_metrics=['simulation_race_time_s','collision_detected'],reused_old_flights=False))
        for rel in hashes:
            dst=out/'source_snapshot'/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,dst)
    original_hashes=json.loads((out/'protocol.json').read_text())['source_sha256']
    changed=[rel for rel,h in original_hashes.items() if digest(ROOT/rel)!=h]
    assert set(changed)<={'convex_timevarying_window/simple_two_gate_soft_only_mpc/run.py'}, changed
    if a.resume:
        save(out/'reporting_revision.json',dict(changed_sources=changed,new_source_sha256=hashes,
             reason='Resume unchanged flights; physical contact remains failure despite missing post-crash truth tail; bounded logger cleanup'))
        revision=out/'reporting_revision';revision.mkdir(exist_ok=True)
        shutil.copyfile(Path(__file__),revision/'run.py')
    rows=[];attempts=[];cache={};boundaries={};base_raw=None
    if a.resume:
        attempts=json.loads((out/'attempts.json').read_text())
        for prior in attempts:
            if prior.get('infrastructure_failure'):continue
            row=evaluate(recover_contact_failure(dict(prior)))
            if row.get('audit_error') and not row.get('contact_evidence_sufficient'):
                raise RuntimeError(f'Unresolved evidence: {row["directory"]}')
            key=(row['trial']['frequency'],row['trial']['amplitude'],row['trial']['method'])
            assert key not in cache
            rows.append(row);cache[key]=row
        save(out/'summary.json',rows)
        if rows:base_raw=json.loads((Path(rows[0]['directory'])/'result.json').read_text())

    def pair(f,amplitude,stage):
        nonlocal base_raw
        for method in METHODS:
            key=(f,amplitude,method)
            if key in cache:continue
            trial=Trial(stage,method,f,amplitude,0.,0)
            for attempt in (1,2):
                assert sources()==hashes,'Frozen sources changed'
                path=out/(trial.name+f'_attempt{attempt}')
                save(out/'status.json',dict(state='running',frequency=f,amplitude=amplitude,method=method,stage=stage,
                     attempt=attempt,completed_trials=len(rows)))
                print('START',trial.name,attempt,flush=True)
                row=evaluate(execute(trial,path,out/(path.name+'.log'),100))
                attempts.append(row);save(out/'attempts.json',attempts)
                if not row.get('infrastructure_failure'):break
            if row.get('infrastructure_failure') or (row.get('audit_error') and not row.get('contact_evidence_sufficient')):
                raise RuntimeError(f'Infrastructure or evidence failure: {path}')
            raw=json.loads((path/'result.json').read_text())
            if base_raw is None:base_raw=raw
            for field in ('mpc_config','parameters'):assert raw[field]==base_raw[field]
            left,right=dict(raw['config']),dict(base_raw['config'])
            left.pop('free_points');right.pop('free_points');assert left==right
            forbidden={'optimizer_residual','deadline','late_result','optimizer_not_converged','head_timeline_changed'}
            assert not any(s.get('rejection_reason') in forbidden for s in raw['solves'])
            assert all(s.get('event_valid') is None for s in raw['solves'] if 'candidate_first_gate' in s)
            assert all(e['reason'] in ('initial','body_exit') for e in raw['planning_events'] if e['action']=='trigger')
            assert all('coefficients' in s for s in raw['solves'] if 'candidate_first_gate' in s)
            row['soft_only_execution_verified']=True
            rows.append(row);cache[key]=row;save(out/'summary.json',rows);report(out,rows,boundaries)
            print('RESULT',json.dumps({k:row.get(k) for k in ('name','simulation_race_time_s','simulation_outcome',
                  'body_exit_count','failure','planner_max_ms','rejections')},ensure_ascii=False),flush=True)
        x,y=(cache[(f,amplitude,m)] for m in METHODS)
        assert x['world_sha256']==y['world_sha256'] and x['scene_manifest']==y['scene_manifest']

    pair(1.,2.,'baseline')
    for route,levels in ROUTES.items():
        first={}
        for f,amplitude in levels:
            pair(f,amplitude,route)
            for m in METHODS:
                row=cache[(f,amplitude,m)]
                if not row['simulation_passed'] and m not in first:
                    first[m]=dict(frequency=f,amplitude=amplitude,outcome=row['simulation_outcome'])
            if len(first)==2:break
        boundaries[route]={m:first.get(m,dict(outcome='no_failure_observed_up_to_cap')) for m in METHODS}
        save(out/'boundaries.json',boundaries);report(out,rows,boundaries)
    assert sources()==hashes
    save(out/'verification.json',dict(source_count=len(hashes),source_changes=changed,algorithm_changes=[],
         snapshot_changes=[rel for rel,h in original_hashes.items() if digest(out/'source_snapshot'/rel)!=h],
         matched_worlds=True,matched_configs=True,soft_only_execution_verified=True))
    save(out/'status.json',dict(state='complete',trials=len(rows),attempts=len(attempts)))


if __name__=='__main__':main()
