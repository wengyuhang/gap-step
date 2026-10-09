"""Frozen one-shot factorial matrix with recoverable per-attempt records."""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from .registry import trials
from .audit import audit
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,manifest
from 复现.polynomial_dynamic_racing_2023.all_versions_current_track.run import source_hashes as wang_hashes
from 复现.polynomial_dynamic_racing_2023.px4_original_track_legacy.run import save,digest,ROOT
from 复现.polynomial_dynamic_racing_2023.all_versions_current_track.run import infrastructure_arm_failure
from convex_timevarying_window.comparisons.original_fixed_plane_crossing_resolved.run import source_hashes as ours_hashes
from convex_timevarying_window.comparisons.checked_soft_racing.adjudicate import assess


def sources():
    hashes=wang_hashes();hashes.update(ours_hashes())
    for directory in (Path(__file__).parent,ROOT/'复现/polynomial_dynamic_racing_2023/px4_matched_x500'):
        for p in directory.rglob('*'):
            if 'results' not in p.parts and p.suffix in ('.py','.md'):
                hashes[str(p.relative_to(ROOT))]=digest(p)
    return hashes


def execute(trial,path,log_path,rho):
    module='复现.polynomial_dynamic_racing_2023.px4_matched_x500.flight' if trial.method=='wang' else 'convex_timevarying_window.comparisons.comprehensive_wang.flight'
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
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--readiness',type=Path,required=True);p.add_argument('--resume',action='store_true');a=p.parse_args()
    out=a.outdir.resolve();ready=json.loads(a.readiness.read_text())
    if not ready.get('accepted_for_matrix'):raise RuntimeError('Readiness review has not accepted the baseline')
    hashes=sources();schedule=trials();out.mkdir(parents=True,exist_ok=a.resume)
    if a.resume:
        protocol=json.loads((out/'protocol.json').read_text())
        if protocol['source_sha256']!=hashes:raise RuntimeError('Frozen source manifest changed')
        if protocol['readiness']!=ready:raise RuntimeError('Frozen baseline configuration changed')
        rows=json.loads((out/'summary.json').read_text());attempts=json.loads((out/'attempts.json').read_text())
    else:
        rows=[];attempts=[]
        save(out/'protocol.json',dict(created=time.time(),source_sha256=hashes,readiness=ready,
            trials=[dict(asdict(t),name=t.name,pair=t.pair) for t in schedule],
            protocol=Path(__file__).with_name('PROTOCOL.md').read_text(),repeat_count=1))
        for rel in hashes:
            target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,target)
        save(out/'summary.json',rows);save(out/'attempts.json',attempts)
    finished={r['name'] for r in rows}
    for trial in schedule:
        if trial.name in finished:continue
        if sources()!=hashes:raise RuntimeError('Sources changed during batch')
        if shutil.disk_usage(out).free<3*1024**3:raise RuntimeError('Less than 3GiB disk remaining; no data removed')
        for attempt in (1,2):
            path=out/(trial.name+f'_attempt{attempt}')
            if path.exists():raise RuntimeError(f'Unadjudicated prior attempt: {path}')
            save(out/'status.json',dict(state='running',done=len(rows),total=len(schedule),trial=trial.name,attempt=attempt,pid=os.getpid()))
            print('START',len(rows)+1,len(schedule),trial.name,attempt,flush=True)
            row=execute(trial,path,out/(path.name+'.log'),ready['rho']);row['attempt']=attempt
            attempts.append(row);save(out/'attempts.json',attempts)
            if not row['infrastructure_failure'] or attempt==2:break
        same=[r for r in rows if r['pair']==trial.pair and 'world_sha256' in r]
        for previous in same:
            for field in ('world_sha256','scene_manifest','mpc_config','px4_parameters'):
                if row.get(field)!=previous.get(field):raise RuntimeError(f'Paired mismatch: {field}')
        rows.append(row);save(out/'summary.json',rows)
        print('RESULT',json.dumps({k:row.get(k) for k in ('name','race_passed','race_time_s','body_exit_count','failure','infrastructure_failure','audit_error')},ensure_ascii=False),flush=True)
        if row.get('audit_error'):raise RuntimeError('Audit implementation/missing-data error; preserve run and inspect')
    verification=dict(source_count=len(hashes),source_changes=[p for p,h in hashes.items() if digest(ROOT/p)!=h],
        snapshot_changes=[p for p,h in hashes.items() if digest(out/'source_snapshot'/p)!=h])
    save(out/'verification.json',verification)
    assert not verification['source_changes'] and not verification['snapshot_changes']
    save(out/'status.json',dict(state='complete',done=len(rows),total=len(schedule),finished=time.time()))
    subprocess.run([sys.executable,'-m','convex_timevarying_window.comparisons.comprehensive_wang.report',str(out)],cwd=ROOT,check=True)
    print('COMPLETE',out,flush=True)


if __name__=='__main__':main()
