"""Complete 17 pairs while preserving six source-identical development flights."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import sources,save,digest
from convex_timevarying_window.comparisons.single_gate_swing.run import CASES,pair_check
from convex_timevarying_window.comparisons.single_gate_swing.contacts import parse_contacts
from .full_analysis import summarize

HERE=Path(__file__).resolve().parent
MODULE='convex_timevarying_window.multistart_minco.flight'
REQUIRED=('result.json','comparison_audit.json','flight.ulg','contact_publishers.json',
          'gate_01_contacts.pbtxt','gate_02_contacts.pbtxt','world.sdf')


def verify_record(row):
    path=Path(row['directory'])
    for name in REQUIRED:
        if not (path/name).is_file():raise RuntimeError('missing evidence: '+str(path/name))
    raw=json.loads((path/'result.json').read_text())
    if raw['method']!='multistart_minco_'+row['variant'] or raw['case']!={'frequency':row['frequency_hz'],'amplitude':row['amplitude_m'],'phase':0.}:
        raise RuntimeError('record method or case mismatch')
    publishers=json.loads((path/'contact_publishers.json').read_text())
    if len(publishers)!=2 or not all('gz.msgs.Contacts' in p for p in publishers.values()):
        raise RuntimeError('contact publisher evidence missing')
    contacts=[parse_contacts(path/f'gate_{i:02d}_contacts.pbtxt',30.+raw['last_time_s']) for i in (1,2)]
    if contacts!=raw['contacts'] or bool(any(c['contact_pair_count'] for c in contacts))!=bool(row['collision_detected']):
        raise RuntimeError('contact audit mismatch')
    if not (path/'flight.ulg').stat().st_size:raise RuntimeError('empty ULog')
    return {name:digest(path/name) for name in REQUIRED}


def fly(root,out,frequency,amplitude,variant,rows):
    name=f'a{amplitude:g}_f{frequency:g}_{variant}'
    for attempt in range(2):
        path=out/(name if attempt==0 else name+'_facility_retry')
        save(out/'status.json',dict(state='running',pid=os.getpid(),completed=len(rows),total=34,
            reused=6,new_completed=sum(not r.get('reuse_from_previous_batch') for r in rows),new_total=28,
            frequency_hz=frequency,amplitude_m=amplitude,variant=variant,attempt=attempt,directory=str(path)))
        print('START',path.name,flush=True)
        command=[sys.executable,'-u','-m',MODULE,'--frequency',str(frequency),'--amplitude',str(amplitude),'--method',variant,'--outdir',str(path)]
        env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',GZ_SHOW_GUI='0')
        with (out/(path.name+'.log')).open('w') as log:
            with subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True) as proc:
                try:code=proc.wait(timeout=600)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM)
                    try:proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                    raise RuntimeError('flight_process_timeout:'+path.name)
        raw=json.loads((path/'result.json').read_text()) if (path/'result.json').exists() else None
        unflown=(raw is not None and not raw['solves'] and raw['last_time_s']==0
            and any(s in str(raw['failure']) for s in ('arm failed','no heartbeat','initial telemetry')))
        if unflown and attempt==0:
            save(path/'facility_retry_reason.json',dict(unflown=True,failure=raw['failure'],original_retained=True,next_attempt=name+'_facility_retry'))
            print('UNFLOWN_FACILITY_RETRY',name,raw['failure'],flush=True)
            continue
        if (path/'comparison_audit.json').exists():
            row=json.loads((path/'comparison_audit.json').read_text())
            row.update(directory=str(path),process_returncode=code,planning_requests=len(raw['solves']),
                offline_found=raw['planner_process']['offline_diagnostic']['found'],reuse_from_previous_batch=False)
            if code:row.update(verdict='evidence_failure',passed=False,flight_time_s=None)
        else:
            row=dict(frequency_hz=frequency,amplitude_m=amplitude,variant=variant,directory=str(path),
                verdict='evidence_failure',passed=False,flight_time_s=None,
                failure=raw['failure'] if raw else 'missing_result_or_audit',process_returncode=code,reuse_from_previous_batch=False)
        return row


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);p.add_argument('--reuse',type=Path,required=True);a=p.parse_args()
    root=HERE.parents[4];out=a.outdir.resolve();old=a.reuse.resolve()
    if not out.is_relative_to(HERE.parent/'results'):raise ValueError('keep results under experiment1/results')
    original=json.loads((old/'protocol.json').read_text())['source_sha256']
    now=sources(root)
    changed=[rel for rel,h in original.items() if now.get(rel)!=h]
    added=sorted(set(now)-set(original))
    allowed={str((HERE/f).relative_to(root)) for f in ('full_run.py','full_analysis.py')}
    if changed or set(added)!=allowed:raise RuntimeError('reuse source mismatch: '+repr(dict(changed=changed,added=added)))
    state=json.loads((old/'status.json').read_text())
    if state['state']!='complete' or not state['sources_unchanged']:raise RuntimeError('prior batch not frozen complete')
    rows=json.loads((old/'summary.json').read_text())
    if len(rows)!=6 or len({(r['amplitude_m'],r['frequency_hz'],r['variant']) for r in rows})!=6:
        raise RuntimeError('unexpected prior records')
    retained=[]
    for r in rows:
        if not r['passed']:raise RuntimeError('unexpected prior verdict')
        retained.append(dict(directory=r['directory'],evidence_sha256=verify_record(r)))
        r.update(reuse_from_previous_batch=True,reuse_source=str(old),reuse_reason='all existing source entries SHA-256 identical; only two orchestration/analysis files added')
    out.mkdir(parents=True,exist_ok=False)
    save(out/'reuse_audit.json',dict(prior_batch=str(old),prior_source_entries=len(original),all_prior_source_hashes_unchanged=True,
        added_coordination_files_only=added,retained_records=retained,valid_previous_flights=6,new_flights=28))
    save(out/'protocol.json',dict(cases=[dict(frequency=f,amplitude=amp,phase=0.) for f,amp in CASES],
        repetitions=1,unflown_facility_retries=1,source_sha256=now,module=MODULE,
        reused_flights=6,new_flights=28,online_budget_ms=None,max_task_s=40,physics_step_s=.001,
        protocol=(HERE/'FULL_PROTOCOL.md').read_text()))
    for rel in now:
        target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(root/rel,target)
    save(out/'summary.json',rows);summarize(out)
    try:
        for i,(f,amp) in enumerate(CASES):
            for variant in (('ours','center') if i%2==0 else ('center','ours')):
                if sources(root)!=now:raise RuntimeError('frozen_source_changed')
                if any((r['frequency_hz'],r['amplitude_m'],r['variant'])==(f,amp,variant) for r in rows):
                    print('RETAIN',f' a{amp:g}_f{f:g}_{variant}',flush=True);continue
                row=fly(root,out,f,amp,variant,rows);rows.append(row);save(out/'summary.json',rows)
                print('RESULT',Path(row['directory']).name,row['verdict'],row.get('flight_time_s'),flush=True)
                if row['verdict']=='evidence_failure':raise RuntimeError('infrastructure_or_audit_failure:'+row['directory'])
                verify_record(row);summarize(out)
            pair_check(out,rows,f,amp)
        if sources(root)!=now:raise RuntimeError('frozen_source_changed_at_end')
        for item in retained:
            if {name:digest(Path(item['directory'])/name) for name in REQUIRED}!=item['evidence_sha256']:
                raise RuntimeError('retained evidence changed')
        save(out/'status.json',dict(state='complete',pid=os.getpid(),completed=len(rows),total=34,reused=6,new_completed=28,new_total=28,
            sources_unchanged=True,retained_evidence_unchanged=True))
        summarize(out,final=True);print('COMPLETE',len(rows),flush=True)
    except Exception as exc:
        save(out/'status.json',dict(state='stopped',pid=os.getpid(),completed=len(rows),total=34,error=repr(exc)))
        raise


if __name__=='__main__':main()
