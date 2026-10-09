"""Two predeclared development flights; original matrix stays paused."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

from convex_timevarying_window.comparisons.fixed_plane_wang.run import source_hashes as frozen_hashes
from convex_timevarying_window.comparisons.fixed_plane_wang.audit import inspect_run
from convex_timevarying_window.comparisons.checked_soft_racing.run import ROOT,save

CASES=[(.8,1.6,.4),(.6,1.2,0.)]
OLD=ROOT/'convex_timevarying_window/comparisons/fixed_plane_wang/results'
BASELINES=[OLD/'pilot_20261007',OLD/'frozen_20261007']
DEADLINE_BASELINE=ROOT/'convex_timevarying_window/wang_progress_mpc/results/deadline_v1_20261007'


def source_hashes():
    result=frozen_hashes()
    for path in sorted(Path(__file__).parent.rglob('*')):
        if 'results' not in path.parts and path.suffix in ('.py','.md'):
            result[str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def mechanism_audit(path):
    import numpy as np
    raw=json.loads((path/'result.json').read_text()); failures=[]
    for s in raw['solves']:
        if 'initial_variables' not in s: continue
        gates=s['gate_count']; legs=len(s['durations'])//2; qi=4*legs+6*gates
        if legs!=gates+int(s['terminal_goal']): failures.append('extra_segment')
        if not np.all(np.asarray(s['initial_variables'][qi:])==0): failures.append('noncenter_seed')
        if s['terminal_goal'] != (s['candidate_first_gate']+gates==7): failures.append('local_stop')
    if raw['config']['horizon']!=3 or raw['passage_deadlines'] or raw['local_stop_terminal']:
        failures.append('configuration_contract')
    audit=dict(passed=not failures,failures=failures,requests=len(raw['solves']))
    save(path/'mechanism_audit.json',audit); return audit


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False);hashes=source_hashes()
    save(out/'protocol.json',dict(cases=CASES,source_sha256=hashes,created=time.time(),
        method='three_gate_center_seed_no_local_stop_v1',max_flight_time_s=60,timeout_s=240,
        retry='One retry only for independently confirmed infrastructure failure',
        old_matrix='Paused and never receives the new version',
        comparison='Two development conditions, same saved Wang and old method flights; not a full matrix'))
    for rel in hashes:
        dest=out/'source_snapshot'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,dest)
    rows=[]
    for i,case in enumerate(CASES):
        for attempt in (1,2):
            if source_hashes()!=hashes:raise RuntimeError('Development version changed during flights')
            path=out/f'case{i:02d}_attempt{attempt}'
            save(out/'status.json',dict(state='running',case=i,attempt=attempt,run=str(path),pid=os.getpid()))
            print('START '+path.name,flush=True)
            command=[sys.executable,'-u','-m','convex_timevarying_window.wang_center_seed_mpc.flight',
                '--frequency',str(case[0]),'--amplitude',str(case[1]),'--phase',str(case[2]),'--outdir',str(path)]
            start=time.monotonic();code=None
            try:
                proc=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,timeout=240)
                output=proc.stdout+'\n'+proc.stderr;code=proc.returncode
            except subprocess.TimeoutExpired as exc:output=repr(exc)
            path.mkdir(exist_ok=True);(path/'console.log').write_text(output)
            save(path/'batch_source_reference.json',dict(protocol=str(out/'protocol.json'),source_sha256=hashes))
            row=dict(method='wang_center_seed_mpc',condition_id=case,case=i,attempt=attempt,run=str(path),returncode=code,wall_s=time.monotonic()-start)
            if (path/'result.json').exists():
                row.update(inspect_run(path,'ours'))
                row['mechanism_audit']=mechanism_audit(path)
                row['realtime_success'] &= row['mechanism_audit']['passed']
            else:
                row.update(physical_success=False,realtime_success=False,infrastructure_failure=False,failure='missing_result')
            rows.append(row);save(out/'summary.json',rows)
            print('RESULT '+json.dumps(row,ensure_ascii=False),flush=True)
            if not row['infrastructure_failure']:break
    if source_hashes()!=hashes:raise RuntimeError('Sources changed')
    save(out/'status.json',dict(state='complete',attempts=len(rows),finished=time.time()))
    selected=[next(r for r in reversed(rows) if r['case']==i) for i in range(2)]
    comparison=[]
    for i,row in enumerate(selected):
        old=json.loads((BASELINES[i]/'case00_ours_attempt1/result.json').read_text())
        wang=json.loads((BASELINES[i]/'case00_wang_attempt1/result.json').read_text())
        deadline=json.loads((DEADLINE_BASELINE/f'case{i:02d}_attempt1/result.json').read_text())
        comparison.append(dict(condition_id=CASES[i],new_passed=row['realtime_success'],
            new_time_s=row.get('flight_time_s') if row['realtime_success'] else None,
            old_time_s=old['flight_time_s'],wang_time_s=wang['flight_time_s'],
            deadline_version_time_s=deadline['flight_time_s'],failure=row.get('failure')))
    save(out/'comparison.json',comparison)
    print('COMPARISON '+json.dumps(comparison,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
