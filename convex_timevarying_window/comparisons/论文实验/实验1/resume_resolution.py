"""Continue only unflown 0.25ms checks after a verified arming failure."""
import json
import os
from pathlib import Path
import subprocess
import sys
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import digest,save
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]

def main():
    old=HERE/'results/resolution_v1_20261105';failed=old/'step0.00025_ours';raw=json.loads((failed/'result.json').read_text())
    if raw['last_time_s']!=0 or raw['solves'] or 'arm failed' not in str(raw['failure']):raise RuntimeError('not an unflown facility failure')
    out=HERE/'results/resolution_continuation_v1_20261105';out.mkdir(exist_ok=False)
    frozen=json.loads((old/'protocol.json').read_text())['source_sha256'];rows=json.loads((old/'summary.json').read_text())
    save(out/'protocol.json',dict(original_batch=str(old),failure=str(failed),verified_unflown=True,original_retained=True,pending_step_s=.00025,original_source_sha256=frozen))
    try:
        for method in ('ours','center'):
            for attempt in range(1 if method=='ours' else 2):
                if any(digest(ROOT/k)!=v for k,v in frozen.items()):raise RuntimeError('original method changed')
                name=method+('_facility_retry' if method=='ours' or attempt else '')
                path=out/name;save(out/'status.json',dict(state='running',pid=os.getpid(),completed=len(rows),total=4,variant=method,directory=str(path)))
                print('START',name,flush=True)
                with (out/(name+'.log')).open('w') as stream:
                    run=subprocess.run([sys.executable,'-u','-m',__package__+'.check_resolution','--step','.00025','--frequency','4','--amplitude','4','--method',method,'--outdir',str(path)],cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,timeout=600)
                raw=json.loads((path/'result.json').read_text()) if (path/'result.json').exists() else None
                unflown=raw is not None and raw['last_time_s']==0 and not raw['solves'] and 'arm failed' in str(raw['failure'])
                if unflown and method=='center' and attempt==0:
                    save(path/'facility_retry_reason.json',dict(unflown=True,failure=raw['failure'],original_retained=True));continue
                if not (path/'comparison_audit.json').exists():raise RuntimeError('missing audit after allowed facility retries')
                row=json.loads((path/'comparison_audit.json').read_text());row.update(step_s=.00025,directory=str(path),process_returncode=run.returncode)
                rows.append(row);save(out/'summary.json',rows);print('RESULT',name,row['verdict'],row.get('flight_time_s'),flush=True)
                if run.returncode:raise RuntimeError('physical-resolution audit failed')
                break
        save(out/'status.json',dict(state='complete',completed=len(rows),total=4,original_source_unchanged=all(digest(ROOT/k)==v for k,v in frozen.items())))
    except Exception as exc:save(out/'status.json',dict(state='stopped',completed=len(rows),total=4,error=repr(exc)));raise

if __name__=='__main__':main()
