"""Serial 4m/4Hz physical timestep checks; retain original 1ms results."""
import json
import os
from pathlib import Path
import subprocess
import sys
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import digest,save

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]

def main():
    out=HERE/'results/resolution_v1_20261105';out.mkdir(exist_ok=False)
    frozen=json.loads((HERE/'results/translation_v1_20261105/protocol.json').read_text())['source_sha256']
    save(out/'protocol.json',dict(source_sha256=frozen,step_s=[.0005,.00025],frequency_hz=4.,amplitude_m=4.,repetitions=1,
        purpose='independent physical-resolution check, not replacement or best-of repeats',wrapper_sha256=digest(HERE/'check_resolution.py')))
    rows=[]
    try:
        for step in (.0005,.00025):
            for method in ('ours','center'):
                if any(digest(ROOT/rel)!=h for rel,h in frozen.items()):raise RuntimeError('original frozen method changed')
                path=out/f'step{step:g}_{method}'
                save(out/'status.json',dict(state='running',pid=os.getpid(),completed=len(rows),total=4,step_s=step,variant=method,directory=str(path)))
                print('START',path.name,flush=True)
                with (out/(path.name+'.log')).open('w') as stream:
                    result=subprocess.run([sys.executable,'-u','-m',__package__+'.check_resolution','--step',str(step),'--frequency','4','--amplitude','4','--method',method,'--outdir',str(path)],cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,timeout=600)
                if not (path/'comparison_audit.json').exists():raise RuntimeError('missing physical resolution audit')
                row=json.loads((path/'comparison_audit.json').read_text());row.update(step_s=step,directory=str(path),process_returncode=result.returncode)
                rows.append(row);save(out/'summary.json',rows);print('RESULT',path.name,row['verdict'],row.get('flight_time_s'),flush=True)
                if result.returncode:raise RuntimeError('resolution flight infrastructure failure')
        save(out/'status.json',dict(state='complete',completed=4,total=4,original_source_unchanged=all(digest(ROOT/rel)==h for rel,h in frozen.items())))
    except Exception as exc:
        save(out/'status.json',dict(state='stopped',completed=len(rows),total=4,error=repr(exc)));raise

if __name__=='__main__':main()
