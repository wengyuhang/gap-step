"""Small frozen four-condition matrix, sequential, with failures retained."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import math

def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);p.add_argument('--kind',choices=['nominal','px4'],required=True);args=p.parse_args()
    args.outdir.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).parent;files=list(root.glob('*.py'))+[root/'kernel.cpp']
    files+=[Path('convex_timevarying_window/dynamic_gate_mppi_v3')/x for x in ('reference.py','native/rollout.cpp','ctbr_probe.py','geometry.py')]
    hashes={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files}
    cases=[(rate,phase) for rate in (1,10) for phase in (0,2*math.pi/3)]
    protocol=dict(kind=args.kind,cases=cases,source_sha256=hashes,seed=0,samples=16384,duration=60.,translation_scale=1.,rotation_scale=1.,baseline_rerun=False)
    (args.outdir/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    rows=[]
    for i,(rate,phase) in enumerate(cases):
        if any(hashlib.sha256(f.read_bytes()).hexdigest()!=hashes[str(f)] for f in files):raise RuntimeError('frozen implementation changed')
        attempts=[]
        for attempt in range(2 if args.kind=='px4' else 1):
            path=args.outdir/(f'case_{i:02d}'+('_arm_retry' if attempt else ''))
            cmd=[sys.executable,'-u','-m','convex_timevarying_window.three_phase_mppi.'+('px4' if args.kind=='px4' else 'experiment'),
                 '--outdir',str(path),'--phase',str(phase),'--motion-rate',str(rate)]
            print('START',i,attempt,flush=True);started=time.perf_counter()
            (args.outdir/'status.json').write_text(json.dumps(dict(state='running',case=i,attempt=attempt,finished=len(rows)))+'\n')
            with (args.outdir/(path.name+'.log')).open('w') as stream:
                try:code=subprocess.run(cmd,stdout=stream,stderr=subprocess.STDOUT,timeout=200).returncode
                except subprocess.TimeoutExpired:code=None
            artifact=path/('result.json' if args.kind=='px4' else 'summary.json')
            r=json.loads(artifact.read_text()) if artifact.exists() else dict(completed=False,failure='missing_result')
            row=dict(case=i,attempt=attempt,rate=rate,phase=phase,path=str(path),completed=r['completed'],failure=r['failure'],returncode=code,wall_s=time.perf_counter()-started)
            if args.kind=='nominal':row.update(gates=r.get('gates'),time=r.get('time'),cycles=r.get('cycles'))
            else:row.update(gates=len(r.get('crossings',[])),time=r.get('last_time_s'))
            if artifact.exists() and row.get('time',0)>0:
                with (args.outdir/(path.name+'_audit.log')).open('w') as stream:
                    audit=subprocess.run([sys.executable,'-m','convex_timevarying_window.three_phase_mppi.report',str(path)],stdout=stream,stderr=subprocess.STDOUT)
                row['audit_returncode']=audit.returncode
                if (path/'audit.json').exists():row['audit']=json.loads((path/'audit.json').read_text())
            attempts.append(row);print('RESULT',json.dumps(row),flush=True)
            if 'arm failed' not in str(r['failure']):break
        rows.append({**attempts[-1],'attempts':attempts})
        (args.outdir/'summary.json').write_text(json.dumps(rows,indent=2)+'\n')
    (args.outdir/'status.json').write_text(json.dumps(dict(state='finished',finished=len(rows)))+'\n')

if __name__=='__main__':main()
