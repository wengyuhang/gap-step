"""Timed controller selection, NOT simulated lap times or PX4 latency."""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import shutil
import time
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track
from .controller import make_controller
from .config import Config
from .rollout import hover_state,KF


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    a.outdir.mkdir(parents=True,exist_ok=False)
    hashes={}
    for source in Path(__file__).parent.rglob('*'):
        rel=source.relative_to(Path(__file__).parent)
        if source.suffix not in ('.py','.cpp') or any(x in rel.parts for x in ('results','build','install','log')):continue
        hashes[str(rel)]=hashlib.sha256(source.read_bytes()).hexdigest()
        dest=a.outdir/'source_snapshot'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)
    track,_=build_track();vary_track(track,.4,0,1,1,1)
    old=Path(__file__).parent/'results/passage_20260920_04/trace.npz'
    last=np.load(old)['rows'][-1]
    states=[('start',hover_state(track.start),0.,0),('old_failure_state',last[1:18],float(last[0]),int(last[18]))]
    rows=[]
    for backend,n in [('cpu',512)]+[('cuda',n) for n in (512,4096,8192,16384,32768)]:
        c=make_controller(track,replace(Config(),samples=n),0,backend)
        for name,state,now,first in states:
            # Fixed motor-hold baseline for all sample counts; not a replay of
            # the lost original cache and not a proof of the failure cause.
            c.initial_controls=np.tile(KF*state[13:]**2,(50,1));durations=[];safe=[];accepted=[]
            c.select(state,now,first,avoid=True)
            for _ in range(12):
                tick=time.perf_counter();selection=c.select(state,now,first,avoid=True)
                durations.append((time.perf_counter()-tick)*1000)
                safe.append(c.last_diagnostics['safe_samples']);accepted.append(selection is not None)
            r=dict(backend=backend,samples=n,state=name,median_ms=float(np.median(durations)),
                max_ms=max(durations),over_20ms=sum(v>20 for v in durations),over_100ms=sum(v>100 for v in durations),
                safe_min=min(safe),safe_max=max(safe),admitted=sum(accepted),repeats=12,
                durations_ms=durations,safe_counts=safe)
            rows.append(r);print(json.dumps(r),flush=True)
    import cupy as cp
    report=dict(device=cp.cuda.runtime.getDeviceProperties(0)['name'].decode(),source_sha256=hashes,
        evidence='Warmed complete select wall time incl CPU FP64 final admission; no PX4 transport; no closed-loop success claim',
        failure_state_baseline='fixed current-motor hold, not the historical cache',rows=rows)
    (a.outdir/'summary.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
