"""Replay only preserved planner requests, never a Gazebo flight or a new seed source."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import time
import numpy as np
from convex_timevarying_window.center_initialized_minco.planner import Planner,Config
from convex_timevarying_window.comparisons.single_gate_swing.scenario import Case,build_case
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import sources,save

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[4]
CASES=((2.,2.5),(2.,3.),(4.,1.5))


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args();out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    frozen=sources(ROOT);save(out/'protocol.json',dict(cases=CASES,config=asdict(Config()),source_sha256=frozen,
        input='saved previous free-point requests',same_head_and_absolute_start=True,offline_diagnostic_only=True,used_as_online_seed=False))
    for rel in frozen:
        dest=out/'source_snapshot'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,dest)
    rows=[]
    for amp,freq in CASES:
        source=HERE.parent/'results/translation_v1_20261105'/f'a{amp:g}_f{freq:g}_ours/result.json';raw=json.loads(source.read_text());accepted=next(s for s in raw['solves'] if s['accepted'])
        head=np.array(accepted['request_head']['pvaj']);at=accepted['made_at'];track,_=build_case(Case(freq,amp,0.));began=time.perf_counter();planner=Planner(track,Config());built=time.perf_counter()-began
        error=None;plan=None;began=time.perf_counter()
        try:plan=planner.solve(head,at,0)
        except Exception as exc:error=repr(exc)
        row=dict(amplitude_m=amp,frequency_hz=freq,source_request=str(source),head_pvaj=head.tolist(),absolute_start_s=at,
            old_free_nominal_s=float(sum(accepted['durations'])),build_s=built,request_ms=1000*(time.perf_counter()-began),error=error)
        if plan is not None:
            center=planner.last_center_plan;row.update(center_s=float(center.durations.sum()),returned_s=float(plan.durations.sum()),
                selected_stage=plan.optimizer_stats['selected_stage'],stats=plan.optimizer_stats,
                durations=plan.durations.tolist(),coefficients=plan.coefficients.tolist())
            if not plan.durations.sum()<=center.durations.sum():raise RuntimeError('nominal incumbent guarantee broken')
        rows.append(row);save(out/'result.json',rows);print('PROBE',amp,freq,{k:row.get(k) for k in ('old_free_nominal_s','center_s','returned_s','selected_stage','request_ms','error')},flush=True)
    unchanged=sources(ROOT)==frozen
    save(out/'status.json',dict(state='complete' if unchanged and not any(r['error'] for r in rows) else 'failed',requests=len(rows),source_unchanged=unchanged))
    if not unchanged or any(r['error'] for r in rows):raise RuntimeError('probe failed; inspect preserved result')


if __name__=='__main__':main()
