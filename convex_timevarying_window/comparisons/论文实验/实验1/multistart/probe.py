"""Paired identical planner inputs; no flight or reuse as online initialization."""
import argparse
from dataclasses import asdict,replace
import json
from pathlib import Path
import shutil
import time
import numpy as np
from convex_timevarying_window.multistart_minco.planner import Planner,Config
from convex_timevarying_window.comparisons.single_gate_swing.scenario import Case,build_case
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import sources,save

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[4]
CASES=((2.,2.5),(2.,3.),(4.,1.5))


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args();out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    hashes=sources(ROOT);save(out/'protocol.json',dict(cases=CASES,source_sha256=hashes,offline_only=True,used_as_online_seed=False,
        same_pvaj_and_absolute_time_both_methods=True,seed_scales=[1.,.8,1.2,1.5]))
    for rel in hashes:
        target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,target)
    rows=[]
    for amp,freq in CASES:
        prior=HERE.parent/'results/center_initialized_flight_v1_20261105'
        record=next(r for r in json.loads((prior/'summary.json').read_text()) if r['amplitude_m']==amp and r['frequency_hz']==freq and r['variant']=='ours')
        source=Path(record['directory'])/'result.json'
        raw=json.loads(source.read_text());request=next(s for s in raw['solves'] if s['accepted']);head=np.array(request['request_head']['pvaj']);at=request['made_at']
        cfg=Config(**raw['config']);track,_=build_case(Case(freq,amp,0.));began=time.perf_counter();planner=Planner(track,cfg);build_s=time.perf_counter()-began
        low,high=planner.lower.copy(),planner.upper.copy();paired_seeds=[]
        for free in (False,True):
            planner.cfg=replace(cfg,free_points=free);planner.lower[:]=low;planner.upper[:]=high
            if not free:planner.lower[2:4]=planner.upper[2:4]=0.
            began=time.perf_counter();error=None;plan=None
            try:plan=planner.solve(head,at,0)
            except Exception as exc:error=repr(exc)
            row=dict(amplitude_m=amp,frequency_hz=freq,variant='ours' if free else 'center',source_request=str(source),
                head_pvaj=head.tolist(),absolute_start_s=at,config=asdict(planner.cfg),build_s=build_s,request_ms=1000*(time.perf_counter()-began),
                error=error,stats=planner.solvers[0][0].stats())
            if plan is not None:row.update(nominal_duration_s=float(plan.durations.sum()),durations=plan.durations.tolist(),coefficients=plan.coefficients.tolist())
            seeds=[r['seed_variables'] for r in row['stats']['temporal_candidates']];paired_seeds.append(seeds)
            rows.append(row);save(out/'result.json',rows)
            print('PROBE',amp,freq,row['variant'],row.get('nominal_duration_s'),row['request_ms'],error,flush=True)
        if paired_seeds[0]!=paired_seeds[1]:raise RuntimeError('unequal initialization families')
    unchanged=sources(ROOT)==hashes
    save(out/'status.json',dict(state='complete' if unchanged and not any(r['error'] for r in rows) else 'failed',requests=len(rows),sources_unchanged=unchanged))
    lines=['# 双方统一多初值：相同规划请求验证','','固定同一实际PVAJ/绝对时刻，各自直接求四初值最短合格解；不是实飞时间。','',
        '|振幅m|频率Hz|方法|最短名义s|总求解ms|所选初值|全部四条|','|---:|---:|---|---:|---:|---:|---|']
    for r in rows:lines.append('| '+' | '.join(map(str,[r['amplitude_m'],r['frequency_hz'],r['variant'],round(r.get('nominal_duration_s',0),6),round(r['request_ms'],2),r['stats'].get('selected_seed_index'),r['stats'].get('all_time_seeds_attempted')]))+' |')
    lines+=['','全部候选及负结果保留在result.json；同种子家族不保证局部自由解一定优于中心，也不是全局最优证明。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    if not unchanged or any(r['error'] for r in rows):raise RuntimeError('paired nominal probe failed')


if __name__=='__main__':main()
