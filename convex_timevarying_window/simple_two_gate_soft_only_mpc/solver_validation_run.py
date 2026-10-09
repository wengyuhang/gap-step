"""One frozen-source flight; no mixing with the old sweep."""
import argparse
import json
import shutil
from pathlib import Path
from .run import sources, execute, evaluate, save, ROOT, Trial


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--frequency',type=float,default=2.5)
    p.add_argument('--amplitude',type=float,default=3.5)
    a=p.parse_args();out=a.outdir.resolve();out.mkdir(parents=True)
    hashes=sources();trial=Trial('solver_validation','ours',a.frequency,a.amplitude,0.,0)
    save(out/'protocol.json',dict(source_sha256=hashes,frequency=a.frequency,amplitude=a.amplitude,phase=0.,
         method='ours',repeats=1,change='L-BFGS-B numerical stagnation repair only',
         time_metric='complete start-seven-gates-return',safety_metric='physical window-frame contacts',
         validation='new PX4/Gazebo flight, not identical initial-state replay'))
    for rel in hashes:
        dst=out/'source_snapshot'/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,dst)
    save(out/'status.json',dict(state='running',case=[a.frequency,a.amplitude],completed=0))
    print('START',trial.name,flush=True)
    path=out/trial.name
    row=evaluate(execute(trial,path,out/(trial.name+'.log'),100))
    save(out/'summary.json',[row])
    if not (path/'result.json').exists():
        save(out/'status.json',dict(state='missing_result',directory=str(path)))
        raise RuntimeError('Flight produced no result')
    raw=json.loads((path/'result.json').read_text())
    stats=dict(outcome=row.get('simulation_outcome'),complete_time_s=row.get('simulation_race_time_s'),
       collision=raw['collision_detected'],failure=raw['failure'],body_exits=len(raw['body_exits']),
       audit_error=row.get('audit_error'),planning_calls=len(raw['solves']),
       error_replans=sum(e['action']=='trigger' and e['reason']=='tracking_error' for e in raw['planning_events']),
       planner_max_ms=max((s['cycle_ms'] for s in raw['solves']),default=None),
       planner_over_100ms=sum(s['cycle_ms']>=100 for s in raw['solves']),
       repaired_calls=sum(bool(s.get('optimizer_stats',{}).get('stagnation_repairs')) for s in raw['solves']),
       first_planning_stats=raw['solves'][0].get('optimizer_stats') if raw['solves'] else None,
       mpc_max_ms=max((c['cycle_ms'] for c in raw['mpc_cycles']),default=None),
       source_unchanged=sources()==hashes)
    save(out/'verification.json',stats)
    lines=[f'# L-BFGS-B停滞修复：{a.frequency:g}Hz/{a.amplitude:g}m自由点实飞','',
      '原初相、每组一次；只改数值求解器，不改MPC/规划目标/安全代价，不进行后验安全准入。','',
      '```json',json.dumps(stats,indent=2,ensure_ascii=False),'```','',
      '完整时间包含七窗和返航。超时不等于碰撞；接触记录只涉及窗框。数值停滞修复不等于安全证明。',
      '求解超过100ms如实记录，不能称已满足在线时限；此次实飞初态不保证与旧轨迹重放逐值一致。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    save(out/'status.json',dict(state='complete' if not row.get('infrastructure_failure') else 'infrastructure_failure',trials=1))
    print('RESULT',json.dumps(stats,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
