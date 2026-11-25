"""Condition-matched comparison; failed runs never supply finish times."""
import argparse
import json
from pathlib import Path
import numpy as np


def key(r):
    return (r['rate'],r['translation_scale'],r['rotation_scale'],r['phase_id'])


def compare(old_root,new_root):
    old_root,new_root=Path(old_root),Path(new_root)
    old={key(r):r for r in json.loads((old_root/'summary.json').read_text()) if not r.get('infrastructure_failure')}
    new={key(r):r for r in json.loads((new_root/'summary.json').read_text()) if not r.get('infrastructure_failure')}
    paired=[];rows=[];cycles=[];incomplete=0
    for k,b in new.items():
        a=old[k]
        delta=b['flight_time_s']-a['flight_time_s'] if a['physical_success'] and b['physical_success'] else None
        if delta is not None:paired.append(delta)
        run=Path(b['run']);trace_path=run/'telemetry.csv'
        trace=np.genfromtxt(trace_path,delimiter=',',names=True) if trace_path.exists() else None
        raw=json.loads((run/'result.json').read_text()) if (run/'result.json').exists() else {}
        timing=b.get('cycle_ms')
        if not timing and raw.get('solves'):timing={'max':max(s['cycle_ms'] for s in raw['solves'])}
        if timing:cycles.append(timing['max'])
        missing=bool(b.get('analysis_error') and raw.get('completed') and not raw.get('collision_detected'))
        incomplete+=missing
        rows.append(dict(condition=k,old_success=a['physical_success'],new_success=b['physical_success'],
                         new_status='INCOMPLETE_EVIDENCE' if missing else ('PASS' if b['physical_success'] else 'FAIL'),
                         old_finish_s=a['flight_time_s'] if a['physical_success'] else None,
                         new_finish_s=b['flight_time_s'] if b['physical_success'] else None,
                         delta_finish_s=delta,new_max_tracking_error_m=float(np.max(trace['error'])) if trace is not None and trace.size else None,
                         new_clearance_m=b.get('min_clearance_m'),new_max_cycle_ms=timing['max'] if timing else None))
    summary=dict(completed_conditions=len(new),old_successes=sum(old[k]['physical_success'] for k in new),
                 new_successes=sum(b['physical_success'] for b in new.values()),
                 new_realtime_successes=sum(b['realtime_success'] for b in new.values()),
                 common_successes=len(paired),paired_mean_delta_s=float(np.mean(paired)) if paired else None,
                 incomplete_evidence_conditions=incomplete,unknown_timing_conditions=len(new)-len(cycles),
                 new_max_cycle_ms=max(cycles) if cycles else None,conditions=rows)
    (new_root/'comparison.json').write_text(json.dumps(summary,indent=2)+'\n')
    lines=['# 可停车延续算法：冻结对照','',f"完成{len(new)}/16条件；旧方法严格通过{summary['old_successes']}/{len(new)}，新方法严格通过{summary['new_successes']}/{len(new)}。",
           f'新批次另有{incomplete}个完成且零接触但审计证据不完整的条件；不计为严格成功。',
           '相同场景、相位、PX4参数和验收规则；算法源码及配置在各批次内冻结。两个固定相位不代表总体成功率。',
           '', '|频率|平移/摆角幅度|旧成功|新成功|旧成功均时 s|新成功均时 s|', '|---:|---:|---:|---:|---:|---:|']
    def average(rr):
        tt=[r['flight_time_s'] for r in rr if r['physical_success']]
        return f'{np.mean(tt):.3f}' if tt else '—'
    for group in sorted({k[:3] for k in new}):
        keys=[k for k in new if k[:3]==group];aa=[old[k] for k in keys];bb=[new[k] for k in keys]
        lines.append(f"|{group[0]}|{group[1]}/{group[2]}|{sum(r['physical_success'] for r in aa)}/{len(aa)}|{sum(r['physical_success'] for r in bb)}/{len(bb)}|{average(aa)}|{average(bb)}|")
    lines+=['',f"已保存周期的最大值：{summary['new_max_cycle_ms']} ms；{summary['unknown_timing_conditions']}个条件缺少完整周期记录。",f"共同成功的{len(paired)}个配对条件，平均新−旧圈时：{summary['paired_mean_delta_s']} s。",
            '各组成功均时可能来自不同条件；速度比较应优先看共同成功配对，失败不算圈时。',
            '真值采样与接触日志不构成连续域安全证明；经验跟踪管仍无误差上界保证。','', '## 未通过完整验收的条件（含证据缺失，不等同碰撞）','']
    for k,b in new.items():
        if not b['physical_success']:
            if b.get('analysis_error'):
                lines.append(f"- {k}：真值审计证据缺失，不计为严格成功，也不等同碰撞；audit_error={b['analysis_error']}。原始飞行状态见result.json及LOGGING_NOTE.md。")
                continue
            lines.append(f"- {k}：{b.get('gates')}门；failure={b.get('failure')}；contact={b.get('collision')}；workspace={b.get('room_passed')}；clearance={b.get('min_clearance_m')}；audit_error={b.get('analysis_error')}。")
    (new_root/'COMPARISON.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('old_root');p.add_argument('new_root');a=p.parse_args()
    compare(a.old_root,a.new_root)
