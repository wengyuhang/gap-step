"""Report actual native contacts, event replay, timings and frozen sources."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from ..events import Progress
from ..planner import route_signs
from ..scenario import build_case,Case


def main():
    p=argparse.ArgumentParser();p.add_argument('directory');args=p.parse_args()
    out=Path(args.directory);result=json.loads((out/'result.json').read_text())
    flight=json.loads((out/'flight.json').read_text());mpc=json.loads((out/'mpc.json').read_text())
    plans=json.loads((out/'plans.json').read_text());attempts=json.loads((out/'planning_attempts.json').read_text())
    race=[v for v in flight if v['phase']=='race'];track,_=build_case(Case(4,4,0))
    radius=result.get('body_envelope',{}).get('planning_radius_m',.38594472)
    progress=Progress(list(track.windows),route_signs(track),radius,.085)
    for row in race:progress.observe(np.asarray(row['state'][:3]),row['time'])
    errors=[np.linalg.norm(np.array(v['state'][:3])-v['reference']) for v in race]
    times=np.diff([v['timestamp'] for v in race])*1e-9
    hashes=json.loads((out/'source_sha256.json').read_text()) if (out/'source_sha256.json').exists() else {}
    changed=[name for name,h in hashes.items() if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=h]
    correction=out/'COLLISION_CORRECTION.json'
    audit=dict(status=result['status'],completed=bool(result['completed']),
        native_contacts=result.get('contacts',[]),independent_event_replay=dict(
        body_exits=progress.exits,crossings=progress.crossings,violation=progress.violation),
        source_changes=changed,actual_state_sample_gap_s=np.quantile(times,[0,.5,.95,1]).tolist() if len(times) else [],
        tracking_error_m=np.quantile(errors,[.5,.95,1]).tolist() if errors else [],
        mpc_max_ms=max((r['cycle_ms'] for r in mpc),default=0),
        accepted_planner_max_ms=max((r['solve_ms'] for r in plans),default=0),
        failed_planner_max_ms=max((r['wall_ms'] for r in attempts),default=0),
        flight_time_s=result['complete_race_time_s'],
        limits=['Native physics lockstep, not real-time','Approximate Cylinder frames, not exact STL',
                'Discrete gate poses, no continuous moving-collision certificate',
                'Body envelope from native visual bounding box, not extracted collision hull'])
    if correction.exists():
        audit['collision_correction']=json.loads(correction.read_text())
        audit['completed']=False
    audit['seven_gate_return_verified']=bool(audit['completed'] and len(progress.exits)==7
        and not progress.violation and not audit['native_contacts'] and not correction.exists())
    (out/'audit.json').write_text(json.dumps(audit,indent=2))
    text=f"# AirSim 4 Hz / 4 m 原生闭环开发试飞\n\n状态：{audit['status']}。"
    text+=f"独立重放计窗：{len(progress.exits)}/7；完成七窗并返航：{audit['seven_gate_return_verified']}。\n\n"
    text+=f"完整竞速时间：{audit['flight_time_s']} 秒（未完成不记有效时间）。\n\n"
    text+=f"原生接触记录数：{len(audit['native_contacts'])}。"
    if correction.exists():text+=' 本趟原碰撞字段无效，独立碰撞接口证实第一窗碰撞，见 COLLISION_CORRECTION.json。'
    text+='\n\n'
    text+=f"跟踪误差中位/P95/最大：{audit['tracking_error_m']} 米。\n\n"
    text+=f"MPC最大：{audit['mpc_max_ms']:.3f} ms；接纳规划最大：{audit['accepted_planner_max_ms']:.3f} ms；失败规划最大：{audit['failed_planner_max_ms']:.3f} ms。\n\n"
    text+=f"实际状态采样间隔最小/中位/P95/最大：{audit['actual_state_sample_gap_s']} 秒。\n\n"
    text+=f"冻结源码变动：{changed}。\n\n"
    text+='本次为原生动力学、电机输入的锁步飞行，不是位置回放；也不是实时在线达标或连续安全证明。窗框为近似网格，远窗低频更新。'
    (out/'REPORT.md').write_text(text+'\n');print(json.dumps(audit,ensure_ascii=False))


if __name__=='__main__':main()
