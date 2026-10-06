"""Feasibility-first racing evaluation for one frozen PX4/Gazebo run."""
import argparse
import json
from pathlib import Path
import numpy as np


def condition_label(condition):
    """Name the actual condition, never label every run as the hardest one."""
    names=('motion_rate','translation_scale','rotation_scale','phase')
    values=[condition.get(name) for name in names]
    return '条件 r/t/a/phase(rad)：'+'/'.join('未知' if x is None else f'{float(x):g}' for x in values)+'。'


def assess(result, audit, sender_checks, baseline=None, execution_rows=None):
    crossings=result.get('crossings',[])
    ordered=[item['gate'] for item in crossings]==list(range(7))
    solves=result.get('solves',[])
    mppi=[float(row['cycle_ms']) for row in solves]
    togt=[float(row['solve_ms']) for row in result.get('reference_updates',[])
          if row.get('solve_ms') is not None]
    sender=np.asarray(sender_checks,dtype=float)
    sender_ms=sender[:,1] if sender.size else np.array([])
    sender_fallbacks=int(sender[:,-1].sum()) if sender.ndim==2 and sender.shape[1]>=12 else None
    execution=np.asarray(execution_rows) if execution_rows is not None else np.empty((0,))
    average_speed=None
    if execution.ndim==2 and len(execution)>1:
        duration=execution[-1,0]-execution[0,0]
        if duration>0:
            speed=np.linalg.norm(execution[:,4:7],axis=1)
            average_speed=float(np.trapz(speed,execution[:,0])/duration)
    physical=bool(result.get('completed') and ordered and not result.get('collision_detected')
                  and audit.get('min_sampled_body_clearance_m',-1)>=0
                  and audit.get('all_six_workspace_faces_passed')
                  and audit.get('strict_sampled_success'))
    realtime=bool(physical and mppi and max(mppi)<100
                  and all(x<100 for x in togt)
                  and (not len(sender_ms) or max(sender_ms)<100)
                  and audit.get('togt',{}).get('deadline_rejections',0)==0)
    lap=float(result['flight_time_s']) if realtime else None
    out=dict(condition=dict(motion_rate=result.get('motion_rate'),
                            translation_scale=result.get('translation_scale'),
                            rotation_scale=result.get('rotation_scale'),phase=result.get('phase')),
             ordered_seven=ordered,physical_success=physical,realtime_success=realtime,
             race_time_s=lap,completed_gates=len(crossings),failure=result.get('failure'),
             gate_exit_times_s=[float(x['time']) for x in crossings],
             gate_local_points_m=[x.get('local_point') for x in crossings],
             mean_speed_mps=average_speed,
             min_sampled_body_clearance_m=audit.get('min_sampled_body_clearance_m'),
             max_speed_mps=audit.get('max_sampled_body_speed_mps'),
             max_acceleration_mps2=audit.get('max_sampled_body_acceleration_mps2'),
             max_tilt_deg=audit.get('max_truth_tilt_deg'),
             max_mppi_ms=max(mppi,default=None),max_togt_ms=max(togt,default=None),
             max_sender_filter_ms=float(max(sender_ms)) if len(sender_ms) else None,
             sender_fallbacks=sender_fallbacks,
             activation_fallbacks=sum(bool(x.get('activation_filter_fallback')) for x in solves),
             future_risk_warnings=sum(bool(x.get('future_warning')) for x in solves),
             reference_updates=sum(bool(x.get('accepted')) for x in result.get('reference_updates',[])),
             evidence='PX4/Gazebo; 10 ms sampled truth, not continuous-time certification')
    if baseline is not None:
        same=(int(baseline['rate'])==result.get('motion_rate')
              and int(baseline['translation_scale'])==result.get('translation_scale')
              and int(baseline['rotation_scale'])==result.get('rotation_scale')
              and abs(float(baseline['phase'])-float(result.get('phase')))<1e-9)
        if not same:raise ValueError('baseline condition does not match this run')
        old=float(baseline['flight_time_s']) if baseline.get('realtime_success') else None
        out['baseline']=dict(method='checked',strict_success=bool(baseline.get('realtime_success')),
                             race_time_s=old,
                             time_saved_s=old-lap if old is not None and lap is not None else None,
                             time_reduction_percent=100*(old-lap)/old if old and lap is not None else None)
    return out


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('run',type=Path)
    parser.add_argument('--baseline-summary',type=Path)
    args=parser.parse_args()
    result=json.loads((args.run/'result.json').read_text())
    audit=json.loads((args.run/'audit.json').read_text())
    checks=np.load(args.run/'telemetry.npz')['sender_checks']
    execution=np.load(args.run/'execution.npz')['rows']
    baseline=None
    if args.baseline_summary is not None:
        rows=json.loads(args.baseline_summary.read_text())
        matches=[row for row in rows if row.get('realtime_success') and
                 int(row['rate'])==result['motion_rate'] and
                 int(row['translation_scale'])==result['translation_scale'] and
                 int(row['rotation_scale'])==result['rotation_scale'] and
                 abs(float(row['phase'])-float(result['phase']))<1e-9]
        if len(matches)!=1:raise ValueError(f'expected one matching successful baseline, got {len(matches)}')
        baseline=matches[0]
    report=assess(result,audit,checks,baseline,execution)
    (args.run/'RACING_EVALUATION.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    status='PASS' if report['realtime_success'] else 'FAIL'
    lines=['# 动态七窗：竞速验收','',condition_label(report['condition']),'',
           f"结论：{status}；七窗 {report['completed_gates']}/7；有效圈时 " +
           (f"{report['race_time_s']:.3f} s。" if report['race_time_s'] is not None else '无（失败不记圈时）。'),'',
           '排序规则：先要求顺序七窗、返航、零接触、10 ms 真值采样安全和全部在线周期 <100 ms；',
           '只有严格通过的运行才按飞行时间排序。部分过门和失败前飞行时间不参与竞速排名。','',
           f"真值最小整机净空 {report['min_sampled_body_clearance_m']:.3f} m；平均/最高速度 "
           f"{report['mean_speed_mps']:.3f}/{report['max_speed_mps']:.3f} m/s；",
           f"最高采样加速度 {report['max_acceleration_mps2']:.3f} m/s²；最大倾角 {report['max_tilt_deg']:.3f}°。",
           f"最大 MPPI/TOGT/发送端滤波耗时：{report['max_mppi_ms']:.3f}/"
           f"{report['max_togt_ms']:.3f}/{report['max_sender_filter_ms']:.3f} ms。",
           f"发送端候选搜索启用 {report['sender_fallbacks']} 次；远期风险预警 {report['future_risk_warnings']} 周期。",
           '七扇门整机退出时刻（s）：'+', '.join(f'{x:.3f}' for x in report['gate_exit_times_s'])+'。','']
    if baseline is not None:
        b=report['baseline']
        lines.append(f"相同条件 checked 基线有效圈时 {b['race_time_s']:.3f} s；"+
                     (f"本次快 {b['time_saved_s']:.3f} s（{b['time_reduction_percent']:.1f}%）。"
                      if b['time_saved_s'] is not None else '本次没有有效圈时，不能比较快慢。'))
        lines.append('这是每方法各一次固定条件试飞，不是总体成功率或稳健优势估计。')
    lines+=['','门框净空和动力学是有限采样数值证据；离散滤波和经验模型不构成连续时间安全证明。','']
    (args.run/'RACING_EVALUATION.md').write_text('\n'.join(lines))
    print(json.dumps(report,indent=2,ensure_ascii=False))


if __name__=='__main__':main()
