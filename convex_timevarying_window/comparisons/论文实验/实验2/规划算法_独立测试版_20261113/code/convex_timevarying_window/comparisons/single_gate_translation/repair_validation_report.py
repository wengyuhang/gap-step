"""Audit the six execution-repair flights against their retained old records."""
import hashlib
import json
from pathlib import Path
import numpy as np
from .audit import audit


def main():
    root=Path(__file__).resolve().parents[3]
    results=root/'convex_timevarying_window/dynamic_passage_velocity_minco/results'
    output=results/'repair_validation_v2_20261009'
    output.mkdir(exist_ok=True)
    conditions=[(3.5,4.,'frequency'),(4.,4.,'frequency'),(3.,4.5,'amplitude'),
                (3.5,4.5,'joint'),(3.,5.,'amplitude'),(3.,5.5,'amplitude')]
    rows=[]
    for frequency,amplitude,route in conditions:
        suffix=f'f{frequency:g}_a{amplitude:g}'
        new=results/f'repair_v2_{suffix}_20261009_01'
        old=results/'difficulty_sweep_v1_20261009'/f'{route}_ours_{suffix}_p000'
        raw=json.loads((new/'result.json').read_text())
        previous=json.loads((old/'result.json').read_text())
        observed=audit(new)
        (new/'physical_audit.json').write_text(json.dumps(observed,indent=2)+'\n')
        hashes=dict(raw['source_sha256'])
        hashes.update(json.loads((new/'passage_sources.json').read_text()))
        mismatches=[name for name,value in hashes.items()
                    if hashlib.sha256((new/'source_snapshot'/name).read_bytes()).hexdigest()!=value]
        tr=np.genfromtxt(new/'telemetry_live.csv',delimiter=',',names=True)
        crosses=observed['crossings']
        row=dict(frequency_hz=frequency,amplitude_m=amplitude,
            old_completed=previous['completed'],old_time_s=previous['flight_time_s'],
            completed=raw['completed'],passed=observed['passed'],failure=raw['failure'],
            collision_detected=observed['collision_detected'],flight_time_s=raw['flight_time_s'],
            crossing_time_s=crosses[0]['time_s'] if len(crosses)==1 else None,
            tracking_error_max_m=float(np.max(tr['error'])),
            max_planning_cycle_ms=max(s['cycle_ms'] for s in raw['solves']),
            max_mpc_cycle_ms=max(s['cycle_ms'] for s in raw['mpc_cycles']),
            planning_requests=len(raw['solves']),accepted=sum(s['accepted'] for s in raw['solves']),
            goal_recovery_accepted=sum(s['accepted'] and s['gate']==1 for s in raw['solves']),
            measured_goal_recovery_accepted=sum(s['accepted'] and s['gate']==1 and
                s['request_head']['source']=='measured_pvaj' for s in raw['solves']),
            build_s=raw['build_s'],source_snapshot_count=len(hashes),source_mismatches=mismatches,
            same_world_bytes=(new/'world.sdf').read_bytes()==(old/'world.sdf').read_bytes(),
            same_scene_bytes=(new/'scene.json').read_bytes()==(old/'scene.json').read_bytes(),
            old_directory=str(old.relative_to(root)),new_directory=str(new.relative_to(root)))
        rows.append(row)
    (output/'summary.json').write_text(json.dumps(rows,indent=2)+'\n')
    lines=['# 执行修复 v2：六档原异常条件复测','',
        '各条件原初相一次；原世界、窗口大小/运动、碰撞和到终点规则不变。旧记录不覆盖。',
        '时间是完整单窗起点到终点，不是穿窗时刻；不是七窗或返航成绩。',
        '安全结果来自窗框接触与 4 ms 机体中心真值，不是连续域认证或安全概率。','',
        '| 幅度 m | 频率 Hz | 旧完整时间 s | 修复完整时间 s | 实测结果 | 规划最大 ms | MPC 最大 ms |',
        '|---:|---:|---:|---:|---|---:|---:|']
    for r in rows:
        old=f"{r['old_time_s']:.3f}" if r['old_completed'] else '未完成'
        new=f"{r['flight_time_s']:.3f}" if r['passed'] else '未完成'
        verdict='无窗框接触完赛' if r['passed'] else ('碰撞' if r['collision_detected'] else str(r['failure']))
        lines.append(f"| {r['amplitude_m']:g} | {r['frequency_hz']:g} | {old} | {new} | {verdict} | {r['max_planning_cycle_ms']:.2f} | {r['max_mpc_cycle_ms']:.2f} |")
    lines+=['','## 改动与边界','',
        '1. MPC 仍是同权重/预测域/输入箱的跟踪器；将独立推力和角速度响应改为共享四旋翼力/力矩分配预测。不是精确 PX4 PID 仿真。',
        '2. 两段、七个变量及局部安全交换保留；动力学软代价补上阻力修正的实际推力需求，不新增窗前动力学硬准入。',
        '3. 整机出窗后继续监听误差事件；终点恢复是一段满足当前 PVAJ 和原终点 PVAJ 的七次曲线。实际推力采样约束和不回穿的平面 Bernstein 约束仅用于此终点段。',
        '4. 漏过开口明确中止，不能反向重穿同窗凑完赛。此项修复不等于能恢复所有漏窗。',
        '5. 旧版入口默认行为保留；这是多个执行修复的联合测试，不能将全部收益归于其中某一个改动。','',
        '第一版 90% 推力硬约束负结果仍在 repair_v1_f3_a4.5_20261009_01：40 s 一直停留起点，无可接纳规划，不算成功。',
        '所有在线耗时按实际日志报告。飞前编译单列，不计为在线规划；90 ms 软件预算不是硬实时认证。',
        '两组实际初态并非逐位一致，不能据一次闭环试验宣称确定性或成功率。','',
        '证据：各目录 result.json、flight.ulg、gate_*_contacts.pbtxt、physical_audit.json、源码快照；详细字段见 summary.json。']
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(rows,indent=2))


if __name__=='__main__':main()
