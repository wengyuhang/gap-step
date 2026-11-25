"""Truth/contact audit and OFFLINE dense nominal diagnostics, not admission."""
import hashlib
import json
from pathlib import Path
import numpy as np
from .audit import audit
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import flat_actuators,KF


def main():
    root=Path(__file__).resolve().parents[3]
    results=root/'convex_timevarying_window/dynamic_passage_velocity_minco/results'
    output=results/'hard_validation_v2_20261009';output.mkdir(exist_ok=True)
    rows=[]
    for version,frequency,amplitude in [('v1',4.,4.),('v2',4.,4.),('v2',3.5,4.5)]:
        path=results/f'hard_{version}_f{frequency:g}_a{amplitude:g}_20261009_01'
        raw=json.loads((path/'result.json').read_text())
        observed=audit(path)
        (path/'physical_audit.json').write_text(json.dumps(observed,indent=2)+'\n')
        hashes=dict(raw['source_sha256']);hashes.update(json.loads((path/'passage_sources.json').read_text()))
        mismatches=[name for name,h in hashes.items()
            if hashlib.sha256((path/'source_snapshot'/name).read_bytes()).hexdigest()!=h]
        old=results/f'repair_v2_f{frequency:g}_a{amplitude:g}_20261009_01'
        accepted=[s for s in raw['solves'] if s['accepted']]
        dense=[]
        for record in accepted:
            plan=Plan(record['made_at'],record['gate'],np.array(record['durations']),
                np.array(record['coefficients']),np.array(record['variables']),0.,0.,record['status'],
                1 if record['gate']==0 else 0)
            grid=np.linspace(plan.made_at,plan.end_time,int(np.ceil((plan.end_time-plan.made_at)/.001))+1)
            derivatives=[plan.sample_many(grid,d).T for d in (1,2,3,4)]
            F,w,rotor=flat_actuators.map(len(grid))(*derivatives)
            F,w,rotor=map(np.array,(F,w,rotor))
            dense.append(dict(gate=record['gate'],accepted_at_s=record['time'],
                max_step_s=float(np.max(np.diff(grid))),peak_collective_n=float(F.max()),
                peak_rate_abs_rps=np.max(abs(w),axis=1).tolist(),
                min_rotor_n=float(rotor.min()),max_rotor_n=float(rotor.max()),
                rotor_limit_n=KF*1000**2,
                max_rotor_relative_overshoot=max(0.,float(rotor.max()/(KF*1000**2)-1.))))
        row=dict(version=version,frequency_hz=frequency,amplitude_m=amplitude,
            passed=observed['passed'],failure=raw['failure'],collision_detected=raw['collision_detected'],
            full_flight_time_s=raw['flight_time_s'],crossings=observed['crossings'],
            max_tracking_error_m=observed['tracking_error_max_m'],
            first_plan_accepted_s=accepted[0]['time'] if accepted else None,
            planning_requests=len(raw['solves']),accepted_plans=len(accepted),
            max_planning_ms=max(s['cycle_ms'] for s in raw['solves']),
            max_mpc_ms=max(s['cycle_ms'] for s in raw['mpc_cycles']),build_s=raw['build_s'],
            same_world_bytes=(path/'world.sdf').read_bytes()==(old/'world.sdf').read_bytes(),
            source_snapshot_count=len(hashes),source_mismatches=mismatches,
            dense_nominal_diagnostic=dense,formal_certificate=False,
            directory=str(path.relative_to(root)))
        rows.append(row)
    probes=[]
    for name in ['hard_v1_probe_f4_a4','hard_v1_probe_f3.5_a4.5','hard_v2_probe_f4_a4','hard_v2_probe_f3.5_a4.5']:
        raw=json.loads((results/(name+'_20261009_01')/'result.json').read_text())
        probes.append({k:raw.get(k) for k in ['head_result','found','solve_ms','error','build_s','nominal_total_time_s']})
    (output/'summary.json').write_text(json.dumps(dict(flights=rows,probes=probes),indent=2)+'\n')
    lines=['# 单窗名义动力学硬约束验证','',
        '同原小窗、原初相、同世界/目标/碰撞判据；各版本各条件一次，负结果保留。',
        '完整时间包括起点等待、穿窗及到终点，不是只统计移动时间。不是七窗成绩。','',
        '| 版本 | 幅度 m | 频率 Hz | 实测结果 | 完整时间 s | 首条接受 s | 规划最大 ms | MPC最大 ms |',
        '|---|---:|---:|---|---:|---:|---:|---:|']
    for r in rows:
        verdict='无窗框接触完赛' if r['passed'] else ('碰撞' if r['collision_detected'] else str(r['failure']))
        flight=f"{r['full_flight_time_s']:.3f}" if r['passed'] else '—'
        first=f"{r['first_plan_accepted_s']:.3f}" if r['first_plan_accepted_s'] is not None else '—'
        lines.append(f"| {r['version']} | {r['amplitude_m']:g} | {r['frequency_hz']:g} | {verdict} | {flight} | {first} | {r['max_planning_ms']:.2f} | {r['max_mpc_ms']:.2f} |")
    lines+=['','## 算法','',
        '目标仅T1+T2。仍为两段七次、起点/窗口内点/终点三个几何节点，内部P/V自由，C5。',
        '阻力修正平坦映射得到总推力、姿态、角速度/角加速度，按x500惯量与混控矩阵计算每个电机推力。',
        '每段17个优化节点硬约束：每电机KF·150²至KF·1000² N；角速度绝对值[3.8,3.8,3.49]rad/s；原工作空间位置界。不是名义速度/加速度/jerk软阈值硬化。',
        '取消窗后速度始终非负的额外形状限制，改为位置Bernstein半空间：可超过终点刹车返回，但不能重新跨回窗口平面。原局部浮点安全交换继续使用。',
        'v2仅改通用数值初始化：穿越前后0~80ms的初始安全见证间隔5ms，余下自适应补点；种子按约束残差/时间排序。不是整轨迹后检查或场景专用选点。',
        'MPC仍用上轮相同的电机分配耦合跟踪器，没有新避障/倾角代价。末窗后继续事件恢复。','',
        '## 边界与负结果','',
        '硬约束只在优化节点强制；1ms离线名义诊断另列summary中的dense_nominal_diagnostic。存在节点间超限时如实记录，不宣称连续域动力学认证。',
        'v2的4Hz/4m首条轨迹节点间单电机峰值超限5.095%，终点恢复段4.934%；3.5Hz/4.5m分别0.607%和0.032%。实际两趟仍无窗框接触完赛，但不能写成整轨迹严格满足硬界。',
        '该平坦映射是名义近似，不含完整PX4 PID、实际姿态/电机初态误差和全部电机响应约束；闭环是否成功必须看实体接触/真值审计。',
        'v1的21.004s包含14.728s等待首个可接纳规划；v1第二组90ms固定头测试失败，不代表物理无解。',
        '飞前编译单列；100ms是实测预算，不是硬实时证明。此次同时改变可行集/目标/初始化，不能称仅改一项的因果消融。']
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(dict(flights=rows,probes=probes),indent=2))


if __name__=='__main__':main()
