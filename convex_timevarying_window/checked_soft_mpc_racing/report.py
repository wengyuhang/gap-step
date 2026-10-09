"""Same-condition, read-only historical comparison and new-run audit."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from convex_timevarying_window.comparisons.checked_soft_racing.run import analyze_run
from convex_timevarying_window.gs_mppi.direct_ctbr.compare_targets import body_dynamics
from convex_timevarying_window.comparisons.checked_soft_racing.adjudicate import assess

BASE = Path('convex_timevarying_window/comparisons/checked_soft_racing/results/frozen_20261006')
OLD = BASE/'r10_t4_a1_p1_attempt1'


def read(path): return json.loads(path.read_text())


def statistics(values):
    a = np.asarray(values, float)
    return None if not len(a) else dict(mean=float(a.mean()), p99=float(np.percentile(a, 99)), max=float(a.max()))


def row(path, existing=None):
    r = read(path/'result.json')
    audit = analyze_run(path) if existing is None else dict(existing)
    contacts = [c['first_contact_sim_time_s']-30 for c in r['contacts'] if c['first_contact_sim_time_s'] is not None]
    cutoff = min(contacts) if contacts else r['last_time_s']
    data = np.genfromtxt(path/'telemetry.csv', delimiter=',', names=True)
    before = data[data['t'] < cutoff] if contacts else data
    error = before['error']
    mpc = r.get('mpc_cycles', [])
    cycles = [c['cycle_ms'] for c in mpc]
    controller_ok = bool(cycles) and max(cycles) < 100 and not any('failure' in c for c in mpc)
    if r['method'] == 'checked_soft_mpc_racing':
        audit['realtime_success'] &= controller_ok
    deltas = np.diff([c['time'] for c in mpc])
    result = dict(method=r['method'], run=str(path.resolve()), verdict=audit,
        first_contact_race_s=min(contacts) if contacts else None,
        tracking_before_first_contact_m=statistics(error), tracking_interval_end_s=cutoff,
        tracking_rms_m=float(np.sqrt(np.mean(error**2))),
        mpc_cycle_ms=statistics(cycles), mpc_over_20ms=int(np.sum(np.asarray(cycles) > 20)),
        mpc_count=len(cycles), actual_mpc_update_hz=float(1/np.mean(deltas)) if len(deltas) else None,
        inexact_qp_steps=sum(c.get('inexact_qp_steps',0) for c in mpc),
        preparation_audit=assess(path),
        full_state_sender_cycle_ms=statistics(np.load(path/'execution.npz')['rows'][:, -1]) if (path/'execution.npz').exists() else None,
        dynamics_before_contact=body_dynamics(path, cutoff),
        result_sha256=hashlib.sha256((path/'result.json').read_bytes()).hexdigest())
    return result


def main():
    p = argparse.ArgumentParser(); p.add_argument('runs', type=Path, nargs='+'); p.add_argument('--outdir', type=Path, required=True)
    a = p.parse_args(); a.outdir.mkdir(parents=True, exist_ok=False)
    raw = next(r for r in read(BASE/'summary.json') if r['condition_id'] == [10,4,1,1])
    rows = [row(OLD, raw)]+[row(path) for path in a.runs]
    baseline = read(OLD/'result.json')
    settings = []
    for path in a.runs:
        r = read(path/'result.json')
        settings.append(dict(run=str(path), world_identical=(path/'world.sdf').read_bytes() == (OLD/'world.sdf').read_bytes(),
                             planner_config_identical=r['config'] == baseline['config'],
                             px4_parameters_identical=r['parameters'] == baseline['parameters']))
    (a.outdir/'comparison.json').write_text(json.dumps(dict(rows=rows, settings=settings), indent=2)+'\n')
    lines = ['# 同一碰撞条件：改为MPC跟踪', '',
        '10倍频率、4倍平移、1倍旋转、120°相位，x500/PX4；相同规划/世界/硬余量/60 s上限。',
        '新控制器直接输出CTBR，旧版输出PVA；旧位置环的50 ms预览由MPC姿态/电机预测代替。',
        '这是开发诊断，不是固定参数完整矩阵，也不是总体成功率或连续安全证明。', '',
        '| 运行 | 按序过窗 | 接触 | 主实时验收 | 有效圈时s | 采样净空m | 碰撞前/全程最大跟踪误差m | 规划最大ms | MPC最大ms |',
        '|---|---|---|---|---:|---:|---:|---:|---:|']
    for r in rows:
        v=r['verdict']; name=Path(r['run']).name; ok=v['realtime_success']
        lap=f"{v['flight_time_s']:.3f}" if ok else '—'
        ms=f"{r['mpc_cycle_ms']['max']:.2f}" if r['mpc_cycle_ms'] else '—'
        lines.append(f"| {name} | {v['gates']}/7 | {v['collision']} | {ok} | {lap} | {v['min_clearance_m']:.4f} | {r['tracking_before_first_contact_m']['max']:.3f} | {v['cycle_ms']['max']:.2f} | {ms} |")
    lines += ['', '主验收沿用旧口径：按序七窗返航、无接触、真值10 ms球—门套净空非负、房间界通过，',
              '加上全部规划/MPC调用<100 ms。真实动力学界另列，不把规划采样界冒充实际界。', '',
              '## 实际动力学与计算', '']
    for r in rows:
        d=r['dynamics_before_contact']; lines.append(f"- {Path(r['run']).name}：碰撞前或全程真值峰速 {d['max_speed_mps']:.3f} m/s、峰加速度 {d['max_acceleration_mps2']:.3f} m/s²、最大倾角 {d['max_tilt_deg']:.3f}°；满足12/8/42诊断界：{d['satisfies_direct_dynamics_limits']}。")
        if r['mpc_cycle_ms']:
            lines.append(f"  MPC计算均值 {r['mpc_cycle_ms']['mean']:.2f} ms，p99 {r['mpc_cycle_ms']['p99']:.2f} ms；{r['mpc_over_20ms']}/{r['mpc_count']} 次超过20 ms；实际平均更新 {r['actual_mpc_update_hz']:.2f} Hz，不能称严格50 Hz。")
        if r['verdict']['failure']: lines.append(f"  原始失败：`{r['verdict']['failure']}`，不计有效圈时。")
    history=[]
    summaries=[('Checked (2026-09-23)',Path('convex_timevarying_window/comparisons/amplitude_racing/results/center_motion_checked_20260923/summary.json')),
               ('Wang independent (2026-09-23)',Path('convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/summary.json')),
               ('CTBR MPPI matrix (2026-09-24)',Path('convex_timevarying_window/comparisons/direct_ctbr_center_motion/results/direct_ctbr_20260924/summary.json'))]
    for name,path in summaries:
        for r in read(path):
            if (r.get('rate',r.get('motion_rate'))==10 and r['translation_scale']==4 and r['rotation_scale']==1
                and abs(r['phase']-2*np.pi/3)<1e-8 and not r.get('infrastructure_failure')):
                history.append(dict(method=name,passed=r['realtime_success'],time=r['flight_time_s'] if r['realtime_success'] else None,
                                    clearance=r['min_clearance_m'],run=r['run'],source=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    for suffix in ('v7_20260926','v7_repeat_20260926'):
        path=Path('convex_timevarying_window/gs_mppi/direct_ctbr/results')/f'flow_smooth_px4_gui_r10_t4_p120_{suffix}'/'RACING_EVALUATION.json'
        r=read(path);history.append(dict(method='CTBR MPPI development '+suffix,passed=r['realtime_success'],time=r['race_time_s'],
                                        clearance=r['min_sampled_body_clearance_m'],run=str(path.parent),source=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    (a.outdir/'historical_context.json').write_text(json.dumps(history,indent=2)+'\n')
    lines+=['','## 同条件其他历史版本（不是同控制器消融）','',
            '| 方法 | 历史主验收 | 有效圈时s | 采样净空m |','|---|---|---:|---:|']
    for r in history:
        lap='—' if r['time'] is None else f"{r['time']:.3f}"
        lines.append(f"| {r['method']} | {r['passed']} | {lap} | {r['clearance']:.4f} |")
    lines+=['','因此不能宣称MPC版比所有已有方法都快：历史CTBR v7的两趟合格圈更快。',
            'Wang虽记录32.760 s返航，但真值球净空为负，按既有协议不得当成安全有效圈时；',
            '不能只看无实际接触就把该条改为通过。历史来源/哈希单列historical_context.json。']
    lines += ['', 'v1日志误读了无brake属性的拒绝候选而中止；v2修复记录接口、保持相同控制参数，但出现姿态发散/QP失败。',
              'v3增加通用SQP信赖域、非线性代价回溯及状态/控制缩放和严格QP容差，不修改规划目标、硬余量或赛道。',
              'v4把PX4控制模式交接放在原准备期；v5增加QP迭代预算、逐次仍限25 ms；v6显式校验原始残差后允许有界未完全收敛迭代参与非线性回溯。',
              'v6首尝试在算法运行前未起飞：真值t=10..29 s位移/速度为零，已解锁、有爬升指令，距规定起点8.973 m；保留原尝试并仅进行一次同代码基础设施重试。',
              'CTBR接口强制MAVLink 2获取电机状态；飞行增益/限幅参数相同，但不能称所有通信参数/控制模式也相同。',
              '旧PVA误差统计截止第一次接触，排除碰撞后失控；新无接触趟使用全程。二者在线重规划的具体参考轨迹并不完全相同，',
              '这能检验更换控制器后的闭环表现，但不是在完全相同参考序列上的控制器因果消融。', '',
              '配置、世界、结果哈希、全部数值见 comparison.json。']
    (a.outdir/'REPORT.md').write_text('\n'.join(lines)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes=plt.subplots(2,1,figsize=(10,6),sharex=True,constrained_layout=True)
    # Show old contact failure and each new attempt; do not select best result.
    for item in rows:
        data=np.genfromtxt(Path(item['run'])/'telemetry.csv',delimiter=',',names=True)
        keep=data['t'] <= item['tracking_interval_end_s']
        t=data['t'][keep]; label='PVA (collision)' if not item['mpc_count'] else Path(item['run']).name.replace('_20261006','')
        axes[0].plot(t,data['error'][keep],label=label,linewidth=1.)
        axes[1].plot(t,np.sqrt(data['vx'][keep]**2+data['vy'][keep]**2+data['vz'][keep]**2),linewidth=1.)
    axes[0].axhline(.35,color='black',linestyle='--',label='Nominal tracking margin (not proven bound)')
    axes[0].set_ylim(0, 1.)
    axes[0].set_title('Same hard case; errors >1 m clipped (full failure metrics in report)',fontsize=10)
    axes[0].set_ylabel('Tracking error [m]');axes[1].set_ylabel('Estimated speed [m/s]');axes[1].set_xlabel('Race time [s]')
    axes[0].legend(fontsize=8)
    for ax in axes: ax.grid(alpha=.2)
    fig.savefig(a.outdir/'tracking_comparison.png',dpi=160);plt.close(fig)
    print(json.dumps([dict(run=r['run'],verdict=r['verdict'],mpc=r['mpc_cycle_ms']) for r in rows],indent=2))


if __name__ == '__main__': main()
