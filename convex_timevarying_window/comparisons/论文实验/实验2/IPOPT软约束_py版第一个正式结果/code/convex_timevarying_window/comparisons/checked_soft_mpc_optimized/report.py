"""Optimized MPC frozen-matrix report, historical baselines remain read-only."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np

from convex_timevarying_window.comparisons.checked_soft_racing.report import (
    BASE, SOURCES, key, selected, stats, paired, format_value)
from convex_timevarying_window.gs_mppi.direct_ctbr.compare_targets import body_dynamics


def read(path): return json.loads(Path(path).read_text())


def groups():
    center={(r,t,1,p) for r in (1,3,6,10) for t in (1,2,4) for p in (0,1)}
    amplitude={(r,t,a,p) for r,t,a in [(r,a,a) for r in (1,10) for a in (1,2,4)]+[(10,4,1),(10,1,4)] for p in (0,1)}
    rate={(r,1,1,p) for r in (1,3,6,10) for p in (0,1,2)}
    return dict(center24=center,amplitude16=amplitude,rate12=rate)


def full_statistics(rows):
    basic=stats(rows)
    basic.update(strict_dynamics_success=sum(bool(r.get('strict_dynamics_success')) for r in rows),
                 mpc_max_ms=max((r['mpc_cycle_ms']['max'] for r in rows if r.get('mpc_cycle_ms')),default=None),
                 sender_max_ms=max((r['sender_cycle_ms']['max'] for r in rows if r.get('sender_cycle_ms')),default=None))
    return basic


def reason(row):
    if row.get('infrastructure_failure'):return '准备/基础设施异常'
    if row.get('analysis_error'):return '验收证据不足'
    if row.get('collision'):return '门框接触'
    if row.get('min_clearance_m',0)<0:return '真值采样球净空为负'
    if not row.get('controller_deadline_passed',True) or (row.get('cycle_ms') and row['cycle_ms']['max']>=100):return '计算超100ms'
    if 'QP' in str(row.get('failure')):return 'MPC数值求解中止'
    if row.get('failure')=='timeout':return '60s未完成'
    return str(row.get('failure') or '其他验收不通过')


def main():
    p=argparse.ArgumentParser();p.add_argument('batch',type=Path);p.add_argument('--historical-dynamics',action='store_true')
    a=p.parse_args();batch=a.batch.resolve()
    attempts=read(batch/'summary.json');first=read(batch/'first_attempt_summary.json');current=read(batch/'condition_summary.json')
    historical=[];manifest=[]
    for matrix,name,relative in SOURCES+[('soft_partial','Checked-Soft/PVA','comparisons/checked_soft_racing/results/frozen_20261006/summary.json'), ('mpc_partial','MPC before optimization','comparisons/checked_soft_mpc_racing/results/frozen_20261006/condition_summary.json')]:
        path=BASE/relative
        rows=selected(read(path))
        historical.append(dict(matrix=matrix,method=name,rows=rows))
        manifest.append(dict(matrix=matrix,method=name,path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    comparisons=[]
    for source in historical:
        oldkeys={key(r) for r in source['rows']}
        matching=[r for r in current if key(r) in oldkeys]
        comparisons.append(dict(matrix=source['matrix'],method=source['method'],old=stats(source['rows']),
                                new_on_common_conditions=full_statistics(matching),paired=paired(matching,source['rows'])))
    dynamics=[]
    if a.historical_dynamics:
        cache={}
        for source in historical:
            # Primary center-motion baselines and the direct controller ablation.
            if source['matrix'] not in ('center24','soft_partial'):continue
            for i,r in enumerate(source['rows']):
                run=Path(r['run'])
                if str(run) not in cache:
                    try:cache[str(run)]=body_dynamics(run,read(run/'result.json')['last_time_s'])
                    except Exception as exc:cache[str(run)]=dict(error=repr(exc))
                dynamics.append(dict(method=source['method'],run=str(run),condition=key(r),
                    original_success=bool(r.get('realtime_success')),
                    strict_dynamics_success=bool(r.get('realtime_success') and cache[str(run)].get('satisfies_direct_dynamics_limits')),
                    **cache[str(run)]))
            print('Historical dynamics '+source['method'],flush=True)
        (batch/'historical_dynamics.json').write_text(json.dumps(dynamics,indent=2)+'\n')
    elif (batch/'historical_dynamics.json').exists():dynamics=read(batch/'historical_dynamics.json')
    summary=dict(first_attempts=full_statistics(first),after_permitted_setup_retries=full_statistics(current),
                 attempts=len(attempts),planned_conditions=38,comparisons=comparisons,
                 failures=dict(Counter(reason(r) for r in current if not r.get('realtime_success'))),
                 source_manifest=manifest,historical_dynamics=dynamics)
    (batch/'comparison.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    lines=['# 优化Checked-Soft/MPC：38条件完整复测','',
           f'已完成 {len(current)}/38 条件，共 {len(attempts)} 次尝试；未修改冻结控制器或按条件调参。',
           f'首尝试主验收 {summary["first_attempts"]["realtime_success"]}/{len(first)}；仅允许基础设施重试后 {summary["after_permitted_setup_retries"]["realtime_success"]}/{len(current)}。',
           f'仅作附加诊断：若再加真值12 m/s、8 m/s²、42°要求，通过 {summary["after_permitted_setup_retries"]["strict_dynamics_success"]}/{len(current)}。',
           '主验收=按序七窗返航、无门框接触、10 ms真值球净空/房间通过、所有规划/MPC/发送周期<100 ms；不是连续安全证书。',
           '历史主标签不改写。每条件单次，不估计随机成功概率；原始、r1/t2/a1/120°、r10/t4/a1/120°曾用于开发，不能称完全独立留出。',
           '优化内容：规划独立进程消除共享CPU限时；MPC线性化动力学解析消元（542→100变量），预测模型/时域/目标/约束不变。',
           '遵照用户澄清，轻微名义动力学超限单列诊断，不作为主失败条件；碰撞、净空、任务完成及100 ms限制不放宽。','',
           '## 对照（相同条件才比较）','',
           '| 矩阵/对照 | 旧主验收 | 新MPC对应条件主验收 | 双方合格数 | 新更快数 | 新减旧圈时中位数s |',
           '|---|---:|---:|---:|---:|---:|']
    for c in comparisons:
        old,new,pair=c['old'],c['new_on_common_conditions'],c['paired']
        lines.append(f"| {c['matrix']} / {c['method']} | {old['realtime_success']}/{old['n']} | {new['realtime_success']}/{new['n']} | {pair['common_successes']} | {pair['new_faster']} | {format_value(pair['median_new_minus_old_s'])} |")
    lines+=['','负数表示新MPC快。失败不填圈时；同时看成功数，不能只挑双方成功交集夸大优势。',
            'Checked-Soft/PVA只有33条件，首批含一次准备期物理静止异常、尚无重试；不能将该差异全部归因于控制器。',
            '部分历史算法本身未跑满矩阵，分母按其实际条件。不同矩阵存在重叠，不相加当作独立样本。',
            '旧状态RL/waypoint MPPI未完成同任务PX4迁移，不当作作者官方成功率；离线TOGT名义时间不列安全圈速。',
            '完整历史来源/已有稀疏开发变体见上一轮Checked-Soft对账；本表是冻结矩阵，不用单条最快开发圈代表一个算法。','',
            '## 新MPC分矩阵','',
            '| 矩阵 | 首尝试 | 准备异常重试后 | 再加严格动力学 | 合格圈时中位数s |',
            '|---|---:|---:|---:|---:|']
    for name,ids in groups().items():
        raw=[r for r in first if tuple(r['condition_id']) in ids];sub=[r for r in current if tuple(r['condition_id']) in ids]
        s=full_statistics(sub)
        lines.append(f"| {name} | {sum(bool(r.get('realtime_success')) for r in raw)}/{len(raw)} | {s['realtime_success']}/{s['n']} | {s['strict_dynamics_success']}/{s['n']} | {format_value(s['median_success_time_s'])} |")
    lines+=['','## 全部条件','',
            '| r/t/a/相位编号 | 尝试 | 主验收 | 严格动力学 | 有效圈时s | 采样净空m | 最大跟踪误差m | 规划/MPC最慢ms | 失败 |',
            '|---|---:|---|---|---:|---:|---:|---|---|']
    for r in current:
        plan=r.get('cycle_ms') or {};mpc=r.get('mpc_cycle_ms') or {}
        lap=r.get('flight_time_s') if r.get('realtime_success') else None
        lines.append(f"| {'/'.join(map(str,r['condition_id']))} | {r['attempt']} | {r.get('realtime_success')} | {r.get('strict_dynamics_success',False)} | {format_value(lap)} | {format_value(r.get('min_clearance_m'),4)} | {format_value(r.get('tracking_error_before_contact_max_m'))} | {format_value(plan.get('max'),2)}/{format_value(mpc.get('max'),2)} | {'' if r.get('realtime_success') else reason(r)} |")
    lines+=['','相位编号0/1/2分别为0°/120°/240°。误差统计在首次接触前截断；无接触时使用全程。','',
            '## 计算与动力学','']
    for r in current:
        if not r.get('realtime_success'):continue
        d=r['dynamics'];m=r['mpc_cycle_ms']
        lines.append(f"- {r['condition_id']}：峰速/峰加速度/倾角={d['max_speed_mps']:.3f}/{d['max_acceleration_mps2']:.3f}/{d['max_tilt_deg']:.3f}；MPC均值/p99/最大={m['mean']:.2f}/{m['p99']:.2f}/{m['max']:.2f} ms，实际更新{r['actual_racing_mpc_hz']:.2f} Hz，{m['over_20ms']}/{m['count']}次≥20 ms。")
    if dynamics:
        lines+=['','共同真值动力学复核（保留旧主标签）：']
        for name in sorted({r['method'] for r in dynamics}):
            sub=[r for r in dynamics if r['method']==name]
            lines.append(f"- {name}：原主验收且严格动力学通过 {sum(r['strict_dynamics_success'] for r in sub)}/{len(sub)}；缺证据{sum('error' in r for r in sub)}。")
    lines+=['','## 保留的基础设施尝试','']
    for r in attempts:
        if r['infrastructure_failure']:lines.append(f"- {r['condition_id']} attempt{r['attempt']}：{r['failure']}；ULog判定见summary.json中的preparation_assessment；原目录{r['run']}。")
    lines+=['','源码、环境、冻结协议见protocol.json/source_snapshot；原始尝试summary.json，首尝试first_attempt_summary.json，',
            '按允许重试选择的条件condition_summary.json。无碰撞但未完成、负净空、计算超限均失败。','',
            '## MPC的职责和本次能说明什么','',
            '规划器决定按什么时间经过哪些穿越点；本MPC只根据参考PVA和实测全状态输出推力/角速度，不读取窗口。',
            '原PVA方案把位置/速度/姿态跟踪留给PX4；本版替换了外层跟踪控制并保留PX4角速度内环。',
            '因此模型/时间对齐、PX4控制交接、数值迭代和闭环稳定性都是实现责任，不是新避障规划。',
            '本冻结版还有明确的约束对齐缺口：规划名义42°，MPC预测倾角上限却为45°（沿用PX4限幅）。',
            '实际43°左右不一定违反MPC自己的45°约束，但违反整套系统要求的42°名义界；这是接口参数未统一，',
            '不能笼统归因于MPC原理。按用户要求，本批保留实测峰值但不因轻微名义超限否决主验收。',
            '此前日志接口错误属于实现错误；准备期物理静止属于前提未成立；QP中止/姿态发散是跟踪执行失败，',
            '不能据此证明赛道不可行。名义0.35 m余量不是已证明误差界，跟踪MPC也不是安全过滤器。',
            '闭环重规划轨迹会随过门时刻变化，故本批是更换跟踪器的系统实验，不是相同参考序列上的严格单因素控制器辨识。','']
    lines += ['## 图','',
              '![中心运动24条件](center24_comparison.png)','',
              '![两个预声明条件的误差与速度](fixed_case_tracking.png)','',
              'PVA碰撞条件的曲线在首次接触处截断，避免碰撞后的失控数据掩盖碰撞前跟踪误差；',
              '该趟仍按失败计入上表，不因图中截断变成成功。',
              '开发过程（隔离v1的1次QP中止、消元v2的3次通过）分别保存在',
              '`checked_soft_mpc_isolated/results/px4_v1_20261006`与`px4_v2_condensed_20261006`，不混入正式38条件。','']
    (batch/'REPORT.md').write_text('\n'.join(lines))
    plot(batch,current,historical)
    print(json.dumps({k:v for k,v in summary.items() if k not in ('comparisons','source_manifest','historical_dynamics')},indent=2,ensure_ascii=False))


def plot(batch,current,historical):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    ids=groups()['center24'];new=[r for r in current if tuple(r['condition_id']) in ids]
    data=[('MPC optimized',new)]
    for wanted in ('Checked-Soft/PVA','Checked','Wang 2023 independent','Direct-CTBR MPPI'):
        source=next(s for s in historical if s['method']==wanted and s['matrix'] in ('center24','soft_partial'))
        keys={key(r) for r in new};data.append((wanted,[r for r in source['rows'] if key(r) in keys]))
    fig,axes=plt.subplots(1,3,figsize=(13,4.5),constrained_layout=True)
    for i,(name,rows) in enumerate(data):
        good=[r for r in rows if r.get('realtime_success')]
        axes[0].bar(i,len(good));axes[0].text(i,len(good)+.3,f'{len(good)}/{len(rows)}',ha='center')
        axes[1].scatter(np.full(len(good),i),[r['flight_time_s'] for r in good],s=22)
        axes[2].scatter(np.full(len(good),i),[r['min_clearance_m'] for r in good],s=22)
    for ax,title in zip(axes,['Primary accepted runs','Accepted lap time [s]','Minimum sampled clearance [m]']):
        ax.set_title(title);ax.set_xticks(range(len(data)),[d[0] for d in data],rotation=30,ha='right',fontsize=8);ax.grid(axis='y',alpha=.2)
    axes[0].set_ylim(0,27);fig.savefig(batch/'center24_comparison.png',dpi=160);plt.close(fig)
    # Two predefined conditions, independent of observed outcome.
    fig,axes=plt.subplots(2,2,figsize=(11,6),constrained_layout=True)
    for col,target in enumerate(((1.,1.,1.,0.,0.),(10.,4.,1.,round(2*np.pi/3,9),0.))):
        for name,rows in data[:2]:
            match=[r for r in rows if key(r)==target]
            if not match:continue
            path=Path(match[0]['run']);file=path/'telemetry.csv'
            if not file.exists():continue
            d=np.genfromtxt(file,delimiter=',',names=True)
            raw=read(path/'result.json')
            contacts=[c['first_contact_sim_time_s']-30. for c in raw.get('contacts',[])
                      if c.get('first_contact_sim_time_s') is not None]
            if contacts:
                cutoff=min(contacts);d=d[d['t']<=cutoff]
                axes[0,col].axvline(cutoff,color='tab:red',ls=':',lw=1.)
                axes[0,col].annotate('PVA contact; trace cut here',(cutoff,0),
                                    xytext=(-4,8),textcoords='offset points',ha='right',fontsize=7)
            axes[0,col].plot(d['t'],d['error'],label=name)
            axes[1,col].plot(d['t'],np.sqrt(d['vx']**2+d['vy']**2+d['vz']**2))
        axes[0,col].set_title('r1/t1/a1/0 deg' if col==0 else 'r10/t4/a1/120 deg')
        axes[0,col].axhline(.35,color='k',ls='--',lw=.8);axes[0,col].set_ylabel('Tracking error [m]');axes[0,col].legend()
        axes[1,col].set_ylabel('Estimated speed [m/s]');axes[1,col].set_xlabel('Race time [s]')
    for ax in axes.flat:ax.grid(alpha=.2)
    fig.savefig(batch/'fixed_case_tracking.png',dpi=160);plt.close(fig)


if __name__=='__main__':main()
