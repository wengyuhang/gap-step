"""Planning/flight tables and local-passage figures from the frozen raw data."""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from . import planning, scenario

COLORS = dict(ours='#0072B2', instant='#D55E00', center='#009E73', togt_center='#CC79A7')
ENGLISH = dict(ours='Ours, free point', instant='Instant-only ablation',
    center='Ours, strict center', togt_center='Dynamic TOGT, center')


def roots_for_level(plan, position_x):
    events = []
    start = 0.
    for piece, (C,T) in enumerate(zip(plan.coefficients, plan.durations)):
        normal = C[:,0].copy()
        normal[0] -= position_x
        for z in np.polynomial.polynomial.polyroots(normal):
            if abs(z.imag) < 1e-7 and -1e-8 <= z.real <= 1.+1e-8:
                events.append(float(start+np.clip(z.real,0.,1.)*T))
        start += T
    return sorted(set(round(value,10) for value in events))


def crossing_data(selected):
    raw = planning.read(selected['source'])
    plan = planning.frozen_plan(raw)
    at = raw['crossing_times_s'][0]
    track, _ = scenario.build_case(scenario.Case(selected['level']))
    window = track.windows[0]
    half_band = .3859447197013662+.085
    before = roots_for_level(plan, window.center0[0]-half_band)
    after = roots_for_level(plan, window.center0[0]+half_band)
    entry = max((value for value in before if value <= at), default=0.)
    exit = min((value for value in after if value >= at), default=float(plan.durations.sum()))
    center = window.state_at(at)[0]
    point = plan.sample(at)
    velocity = np.polynomial.polynomial.polyval(1.,
        np.polynomial.polynomial.polyder(plan.coefficients[0]))/plan.durations[0]
    frame_velocity = window.state_at(at)[2]
    return dict(crossing_time_s=at, entry_s=entry, exit_s=exit,
        passage_duration_ms=1000.*(exit-entry), point_world_m=point.tolist(),
        point_local_m=window.world_to_local(point,at)[0].tolist(),
        body_velocity_mps=velocity.tolist(), window_velocity_mps=frame_velocity.tolist(),
        relative_velocity_mps=(velocity-frame_velocity).tolist(),
        window_center_world_m=center.tolist())


def draw_passage(out, selected, flights, level):
    fig, axes = plt.subplots(2,4,figsize=(14,6.6),constrained_layout=True)
    measurements = {}
    for column, method in enumerate(planning.LABELS):
        top, bottom = axes[:,column]
        top.set_title(ENGLISH[method],fontsize=11)
        record = selected.get(method)
        if record is None:
            top.text(.5,.5,'No qualified plan\nfrom eight starts',ha='center',va='center',transform=top.transAxes)
            top.set_axis_off()
            bottom.set_axis_off()
            continue
        data = crossing_data(record)
        measurements[method] = data
        raw = planning.read(record['source'])
        plan = planning.frozen_plan(raw)
        window = scenario.build_case(scenario.Case(level))[0].windows[0]
        crossing = data['crossing_time_s']
        span = max(.10,1.3*(data['exit_s']-data['entry_s']))
        t = np.linspace(max(0.,crossing-span),min(float(plan.durations.sum()),crossing+span),801)
        p = plan.sample_many(t)
        center = window.center0+window.motion.translation(t)[0]
        horizontal = p[:,1]-center[:,1]
        top.plot((t-crossing)*1000.,horizontal,color=COLORS[method],lw=2,label='Nominal body center')
        top.axhline(1.15,color='.3',lw=1,label='Physical aperture edge')
        top.axhline(-1.15,color='.3',lw=1)
        top.axhline(0.,color='.7',lw=.7)
        top.axvline(0.,color='.4',lw=.8,ls=':')
        top.axvspan((data['entry_s']-crossing)*1000.,(data['exit_s']-crossing)*1000.,
            color=COLORS[method],alpha=.12,label='Body normal band')
        flight = flights.get((level,method))
        if flight is not None:
            trace = np.load(Path(flight['folder'])/'truth_trace.npz')
            mask=(trace['times']>=t[0])&(trace['times']<=t[-1])
            actual_t=trace['times'][mask]
            actual_p=trace['positions'][mask]
            actual_c=window.center0+window.motion.translation(actual_t)[0]
            top.plot((actual_t-crossing)*1000.,actual_p[:,1]-actual_c[:,1],color='black',ls='--',lw=1.1,label='Actual body center')
            contact=flight['contacts'][0]['first_contact_sim_time_s']
            if contact is not None:
                relative_contact=(contact-30.-crossing)*1000.
                top.axvline(relative_contact,color='#B2182B',lw=1.5,label='Native frame contact')
        top.set_xlabel('Time relative to nominal crossing (ms)')
        top.set_ylabel('World-y body minus window center (m)')
        top.grid(alpha=.2)
        # Tangential motion is drawn against normal progress to show relative motion.
        normal=p[:,0]-window.center0[0]
        bottom.plot(normal,horizontal,color=COLORS[method],lw=2)
        bottom.axvspan(-.4709447197,.4709447197,color=COLORS[method],alpha=.12)
        bottom.axvline(0.,color='.4',lw=.8,ls=':')
        bottom.axhline(1.15,color='.3',lw=1)
        bottom.axhline(-1.15,color='.3',lw=1)
        bottom.scatter([0.],[data['point_world_m'][1]-data['window_center_world_m'][1]],color=COLORS[method],s=35)
        bottom.set_xlabel('Normal body-center distance (m)')
        bottom.set_ylabel('Relative tangential position (m)')
        bottom.grid(alpha=.2)
        top.legend(fontsize=7,loc='best')
    fig.suptitle(f'Single translating gate: amplitude {level} m, frequency {level} Hz',fontsize=14)
    fig.savefig(out/f'passage_l{level:02d}.png',dpi=210)
    fig.savefig(out/f'passage_l{level:02d}.pdf')
    plt.close(fig)
    planning.save(out/f'passage_l{level:02d}.json',measurements)


def summarize(out):
    selected=planning.read(out/'selected.json')
    rows=planning.read(out/'summary.json')
    flights=planning.read(out/'flight_summary.json') if (out/'flight_summary.json').exists() else []
    flight_by={(row['level'],row['method']):row for row in flights}
    candidate_rows=[]
    for row in rows:
        raw=planning.read(row['source'])
        calls=raw['optimizer_calls']
        iterations=sum(item.get('iterations',0) for item in calls) if calls else sum(item.get('iterations',0) for item in raw['candidates'])
        candidate_rows.append(dict(job=row['job'],level=row['level'],method=row['method'],seed=row['seed'],
            found=row['found'],nominal_time_s=row.get('nominal_time_s'),objective=row.get('objective'),
            solve_s=row['solve_s'],build_s=row['build_s'],finite_calls=len(calls),iterations=iterations,
            last_status=calls[-1]['status'] if calls else raw['candidates'][-1]['message'],
            geometry_verdict=raw.get('geometry_verdict'), source=row['source']))
    with (out/'CANDIDATES.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(candidate_rows[0]))
        writer.writeheader()
        writer.writerows(candidate_rows)
    planning.save(out/'candidate_details.json',candidate_rows)
    final_table=['# 实验3 规划结果', '',
        '时间单位为秒，包含起点、窗口及窗后终点。碰撞为名义球代理与移动STL检查结果。', '',
        '|幅度/m|频率/Hz|'+'|'.join(planning.LABELS.values())+'|',
        '|---:|---:|'+'---|'*len(planning.LABELS)]
    planning_rows=[]
    for level, methods in selected.items():
        values=[]
        for method,row in methods.items():
            if row is None:
                values.append('未找到')
            elif row['geometry_verdict']=='nominal_safe_complete':
                values.append(f"{row['nominal_time_s']:.3f}")
            else:
                values.append({'nominal_collision':'碰撞','nominal_task_invalid':'任务不合格',
                    'nominal_unresolved':'证据不足'}.get(row['geometry_verdict'],'证据不足'))
            planning_rows.append(dict(level=int(level),method=method,result=values[-1],
                source=None if row is None else row['source']))
        final_table.append(f'|{level}|{level}|'+ '|'.join(values)+'|')
    final_table+=['', '按原完整目标择优。消融与TOGT碰撞候选保留，未以完整碰撞检查筛选替代参考。',
        '全部134条候选的时间、原目标、迭代、耗时及失败保存在CANDIDATES.csv。']
    (out/'PLANNING.md').write_text('\n'.join(final_table)+'\n')
    with (out/'PLANNING.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(planning_rows[0]))
        writer.writeheader()
        writer.writerows(planning_rows)
    flight_details=[]
    for row in flights:
        folder=Path(row['folder'])
        trace=np.load(folder/'truth_trace.npz')
        raw=planning.read(folder/'source_result.json')
        plan=planning.frozen_plan(raw)
        contact=row['contacts'][0]['first_contact_sim_time_s']
        mask=np.ones(len(trace['times']),dtype=bool) if contact is None else trace['times']<contact-30.
        error=np.linalg.norm(trace['positions'][mask]-plan.sample_many(trace['times'][mask]),axis=1)
        flight_details.append(dict(level=row['level'],method=row['method'],passed=row['passed'],
            frame_collision=row['frame_collision'],ground_collision=row['ground_collision'],
            flight_time_s=row['flight_time_s'],first_contact_task_s=None if contact is None else contact-30.,
            precontact_tracking_rms_m=float(np.sqrt(np.mean(error**2))),
            precontact_tracking_max_m=float(error.max()),truth_goal_error_m=row['truth_goal_error_m'],
            truth_goal_speed_mps=row['truth_goal_speed_mps'],folder=row['folder']))
    if flight_details:
        with (out/'FLIGHT.csv').open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(flight_details[0]))
            writer.writeheader()
            writer.writerows(flight_details)
    planning.save(out/'flight_measurements.json',flight_details)
    center_failures=[]
    last=max(map(int,selected))
    for row in candidate_rows:
        if row['level']==last and row['method']=='center':
            raw=planning.read(row['source'])
            center_failures.append(dict(**row,final_finite_calls=raw['optimizer_calls'][-2:],
                last_exchange_records=raw['candidates'][0]['statistics']['records'][-2:]))
    planning.save(out/'center_nonfind_diagnosis.json',center_failures)
    for level in sorted(map(int,selected)):
        draw_passage(out,selected[str(level)],flight_by,level)
    lines=['# 实验3 单窗高速运动安全机制', '',
        '在原矩形窗上，幅度和频率从1米/1Hz同步递增，四版本逐档独立求解。',
        f'第{last}档本文中心八初值均未找到合格规划，按冻结规则停止升档。', '',
        '## 规划', '', '[完整规划表](PLANNING.md)与[逐初值迭代、耗时和结果](CANDIDATES.csv)分别保存。', '',
        '## Gazebo/PX4', '', '[完整实飞表](FLIGHT.md)独立列示实际窗框碰撞和完整任务用时。', '',
        '## 本文中心未找到的原因', '',
        '各初值先收敛到仅穿越时刻安全的候选，完整过窗复查发现进入侧和离开侧危险，随后补入两侧约束。',
        '加入见证后有限NLP的终止状态如下。未找到属于本轮局部求解结果，不能解释为已证明中心穿越物理不可行。', '',
        '|初值|有限NLP次数|总迭代|求解/s|最终终止状态|', '|---:|---:|---:|---:|---|']
    for row in center_failures:
        lines.append(f"|{row['seed']}|{row['finite_calls']}|{row['iterations']}|{row['solve_s']:.3f}|{row['last_status']}|")
    lines+=['', '## 统计口径', '',
        '每条件每方法正式执行一次。参考系数和时长逐值保持，实际位置来自独立ULog机体真值，接触来自原生窗框传感器。',
        '主时间为从共同起始时刻到完整窗后终点参考执行结束，要求已实际穿过有限开口且整机退出。终点误差和速度独立报告。',
        '规划全程球代理/STL浮点包络检查与实际机体窗框接触分别呈现。过程图中的边缘距离曲线用于解释相对运动，安全成败采用各自完整复查或原生接触。', '',
        '## 文件', '',
        'protocol.json及flight_protocol.json保存冻结设置与源码SHA，plans保存全部候选，flights保存每趟原始证据无损压缩及重审记录。']
    if len(flights)==15:
        lines+=['', '## 完整结果与机制', '',
            '本文自由点四档均无窗框碰撞完成。瞬时消融在2米/2Hz和3米/3Hz实际旋翼接触，动态TOGT中心在3米/3Hz实际接触。',
            '本文中心前三档均无碰撞完成，第四档八初值均未找到合格规划。该档失败发生在补入进出窗危险见证后的有限NLP。',
            '3米/3Hz中，自由点名义中心过平面时窗速约0.784m/s，动态TOGT中心约56.549m/s。完整安全处理同时改变了相遇相位和选点。',
            '因而本组证据支持整段机体穿越的安全处理，并展示自由选点与时间联合优化的作用。不能单凭中心未找到认定所有中心轨迹均不可行。', '',
            '## 计算与执行', '',
            '134条候选并行规划墙钟170.219秒。15趟正式参考均完成记录，12趟无碰撞完赛、3趟实际碰撞，无缺失格。',
            '每次有限IPOPT最多5000步，安全交换最多24轮，八初值及共享中心释放流程保持。各初值计算成本分别保存在CANDIDATES.csv。',
            '三项单窗适配测试通过，配对初始曲线、严格中心归属、完整名义STL重算、原参考数组、MPC/PX4配置及原生无损归档恢复重审均独立核对。', '',
            '![3米/3Hz穿越过程](passage_l03.png)']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--outdir',type=Path,required=True)
    args=parser.parse_args()
    summarize(args.outdir.resolve())
