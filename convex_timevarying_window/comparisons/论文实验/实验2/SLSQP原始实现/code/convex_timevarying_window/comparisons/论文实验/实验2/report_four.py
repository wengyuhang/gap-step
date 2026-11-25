"""Four all-global methods; every failed planning/execution condition retained."""
import csv,json,os
from pathlib import Path
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import save
from .scenario import LAYOUTS,LEVELS
METHODS=('ours','center','dynamic_togt','dynamic_togt_center')
NAMES=dict(ours='本文自由点',center='本文中心',dynamic_togt='动态TOGT自由点',dynamic_togt_center='动态TOGT中心')
LABELS={'completed_no_contact':'无碰撞完赛','collision':'碰撞','missed_gate':'漏窗','not_completed':'未完成','planning_not_found':'未找到合格规划','evidence_failure':'证据不足','pending':'待完成'}

def report(out,rows,final=False):
    out=Path(out);data=[]
    for layout in LAYOUTS:
        for level in LEVELS:
            pair={r['variant']:r for r in rows if r['case']==dict(layout=layout,level=level)};d=dict(layout=layout,level=level)
            for v in METHODS:
                r=pair.get(v,{});d[v+'_verdict']=r.get('verdict','pending');d[v+'_time_s']=r.get('flight_time_s') if r.get('passed') else None
            for va,vb,key in [('ours','center','ours_vs_center_saving_percent'),('dynamic_togt','dynamic_togt_center','togt_vs_center_saving_percent'),('ours','dynamic_togt','ours_vs_togt_saving_percent')]:
                a,b=pair.get(va,{}),pair.get(vb,{});d[key]=100*(b['flight_time_s']-a['flight_time_s'])/b['flight_time_s'] if a.get('passed') and b.get('passed') else None
            data.append(d)
    with (out/'comparison.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
    methods={v:dict(records=sum(r['variant']==v for r in rows),no_collision_completed=sum(r['variant']==v and r.get('passed',False) for r in rows),collisions=sum(r['variant']==v and r.get('collision_detected',False) for r in rows),planning_not_found=sum(r['variant']==v and r.get('verdict')=='planning_not_found' for r in rows)) for v in METHODS}
    save(out/'analysis.json',dict(final=final,records=len(rows),total=36,conditions=data,methods=methods))
    lines=['# 实验2：四方法全局七窗混合运动完整对比','',f"状态：{'完成' if final else '进行中'}，{len(rows)}/36条记录。",'',
        '三赛道×静态/1Hz2m/3Hz4m×四全局方法，各一次初相零。每条均4个平移窗、3个绕外支点摆动窗。',
        '全部一次规划七窗和返航，无停车中间点、无滚动视域或执行中重规划。Wang不在本轮对比，旧记录独立保留。',
        '完整竞速从执行开始到七窗全部整机退出、返航和减速；赛前规划耗时另表。只有实际无碰撞完赛才填有效时间。','',
        '|赛道|运动等级|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|',
        '|---|---|---|---|---|---|']
    for d in data:
        values=[d['layout'],d['level']]
        for v in METHODS:values.append(f"{d[v+'_time_s']:.3f} s · 无碰撞" if d[v+'_time_s'] is not None else LABELS.get(d[v+'_verdict'],d[v+'_verdict']))
        lines.append('| '+' | '.join(values)+' |')
    lines+=['','## 完整规划及计算成本','',
        '|赛道/等级|方法|结果|全局四初值求解s|名义轨迹s|在线规划请求|MPC最大ms|原始记录|',
        '|---|---|---|---:|---:|---:|---:|---|']
    for r in sorted(rows,key=lambda r:(LAYOUTS.index(r['case']['layout']),list(LEVELS).index(r['case']['level']),METHODS.index(r['variant']))):
        fmt=lambda v:f'{v:.3f}' if v is not None else '—';rel=os.path.relpath(Path(r['directory'])/'result.json',out)
        values=[r['case']['layout']+'/'+r['case']['level'],NAMES[r['variant']],LABELS.get(r['verdict'],r['verdict']),fmt(r.get('global_planning_s')),fmt(r.get('nominal_time_s')),str(r.get('planner_requests',0)),fmt(r.get('mpc_max_ms')),f'[原始记录]({rel})']
        lines.append('| '+' | '.join(values)+' |')
    lines+=['','## 方法与口径','',
        '- 本文自由点/中心：完整PVAJ八段曲线，时间目标，名义节点执行量硬约束和全部有限窗框局部浮点包络/见证交换；中心精确锁点。未找到合格规划不飞行，原样保留。',
        '- 动态TOGT自由点/中心：发布版位置型MINCO-s4/C6，原生x500目标/伴随、发布版L-BFGS；p_i=c_i(t_i)+R_i q_i，时刻和选点联合优化、完整时间链式梯度。无本文安全目标/轨迹安全过滤。',
        '- TOGT给予机体半径+0.10m的过面内缩，四组中心空间初值/时间尺度一致，按原TOGT目标选最低收敛候选；本文选最短验收通过候选。二者原目标和约束处理不同，不能当成仅更换选点的消融。',
        '- TOGT使用已有x500适配原生目标（二进制、C++包装源码均冻结）：质量/惯量/推力适配平台，动力学罚权1000；不是静态作者原机参数。本文沿用既有90%规划推力配置。',
        '- 接触以任务区间七窗框+地面20路1000Hz（按7窗+地面汇总）传感器为准；ULog独立核对七窗正向各一次、整机退出、返航终点。窗口真实pose/info与查询逐趟核对。',
        '- 1ms物理与单次仿真观测不升级为闭环安全证书；各条件一次不估计随机成功概率。优化器收敛和名义过窗不等于实飞完成。',
        '- 仅保留旧批次算法/几何未变的全局规划请求，来源见reuse_audit.json；旧实飞接触覆盖不全均排除，本批完整记录所有框边。',
        '- 仅双方无碰撞完成时比较时间，未完成/碰撞不赋有效圈时；planning_not_found不表示已证明无解。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
