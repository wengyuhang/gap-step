"""Summarize only completed, independently checked repair validation batches."""
from pathlib import Path
from collections import defaultdict
import csv
import hashlib
import json
import shutil

HERE=Path(__file__).resolve().parent
RESULTS=HERE/'results'
OUT=RESULTS/'repair_verified_v2_20261113'
BATCHES=['repair_low_eight_v2_20261113', 'repair_medium_high_eight_v1_20261113',
    'repair_other_layouts_medium_v1_20261113']


def main():
    OUT.mkdir(exist_ok=True)
    rows=[]
    for batch in BATCHES:
        p=RESULTS/batch
        for row in json.loads((p/'summary.json').read_text()):
            if row.get('level','2026111301_low').endswith('_high'):
                continue
            level=row.get('level','2026111301_low')
            source=p/'plans'/row.get('job', row.get('id', f"{row['method']}_{row['seed']}"))/'result.json'
            raw=json.loads(source.read_text())
            valid=bool(raw.get('found') and raw.get('independent_frame_clear')
                and raw.get('independent_task_passed'))
            rows.append(dict(level=level, method=row['method'], seed=row['seed'],
                qualified=valid, time_s=raw.get('nominal_time_s'), solve_s=raw.get('solve_s'),
                finite_calls=len(raw['optimizer_calls']),
                all_finite_converged=all(c['status']=='Solve_Succeeded' for c in raw['optimizer_calls']),
                final_finite_status=raw['optimizer_calls'][-1]['status'],
                source=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
    rows.sort(key=lambda r:(r['level'],r['method'],r['seed']))
    assert len(rows)==64
    assert all(r['qualified'] and r['all_finite_converged'] for r in rows)
    grouped=defaultdict(list)
    for row in rows:
        grouped[row['level'],row['method']].append(row)
    table=[]
    for (level,method),rr in sorted(grouped.items()):
        tt=[r['time_s'] for r in rr]
        table.append(dict(level=level,method=method,qualified=len(rr),minimum_s=min(tt),
            maximum_s=max(tt),relative_spread_percent=100*(max(tt)/min(tt)-1),
            min_solve_s=min(r['solve_s'] for r in rr),max_solve_s=max(r['solve_s'] for r in rr),
            finite_calls=sum(r['finite_calls'] for r in rr)))
    (OUT/'ALL_STARTS.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n')
    (OUT/'SUMMARY.json').write_text(json.dumps(table,ensure_ascii=False,indent=2)+'\n')
    with (OUT/'ALL_STARTS.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    lines=['# 初始化与方向修复：已完成验证','','本表覆盖第一套布局的低、中难度及另外两套布局的中难度。赛道文件保持冻结，目标函数、安全半径、动力学软约束权重与限制未放宽。','',
        '所有时间均为完整七窗口后返航的名义轨迹时间。64条均通过原完整窗框浮点数值复查及独立STL/顺序唯一穿窗/端点检查；未进行Gazebo实飞。动力学仍为原软约束，不能据此宣称连续硬动力学可行。','',
        '|布局与难度|方法|合格初值|完整时间范围/s|初值间时间差/%|单初值求解范围/s|有限调用|',
        '|---|---|---|---|---|---|---|']
    for r in table:
        label='本文自由点' if r['method']=='ours' else '强制中心'
        lines.append(f"|{r['level']}|{label}|{r['qualified']}/8|{r['minimum_s']:.6f}–{r['maximum_s']:.6f}|{r['relative_spread_percent']:.4f}|{r['min_solve_s']:.2f}–{r['max_solve_s']:.2f}|{r['finite_calls']}，全部Solve_Succeeded|")
    lines+=['','![各初值结果](all_initial_times.png)','','## 实际修复','',
        '1. 穿越方向由布局中相邻入出单位弦的平分方向与固定窗法向确定，所有初值共用。旧规则只看前一个中心位于哪侧，在第一套布局的第五窗会选反。未改窗的位置或运动。',
        '2. 给定各初值的时间和中心点，用最小snap凸QP恢复一致的节点VAJ，并满足原有限问题的方向与工作空间条件；初值动力学代价过大时才适度增加初始时间、重新查询窗口中心。正式NLP各段时间仍独立优化。',
        '3. 原半无限安全补点、软动力学目标、优化后完整复查保持。没有从其他初值、中心优化结果或其他方法借解。','',
        '这里保留归档实现的法向速度至少0.1 m/s、工作空间采样边界和段时间边界；这些不是本次新增条件。该归档版本的时间变量仍为对数段时长。本轮验证没有声称完成新版全C++原生MINCO迁移。','',
        '## 时间口径','',
        '表中求解时间为原result.json的solve_s，不包括初值生成和额外诊断。初值生成时间另见各批initials文件。低难16候选并行墙钟84.05秒；第一套中高难开发批后来中止高难，所以不能把整批墙钟算作中难求解时间。新增两布局32候选、8并发的总墙钟见原status.json。','',
        '## 尚未解决','',
        '高难度使用另一项周期相位全局化开发验证，结果单独保存在 ../repair_phase_eight_v2_20261113/REPORT.md，不混入上面的同版本64条。当前已证明初始化和方向修复能够跨布局工作，尚未证明任意初值都能得到同样好的局部解，也没有全局最优保证。',
        '原验收器遇到额外穿窗事件但没有距离witness时可能不再补点；通用事件修复尚未完成。修正方向后，这64条中没有再触发该失败；这不能代替一般情形的修复证明。','',
        '九赛道五方法正式比较保持暂停。当前算法层为归档Python/CasADi/IPOPT加原C++数值内核；可复现冻结源由bootstrap.py载入，来源SHA见原comparison_v2协议。','',
        '逐初值原始路径和SHA见 ALL_STARTS.json。所有失败开发分支仍保留在results，不纳入本表。']
    (OUT/'REPORT.md').write_text('\n'.join(lines)+'\n')
    (OUT/'source_snapshot').mkdir(exist_ok=True)
    hashes={}
    for name in ['bootstrap.py','scenario.py','repair.py','repair_worker.py','validate_repair.py','summarize_repair.py']:
        src=HERE/name;shutil.copy2(src,OUT/'source_snapshot'/name)
        hashes[name]=hashlib.sha256(src.read_bytes()).hexdigest()
    (OUT/'current_source_sha256.json').write_text(json.dumps(hashes,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,4,figsize=(13.5,3.6),sharey=True,constrained_layout=True)
    for ax,level in zip(axes,sorted({r['level'] for r in rows})):
        for method,color,label in [('ours','#2171b5','Free point'),('center','#d95f02','Center')]:
            rr=sorted(grouped[level,method],key=lambda r:r['seed'])
            ax.plot([r['seed']+1 for r in rr],[r['time_s'] for r in rr],'.-',color=color,label=label,ms=7,lw=.8)
        ax.set_title(level.replace('20261113','Layout '),fontsize=10)
        ax.set_xticks(range(1,9));ax.set_xlabel('Independent start');ax.grid(alpha=.2)
    axes[0].set_ylabel('Full race time incl. return (s)');axes[-1].legend(frameon=False)
    fig.savefig(OUT/'all_initial_times.png',dpi=180)
    print(json.dumps(table,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
