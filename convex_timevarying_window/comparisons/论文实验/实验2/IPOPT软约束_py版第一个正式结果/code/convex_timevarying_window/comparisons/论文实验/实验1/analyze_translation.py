"""All-case reporting; physical failure never receives a valid racing time."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import numpy as np

LABELS = dict(completed_no_contact='无碰撞完赛', collision='碰撞', missed_gate='漏窗',
    not_completed='未完成', online_no_accepted_plan='未接纳初始规划', evidence_failure='证据不足')


def analyze(output):
    output = Path(output)
    status = json.loads((output/'status.json').read_text())
    if status['state'] != 'complete' or not status.get('sources_unchanged'):
        raise RuntimeError('batch is incomplete or source freeze was not verified')
    rows = json.loads((output/'summary.json').read_text())
    rows.sort(key=lambda r: (r['amplitude_m'], r['frequency_hz'], r['variant']))
    cases = sorted({(r['amplitude_m'], r['frequency_hz']) for r in rows})
    paired = []
    for amplitude, frequency in cases:
        pair = {r['variant']: r for r in rows if (r['amplitude_m'], r['frequency_hz']) == (amplitude, frequency)}
        if len(pair) != 2:
            raise RuntimeError('missing pair')
        if all(pair[v]['passed'] for v in ('ours', 'center')):
            a, b = pair['ours']['flight_time_s'], pair['center']['flight_time_s']
            paired.append(dict(amplitude_m=amplitude, frequency_hz=frequency,
                free_time_s=a, center_time_s=b, saving_s=b-a, saving_percent=100*(b-a)/b))
    result = dict(flights=len(rows), conditions=len(cases), per_condition_repetitions=1,
        probability_estimate=False, motion='linear horizontal sinusoid, NOT pivoted pendulum', paired=paired)
    result['variants'] = {}
    for variant in ('ours', 'center'):
        selected = [r for r in rows if r['variant'] == variant]
        raw = [json.loads((Path(r['directory'])/'result.json').read_text()) for r in selected]
        planning = [s['cycle_ms'] for r in raw for s in r['solves']]
        mpc = [s['cycle_ms'] for r in raw for s in r['mpc_cycles'] if s.get('time', 0) >= 0]
        def timing(values):
            return dict(count=len(values), p50_ms=float(np.median(values)), p95_ms=float(np.percentile(values, 95)),
                maximum_ms=float(max(values)), over_100ms=sum(x > 100 for x in values)) if values else None
        result['variants'][variant] = dict(completed_no_collision=sum(bool(r['passed']) for r in selected),
            collisions=sum(bool(r.get('collision_detected')) for r in selected),
            incomplete=sum(not r['passed'] and not r.get('collision_detected') for r in selected),
            planner=timing(planning), mpc=timing(mpc))
    dynamic = [p['saving_percent'] for p in paired if p['amplitude_m'] > 0]
    result['dynamic_pair_saving_percent_mean'] = float(np.mean(dynamic)) if dynamic else None
    result['dynamic_pair_saving_percent_range'] = [float(min(dynamic)), float(max(dynamic))] if dynamic else None
    (output/'analysis.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    fields = ['amplitude_m', 'frequency_hz', 'variant', 'verdict', 'passed', 'flight_time_s',
        'frame_collision', 'ground_collision', 'failure', 'planner_max_ms', 'mpc_max_ms', 'directory']
    with (output/'all_results.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader(); writer.writerows(rows)
    lines = ['# 实验1A：单窗口周期平移', '',
        '当前PV-MINCO／单重射击MPC／x500／PX4；每条件各一次、相位0。不是圆弧摆动，不估计随机成功概率。',
        '时间含首解等待、穿窗、到共同窗后终点和刹车；规划无墙钟截止，超过100ms单独记录。', '',
        '|振幅m|频率Hz|自由结果|自由完整s|中心结果|中心完整s|自由节省|',
        '|---:|---:|---|---:|---|---:|---:|']
    for amplitude, frequency in cases:
        pair = {r['variant']: r for r in rows if (r['amplitude_m'], r['frequency_hz']) == (amplitude, frequency)}
        values = [f'{amplitude:g}', '静止' if amplitude == 0 else f'{frequency:g}']
        for v in ('ours', 'center'):
            r = pair[v]
            values += [LABELS.get(r['verdict'], r['verdict']), f"{r['flight_time_s']:.3f}" if r['passed'] else '—']
        p = next((p for p in paired if (p['amplitude_m'], p['frequency_hz']) == (amplitude, frequency)), None)
        values.append(f"{p['saving_percent']:.2f}%" if p else '—')
        lines.append('| '+' | '.join(values)+' |')
    lines += ['', '## 解释', '', '未接纳规划不是碰撞或物理不可行；双方零接触不能解释为碰撞概率优势。',
        '全部失败保留，时间收益只对共同无碰撞完成的条件计算。',
        '原始result、接触、ULog、实际初态、源码快照及逐对配置审计均在各子目录。',
        '物理步长1ms，高速运动下仍需独立步长复核；名义包络和节点动力学不构成闭环安全证书。',
        '', '## 计时统计', '', '```json', json.dumps(result['variants'], ensure_ascii=False, indent=2), '```']
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')
    fig, axes = plt.subplots(2, 2, figsize=(10, 7), sharex='col')
    for j, amplitude in enumerate((2., 4.)):
        for variant, label, color, marker in [('ours', 'Free point', '#167d9a', 'o'), ('center', 'Fixed centre', '#d67125', 's')]:
            data = [r for r in rows if r['variant'] == variant and r['amplitude_m'] == amplitude]
            frequencies = [r['frequency_hz'] for r in data]
            axes[0, j].plot(frequencies, [r['flight_time_s'] if r['passed'] else np.nan for r in data],
                color=color, marker=marker, label=label)
            axes[1, j].scatter(np.asarray(frequencies)+(-.045 if variant == 'ours' else .045),
                [2 if r['passed'] else 0 if r.get('collision_detected') else 1 for r in data], color=color, marker=marker)
        axes[0, j].set(title=f'Amplitude {amplitude:g} m', ylabel='Full task time (s), completed only')
        axes[1, j].set(xlabel='Frequency (Hz)', ylim=(-.3, 2.3))
        axes[1, j].set_yticks([0, 1, 2], ['Collision', 'Incomplete', 'Completed'])
        for ax in axes[:, j]: ax.grid(alpha=.25)
    axes[0, 0].legend(); fig.tight_layout()
    fig.savefig(output/'full_task_time_and_outcomes.png', dpi=190); plt.close(fig)
    representatives = ([paired[0]] if paired else []) + ([max([p for p in paired if p['amplitude_m'] > 0],
        key=lambda p: (p['amplitude_m'], p['frequency_hz']))] if dynamic else [])
    if representatives:
        fig, axes = plt.subplots(len(representatives), 2, figsize=(10, 4*len(representatives)), squeeze=False)
        for i, condition in enumerate(representatives):
            for j, variant in enumerate(('ours', 'center')):
                row = next(r for r in rows if r['variant'] == variant and r['amplitude_m'] == condition['amplitude_m'] and r['frequency_hz'] == condition['frequency_hz'])
                trace = np.load(Path(row['directory'])/'mechanism_trace.npz')
                at = row['crossings'][0]['time_s']; selected = np.abs(trace['times']-at) < .12
                q = trace['local_positions']; ax = axes[i, j]
                ax.add_patch(Rectangle((-1.45, -1.15), 2.9, 2.3, fill=False, linewidth=3, color='black'))
                ax.plot(q[selected, 0], q[selected, 1], '.-', color=('#167d9a' if variant == 'ours' else '#d67125'))
                raw = json.loads((Path(row['directory'])/'result.json').read_text()); radius = raw['config']['radius']
                for t in (row['entry_time_s'], at, row['exit_time_s']):
                    values = [np.interp(t, trace['times'], q[:, k]) for k in range(3)]
                    section = np.sqrt(max(0., radius**2-values[2]**2))
                    ax.add_patch(Circle(values[:2], section, fill=False, alpha=.65))
                ax.scatter(*row['crossings'][0]['point_local_m'], marker='x', color='red', zorder=5)
                ax.set(xlabel='Gate-local axis 1 (m)', ylabel='Gate-local axis 2 (m)', xlim=(-1.8, 1.8), ylim=(-1.5, 1.5),
                    title=f"{variant}: A={condition['amplitude_m']:g} m, f={condition['frequency_hz']:g} Hz")
                ax.set_aspect('equal'); ax.grid(alpha=.2)
        fig.suptitle('Measured passage in moving gate coordinates; circles: sphere-proxy plane sections')
        fig.tight_layout(); fig.savefig(output/'measured_passage_sections.png', dpi=190); plt.close(fig)
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('directory', type=Path)
    analyze(parser.parse_args().directory)
