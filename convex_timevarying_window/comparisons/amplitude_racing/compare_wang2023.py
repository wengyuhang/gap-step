"""Write a matched-condition report from the two immutable run summaries."""
import argparse
import json
from pathlib import Path
import statistics


def final_rows(rows, key_fields):
    result = {}
    for row in rows:
        if any(field not in row for field in key_fields):
            continue  # Infrastructure-only attempt; its valid retry is a separate row.
        key = tuple(row[field] for field in key_fields)
        if key not in result or not row.get('infrastructure_failure'):
            result[key] = row
    return result


def outcome(row):
    if row.get('realtime_success'):
        return f"PASS {row['flight_time_s']:.3f} s"
    if row.get('infrastructure_failure'):
        return 'INFRA'
    if row.get('collision'):
        return 'CONTACT'
    if row.get('min_clearance_m', 0) < 0:
        return 'SAMPLED OVERLAP'
    if row.get('room_passed') is False:
        return 'OUT OF BOUNDS'
    if not row.get('completed', row.get('flight_time_s') is not None):
        return str(row.get('failure') or 'INCOMPLETE')
    if row.get('cycle_ms') and row['cycle_ms']['max'] >= 100:
        return 'CYCLE ≥100 ms'
    return 'FAIL / MISSING EVIDENCE'


def report(proposed_root, wang_root):
    proposed_attempts = json.loads((proposed_root/'summary.json').read_text())
    wang_attempts = json.loads((wang_root/'summary.json').read_text())
    proposed = final_rows(proposed_attempts, ('rate','translation_scale','phase_id'))
    wang = final_rows(wang_attempts, ('motion_rate','translation_scale','phase_id'))
    expected = {(rate, amplitude, phase) for rate in (1,3,6,10)
                for amplitude in (1,2,4) for phase in (0,1)}
    if set(proposed) != expected or set(wang) != expected:
        raise ValueError(f'incomplete matrix: proposed {len(proposed)}/24, Wang {len(wang)}/24')

    proposed_ok = sum(bool(row.get('realtime_success')) for row in proposed.values())
    wang_ok = sum(bool(row.get('realtime_success')) for row in wang.values())
    paired = [(proposed[key], wang[key]) for key in sorted(expected)
              if proposed[key].get('realtime_success') and wang[key].get('realtime_success')]
    differences = [w['flight_time_s']-p['flight_time_s'] for p,w in paired]
    lines = [
        '# 2023 Wang 独立复现 vs 当前 checked 方法：窗口中心运动矩阵',
        '',
        '同一动态七窗、PX4/x500、窗口运动函数与相位、60 s 上限、0.05 s 跟踪预览；',
        '频率 1/3/6/10 倍 × 中心位移 1/2/4 倍 × 两个固定相位。GUI 关闭。',
        'Wang 是有披露差异的独立复现，不是作者官方代码或原论文成绩。',
        'Wang 固定穿越动态门中心；当前方法联合优化穿越点并检查局部计划。',
        '',
        f'严格通过：当前方法 {proposed_ok}/24，Wang 独立复现 {wang_ok}/24。',
        '严格通过要求七门返航、零 Gazebo 门套接触、10 ms 真值采样球—门套净空非负、工作区通过，',
        '且完整在线规划周期均 <100 ms；不是连续时间安全证书。',
        f'共同严格通过 {len(paired)} 组；其中 Wang 用时更短 {sum(d<0 for d in differences)} 组，'
        f'当前方法更短 {sum(d>0 for d in differences)} 组。',
    ]
    if differences:
        lines.append(f'共同通过条件下，Wang−当前方法的飞行时间差中位数 {statistics.median(differences):+.3f} s；'
                     '不能把不同成功子集的均值当作公平竞速排名。')
    lines += ['', '|频率|中心位移|相位|当前方法|Wang 独立复现|当前最小净空 m|Wang 最小净空 m|',
              '|---:|---:|---:|---|---|---:|---:|']
    for key in sorted(expected):
        p,w = proposed[key],wang[key]
        clearance = lambda row: f"{row['min_clearance_m']:.3f}" if row.get('min_clearance_m') is not None else '—'
        lines.append(f'|{key[0]}×|{key[1]}×|{0 if key[2]==0 else 120}°|'
                     f'{outcome(p)}|{outcome(w)}|{clearance(p)}|{clearance(w)}|')
    pmax=max(row['cycle_ms']['max'] for row in proposed.values() if row.get('cycle_ms'))
    wmax=max(row['cycle_ms']['max'] for row in wang.values() if row.get('cycle_ms'))
    lines += ['', f'全批次最大完整在线周期：当前 {pmax:.3f} ms，Wang {wmax:.3f} ms。',
              f'基础设施失败额外尝试：当前 {sum(r.get("infrastructure_failure",False) for r in proposed_attempts)} 次，'
              f'Wang {sum(r.get("infrastructure_failure",False) for r in wang_attempts)} 次。',
              '每条件仅两个固定相位，不估计总体成功概率。失败行不报告成功圈速；'
              '真值审计、接触记录、飞控 ULog 和完整源码哈希在各自运行目录。',
              '', f'- 当前方法原始结果：`{proposed_root.resolve()}`',
              f'- Wang 原始结果：`{wang_root.resolve()}`', '']
    (wang_root/'COMPARISON_REPORT.md').write_text('\n'.join(lines))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('proposed_root',type=Path)
    parser.add_argument('wang_root',type=Path)
    args=parser.parse_args()
    report(args.proposed_root,args.wang_root)


if __name__=='__main__':main()
