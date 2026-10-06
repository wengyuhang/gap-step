"""Compare frozen direct CTBR trials with the two 2026-09-23 datasets."""
import argparse
import json
from pathlib import Path
import statistics

from .run import conditions

CHECKED = Path('convex_timevarying_window/comparisons/amplitude_racing/results/center_motion_checked_20260923/summary.json')
WANG = Path('convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/summary.json')


def index(rows):
    selected = {}
    for row in rows:
        if 'rate' not in row and 'motion_rate' not in row:
            rate, translation, phase_id, _ = conditions()[row['index']]
            key = (rate, translation, phase_id)
        else:
            key = (int(row.get('rate', row.get('motion_rate'))),
                   int(row['translation_scale']), int(row['phase_id']))
        if key not in selected or selected[key]['infrastructure_failure']:
            selected[key] = row
    return selected


def label(row):
    if row is None:
        return 'NOT RUN'
    if row.get('realtime_success'):
        return f"PASS {row['flight_time_s']:.3f} s"
    if row.get('infrastructure_failure'):
        return 'INFRA FAILURE'
    if row.get('analysis_error'):
        return 'AUDIT MISSING'
    if row.get('collision'):
        return 'CONTACT'
    if row.get('min_clearance_m', 0) < 0:
        return 'SAMPLED OVERLAP'
    if 'fresh_state_CTBR_not_admitted' in str(row.get('failure')):
        return f"NO ADMISSIBLE CTBR ({row.get('gates', 0)}/7)"
    if not row.get('physical_success'):
        return str(row.get('failure') or 'PHYSICAL FAIL').replace('|', '/')
    return 'DEADLINE'


def paired(a, b):
    pairs = [(a[key], b[key]) for key in a.keys() & b.keys()
             if a[key].get('realtime_success') and b[key].get('realtime_success')]
    differences = [x['flight_time_s'] - y['flight_time_s'] for x, y in pairs]
    return dict(joint_success=len(pairs), direct_faster=sum(d < 0 for d in differences),
                baseline_faster=sum(d > 0 for d in differences),
                ties=sum(d == 0 for d in differences),
                median_direct_minus_baseline_s=statistics.median(differences) if differences else None)


def crossing_deltas(direct, checked):
    pairs = [(direct[k], checked[k]) for k in direct.keys() & checked.keys()
             if direct[k].get('realtime_success') and checked[k].get('realtime_success')]
    results = [(json.loads((Path(a['run']) / 'result.json').read_text()),
                json.loads((Path(b['run']) / 'result.json').read_text())) for a, b in pairs]
    # The old result records centre-plane crossings; direct CTBR records both
    # centre-plane and full-body exit. Compare the matching centre events.
    return dict(median_gate1_center_delta_s=statistics.median(
                    a['center_crossings'][0]['time'] - b['crossings'][0]['time']
                    for a, b in results),
                median_gate7_center_delta_s=statistics.median(
                    a['center_crossings'][6]['time'] - b['crossings'][6]['time']
                    for a, b in results),
                median_after_gate7_delta_s=statistics.median(
                    (a['flight_time_s'] - a['center_crossings'][6]['time'])
                    - (b['flight_time_s'] - b['crossings'][6]['time'])
                    for a, b in results)) if results else {}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('direct_dir', type=Path)
    a = p.parse_args()
    all_direct_rows = json.loads((a.direct_dir / 'summary.json').read_text())
    direct = index(all_direct_rows)
    checked = index(json.loads(CHECKED.read_text()))
    wang = index(json.loads(WANG.read_text()))
    keys = [(r, t, phase_id) for r, t, phase_id, _ in conditions()]
    metrics = dict(expected_conditions=len(keys), direct_run=sum(k in direct for k in keys),
                   direct_success=sum(direct[k].get('realtime_success', False) for k in keys if k in direct),
                   checked_success=sum(checked[k].get('realtime_success', False) for k in keys if k in checked),
                   wang_success=sum(wang[k].get('realtime_success', False) for k in keys if k in wang),
                   direct_vs_checked=paired(direct, checked),
                   direct_vs_wang=paired(direct, wang),
                   direct_vs_checked_crossings=crossing_deltas(direct, checked),
                   direct_strict_dynamics_pass=sum(direct[k].get('strict_dynamics_success', False)
                                                   for k in keys if k in direct),
                   direct_infrastructure_retries=sum(x['infrastructure_failure'] for x in all_direct_rows),
                   direct_mppi_max_ms=max((x['mppi_max_ms'] for x in direct.values() if x.get('mppi_max_ms') is not None), default=None),
                   direct_togt_max_ms=max((x['togt_max_ms'] for x in direct.values() if x.get('togt_max_ms') is not None), default=None),
                   direct_min_clearance_m=min((x['min_clearance_m'] for x in direct.values() if x.get('min_clearance_m') is not None), default=None))
    if len(direct) != 24:
        metrics['matrix_complete'] = False
    else:
        metrics['matrix_complete'] = True
    (a.direct_dir / 'comparison.json').write_text(json.dumps(metrics, indent=2) + '\n')
    crossing = metrics['direct_vs_checked_crossings']
    crossing_summary = (f"相对于 checked 的共同通过条件，直接控制的第1/第7门中心过平面时间分别晚"
                        f" {crossing['median_gate1_center_delta_s']:.3f}/"
                        f"{crossing['median_gate7_center_delta_s']:.3f} s（逐条件差值的中位数）；"
                        f"第7门中心过平面至返航的时间段又多 {crossing['median_after_gate7_delta_s']:.3f} s。"
                        '这些是运动结果，不单独证明慢的控制机制。') if crossing else '没有共同成功条件，无法作逐门时间比较。'
    lines = [
        '# Direct CTBR 对比昨日 checked 与 Wang 2023 独立复现',
        '',
        '同一 24 条件动态七窗、PX4/x500/Gazebo 及 2026-09-23 PX4 参数。每条件一次有效运行，',
        '仅基础设施失败允许一次重试。Wang 是独立复现，并非作者官方实现。',
        'Direct CTBR 输出总推力/机体系角速度；旧方法输出 PVA 并使用 50 ms 跟踪预览，',
        '控制接口不是逐项相同。以下主要对齐赛道、物理平台、运动条件和验收口径。',
        '',
        f"严格通过：direct CTBR {metrics['direct_success']}/24，checked {metrics['checked_success']}/24，Wang {metrics['wang_success']}/24。",
        '严格通过 = 顺序七窗返航、零门框接触、10 ms 真值采样整机净空非负、工作区通过、',
        '所有完整在线周期 <100 ms。不是连续安全证书；固定条件不能推断总体成功概率。',
        '',
        f"与 checked 共同通过 {metrics['direct_vs_checked']['joint_success']} 组，direct 较快 {metrics['direct_vs_checked']['direct_faster']} 组；"
        f"共同通过组的 direct−checked 时间差中位数 {metrics['direct_vs_checked']['median_direct_minus_baseline_s']:.3f} s。",
        f"与 Wang 共同通过 {metrics['direct_vs_wang']['joint_success']} 组，direct 较快 {metrics['direct_vs_wang']['direct_faster']} 组；"
        f"共同通过组的 direct−Wang 时间差中位数 {metrics['direct_vs_wang']['median_direct_minus_baseline_s']:.3f} s。",
        crossing_summary,
        f"Direct 全有效批次最大MPPI/TOGT周期 {metrics['direct_mppi_max_ms']:.3f}/{metrics['direct_togt_max_ms']:.3f} ms；"
        f"最小10 ms真值采样机体净空 {metrics['direct_min_clearance_m']:.3f} m；"
        f"基础设施解锁失败额外尝试 {metrics['direct_infrastructure_retries']} 次。",
        '',
        '|频率|中心位移|相位|Direct CTBR|checked|Wang 独立复现|Direct 最小净空 m|',
        '|---:|---:|---:|---|---|---|---:|',
    ]
    for rate, translation, phase_id in keys:
        key = (rate, translation, phase_id)
        row = direct.get(key)
        clear = '—' if row is None or row.get('min_clearance_m') is None else f"{row['min_clearance_m']:.3f}"
        lines.append(f'|{rate}×|{translation}×|{phase_id * 120}°|{label(row)}|{label(checked.get(key))}|{label(wang.get(key))}|{clear}|')
    lines += ['', '没有把失败行补成成功圈速；逐次源码哈希、世界、接触话题、ULog 与数值审计见本结果目录。',
              'Direct 的两次算法失败均为 `fresh_state_CTBR_not_admitted`，对应 3×/2×/120°（已过4门）和 10×/4×/120°（首门前）。'
              ' 最后一轮采样均无合格的2 s序列，主导拒绝位1代表预测门套/工作区几何违反；零实测接触不等于完赛。',
              f"Direct 另有速度/加速度/倾角采样审计，满足其更严格口径的次数为 {metrics['direct_strict_dynamics_pass']}/24。", '']
    (a.direct_dir / 'COMPARISON_REPORT.md').write_text('\n'.join(lines))
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
