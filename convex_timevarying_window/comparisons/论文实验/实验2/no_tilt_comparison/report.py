"""Export two five-method comparisons with explicit measurement provenance."""
import argparse
import csv
from pathlib import Path

from ..soft_dynamics import run as common

METHODS = ('ours', 'center', 'dynamic_togt', 'dynamic_togt_center', 'wang2023')
NAMES = ('本文自由点', '本文中心', '动态TOGT自由点', '动态TOGT中心', 'Wang独立复现')
OLD = common.RESULTS/'planning_actual_separated_audit_20261107'


def cell(row, key):
    labels = dict(collision='碰撞', planning_not_found='未找到规划', not_completed='未完成',
        unresolved='证据不足', evidence_failure='证据不足', task_invalid='未完成')
    if row is None:
        return '待运行'
    if not row['passed']:
        return labels.get(row['outcome'], '未完成')
    return '缺测†' if row.get(key) is None else f'{row[key]:.3f}'


def table(rows, key):
    lines = ['|幅度/窗宽|频率倍数|'+'|'.join(NAMES)+'|', '|---|---:|---:|---:|---:|---:|---:|']
    for level in common.scenario.LEVELS:
        by = {r['variant']: r for r in rows if r['level'] == level}
        good = [r[key] for r in by.values() if r['passed'] and r.get(key) is not None]
        cells = []
        for method in METHODS:
            row = by.get(method)
            value = cell(row, key)
            if good and row and row['passed'] and row.get(key) is not None and abs(row[key]-min(good)) < 1e-9:
                value = '**'+value+'**'
            cells.append(value)
        ratio, factor = (0., 1.) if level == 'static' else common.scenario.CONDITIONS[level]
        lines.append('|'+('静态' if level == 'static' else f'{ratio:g}')+f'|{factor:g}|'+'|'.join(cells)+'|')
    return lines


def write_csv(path, rows):
    fields = sorted(set().union(*(r.keys() for r in rows)))
    with path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def collect(out, version, stage):
    old = common.read(OLD/('planning.json' if stage == 'planning' else 'actual.json'))
    fresh = common.read(out/('planning_summary.json' if stage == 'planning' else 'flight_summary.json'))
    rows = []
    for level in common.scenario.LEVELS:
        for method in METHODS:
            use_old = method == 'wang2023' or version == 'hard' and method in ('ours', 'center')
            if use_old:
                source = next(r for r in old if (r['level'], r['variant']) == (level, method))
                passed = source['outcome'] in ('frame_clear', 'old_rule_complete')
                outcome = 'clear' if passed else 'collision' if source['outcome'] in ('collision', 'frame_collision') else source['outcome']
                row = dict(level=level, variant=method, passed=passed, outcome=outcome,
                    data_origin='original_frozen_experiment', source_file=str(OLD/('planning.json' if stage == 'planning' else 'actual.json')),
                    source_sha256=common.previous.digest(OLD/('planning.json' if stage == 'planning' else 'actual.json')),
                    time_s=source.get('planned_full_duration_s') if stage == 'planning' else source.get('new_no_threshold_finish_time_s'),
                    legacy_threshold_time_s=source.get('old_rule_finish_time_s'),
                    measurement_definition='full_seven_gate_return_reference_duration' if stage == 'planning' else 'full_reference_execution_unmeasured_in_legacy_run')
            else:
                matches = [r for r in fresh if (r['level'], r['variant']) == (level, method)]
                if not matches:
                    continue
                source = matches[0]
                passed = source['passed']
                outcome = source.get('outcome', source.get('verdict'))
                if source.get('frame_collision'):
                    outcome = 'collision'
                row = dict(level=level, variant=method, passed=passed, outcome=outcome,
                    data_origin='new_zero_tilt_weight_experiment', source_file=str(Path(source['directory'])/'result.json') if source.get('directory') else None,
                    source_sha256=common.previous.digest(Path(source['directory'])/'result.json') if source.get('directory') else None,
                    time_s=source.get('full_trajectory_time_s') if stage == 'planning' else source.get('full_reference_execution_time_s'),
                    legacy_threshold_time_s=None,
                    measurement_definition='full_seven_gate_return_reference_duration' if stage == 'planning' else 'actual_clock_and_measurement_reach_full_reference_T')
                for key in ('planning_total_s', 'trajectory_distance_m', 'dynamic_soft_integral', 'objective',
                    'truth_goal_error_m', 'truth_goal_speed_mps', 'truth_tracking_rms_m', 'truth_tracking_max_m', 'actual_gate_count'):
                    if key in source:
                        row[key] = source[key]
            rows.append(row)
    return rows


def export(out):
    summaries = {}
    for version in ('hard', 'soft'):
        for stage in ('planning', 'flight'):
            rows = collect(out, version, stage)
            name = ('PLANNING' if stage == 'planning' else 'FLIGHT')+'_'+version.upper()
            title = '原本文硬动力学' if version == 'hard' else '新本文软动力学'
            lines = [f'# {title}五方法'+('规划' if stage == 'planning' else 'Gazebo实飞')+'对比', '',
                '单位秒。完整赛道包含七窗后返航。地面接触排除，窗框碰撞格直接显示碰撞。', '',
                '动态TOGT两方法及本文软版均使用本轮零倾角惩罚的新数据。本文硬版及Wang沿用指定原数据，来源见同名CSV和JSON。', '']
            if stage == 'flight':
                lines += ['本表数值为实际完整参考执行时间，真实末端误差与速度在CSV单列。†表示旧实飞缺少该口径测量，已有七窗和窗框接触结果保留。', '',
                    '加粗仅比较有相同口径实测时间的成功项。旧提前结束规则的实测时间在后表单独列出。', '']
            else:
                lines += ['表内为选中轨迹完整参考时长，所有有效时间最短项加粗。名义球/STL窗框检查与实际实体接触分别评估。', '']
            lines += table(rows, 'time_s')
            if stage == 'flight':
                lines += ['', '## 原实飞记录的历史结束时间', '',
                    '下面保留原实测时间。其结束规则为七窗后距返航点小于0.55m且速度小于1m/s，随后切位置保持。该字段与上表完整参考执行时间分开。', '',
                    '|条件|方法|原实测时间 s|', '|---|---|---:|']
                for row in rows:
                    if row['data_origin'] == 'original_frozen_experiment':
                        value = cell(dict(row, time_s=row['legacy_threshold_time_s']), 'time_s')
                        lines.append(f"|{row['level']}|{NAMES[METHODS.index(row['variant'])]}|{value}|")
            lines += ['', '幅度按每窗水平开口宽度归一化。频率倍数乘七窗基频0.30/0.40/0.50/0.60/0.55/0.45/0.35Hz。四平移、三摆动，初相0，各正式条件一次。']
            (out/(name+'.md')).write_text('\n'.join(lines)+'\n')
            common.save(out/(name+'.json'), rows)
            write_csv(out/(name+'.csv'), rows)
            summaries[name] = {method: dict(
                no_frame_collision_complete=sum(r['variant'] == method and r['passed'] for r in rows),
                collision=sum(r['variant'] == method and r['outcome'] == 'collision' for r in rows),
                planning_not_found=sum(r['variant'] == method and r['outcome'] == 'planning_not_found' for r in rows),
                missing_full_reference_execution_time=sum(r['variant'] == method and r['passed'] and r['time_s'] is None for r in rows)) for method in METHODS}
    common.save(out/'comparison_summary.json', summaries)
    (out/'REPORT.md').write_text('# 无倾角惩罚五方法对比\n\n'+
        '\n'.join(f'- [{name}](./{name}.md)' for name in summaries)+
        '\n\n本文硬版沿用原始数据，本文软版及两个TOGT采用本轮独立规划和实飞。Wang沿用原数据。旧实飞完整参考执行时间缺测，原实测结束时间另表保留。\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    export(parser.parse_args().outdir.resolve())
