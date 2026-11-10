"""Five-method tables with independent planning and measured-flight provenance."""
import csv
from pathlib import Path

from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import run


METHODS = ('ours', 'center', 'dynamic_togt', 'dynamic_togt_center', 'wang2023')
NAMES = ('本文自由点·软约束', '本文中心·软约束', '动态TOGT自由点', '动态TOGT中心', 'Wang独立复现')


def normalize(row, stage):
    outcome = row.get('outcome', row.get('verdict'))
    if row.get('frame_collision'):
        outcome = 'collision'
    key = 'full_trajectory_time_s' if stage == 'planning' else 'full_reference_execution_time_s'
    directory = row.get('directory')
    result = dict(level=row['level'], variant=row['variant'], passed=row['passed'],
        outcome=outcome, time_s=row.get(key), data_origin='new_frozen_run',
        measurement_definition='full_seven_gate_return_reference' if stage == 'planning'
            else 'actual_clock_and_measurement_reach_full_reference_T',
        source_file=str(Path(directory)/'result.json') if directory else None,
        source_sha256=run.previous.digest(Path(directory)/'result.json') if directory else None)
    for name in ('planning_total_s', 'solve_s', 'build_s', 'trajectory_distance_m',
            'dynamic_soft_integral', 'objective', 'actual_gate_count', 'ground_collision',
            'truth_goal_error_m', 'truth_goal_speed_mps', 'truth_tracking_rms_m', 'truth_tracking_max_m'):
        if name in row:
            result[name] = row[name]
    return result


def cell(row):
    if row is None:
        return '待运行'
    if row['outcome'] == 'collision':
        return '碰撞'
    if row['outcome'] == 'planning_not_found':
        return '未找到'
    if not row['passed']:
        return '证据不足' if row['outcome'] in ('unresolved', 'evidence_failure') else '未完成'
    if row.get('time_s') is None:
        return '缺测'
    value = f"{row['time_s']:.3f}"
    return value+'†' if row['measurement_definition'] == 'legacy_terminal_threshold' else value


def table(rows):
    lines = ['|幅度比|频率倍数|'+'|'.join(NAMES)+'|', '|---|---:|---:|---:|---:|---:|---:|']
    for level in run.scenario.LEVELS:
        by = {row['variant']: row for row in rows if row['level'] == level}
        eligible = [row['time_s'] for row in by.values()
            if row['passed'] and row.get('time_s') is not None
            and row['measurement_definition'] != 'legacy_terminal_threshold']
        cells = []
        for method in METHODS:
            row = by.get(method)
            value = cell(row)
            if eligible and row and row['passed'] and row.get('time_s') is not None:
                if row['measurement_definition'] != 'legacy_terminal_threshold' and abs(row['time_s']-min(eligible)) < 1e-9:
                    value = '**'+value+'**'
            cells.append(value)
        ratio, factor = (0., 1.) if level == 'static' else run.scenario.CONDITIONS[level]
        amplitude = '静态' if level == 'static' else f'{ratio:g}'
        lines.append('|'+amplitude+f'|{factor:g}|'+'|'.join(cells)+'|')
    return lines


def export(out):
    statistics = {}
    for stage, filename in (('planning', 'PLANNING'), ('flight', 'FLIGHT')):
        fresh_name = 'planning_summary.json' if stage == 'planning' else 'flight_summary.json'
        rows = [normalize(row, stage) for row in run.read(out/fresh_name)]
        rows += run.read(out/('wang_'+stage+'.json'))
        rows.sort(key=lambda row: (run.scenario.LEVELS.index(row['level']), METHODS.index(row['variant'])))
        assert len({(row['level'], row['variant']) for row in rows}) == len(rows)
        lines = [f"# 五方法{'规划' if stage == 'planning' else 'Gazebo实飞'}结果", '',
            '单位秒。原静态加九组混合运动，四平移三摆动、全部初相零。幅度比按各窗水平开口宽度归一化。', '',
            '本文两种方法采用最新IPOPT软动力学版本，当前95%动力学软上界、固定32区间梯形积分。两个动态TOGT采用相同软参数、积分与逐形状安全内缩区域。',
            '四种方法本轮各自重新规划和实飞，Wang沿用原始数据。碰撞和未找到直接标注。', '']
        if stage == 'planning':
            lines += ['表中时间为完整七窗及返航参考时长。名义球/STL窗框结果决定分类，动力学软超限单列记录。', '']
        else:
            lines += ['四种新方法按实际时钟及测量达到完整返航参考T结束。末端真值误差、速度和实际过窗数在CSV单列。', '',
                '†为Wang历史实测时间，原结束条件为七窗后距返航点小于0.55m且速度小于1m/s。原记录缺少新口径的完整参考执行时间。',
                '加粗仅比较同口径成功项，Wang历史时间保留展示。窗框碰撞优先分类，地面接触单列。', '']
        lines += table(rows)
        lines += ['', '原始数据来源、SHA-256、终止状态和各项诊断见同名JSON/CSV。每个条件每方法原初相各一次。']
        (out/(filename+'.md')).write_text('\n'.join(lines)+'\n')
        run.save(out/(filename+'.json'), rows)
        columns = sorted(set().union(*(row.keys() for row in rows)))
        with (out/(filename+'.csv')).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)
        statistics[stage] = {method: dict(
            recorded=sum(row['variant'] == method for row in rows),
            passed=sum(row['variant'] == method and row['passed'] for row in rows),
            collision=sum(row['variant'] == method and row['outcome'] == 'collision' for row in rows),
            not_found=sum(row['variant'] == method and row['outcome'] == 'planning_not_found' for row in rows),
            incomplete=sum(row['variant'] == method and not row['passed']
                and row['outcome'] not in ('collision', 'planning_not_found') for row in rows))
            for method in METHODS}
    run.save(out/'comparison_summary.json', statistics)
    (out/'REPORT.md').write_text('# 最新软约束版本五方法完整实验\n\n'
        '[规划结果](PLANNING.md)和[实飞结果](FLIGHT.md)分别列出。\n\n'
        '静态加九个动态条件，四个新方法共40项新规划、每条找到的参考一次正式飞行。'
        '每方8组匹配时间冷初值；本文自由点另从最佳中心解开放选点优化一次，再按原目标择优。Wang十组原始规划和实飞沿用历史数据，其实飞计时口径带†标注。\n\n'
        '当前进度见status.json，协议、源码与旧结果哈希见protocol.json。\n')


def self_check():
    assert cell(dict(outcome='collision', passed=False, time_s=3.)) == '碰撞'
    assert cell(dict(outcome='planning_not_found', passed=False)) == '未找到'
    assert cell(dict(outcome='unresolved', passed=False)) == '证据不足'
    assert cell(dict(outcome='clear', passed=True, time_s=4.,
        measurement_definition='legacy_terminal_threshold')) == '4.000†'
    assert cell(None) == '待运行'

