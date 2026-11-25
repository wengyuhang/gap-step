"""Search a shared pool and export exactly eight initial curves per condition."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent/'heuristic_initialization'))
from run_pool import PACKAGE, materialize_inputs, parallel, save


def select_pool(directory, windows):
    pool = []
    for path in sorted(directory.glob('seed_*.json')):
        result = json.loads(path.read_text())
        for index, candidate in enumerate(result['archive']):
            if candidate['quality']['finite']:
                pool.append(dict(candidate, source_seed=result['seed'], archive_index=index,
                                 time_band=result['time_band'], target_time_s=result['target_time_s']))
    periods = np.array([1/w['frequency'] if w['frequency'] > 0 else 1. for w in windows])
    selected = []
    for band in range(4):
        choices = sorted((c for c in pool if c['time_band'] == band),
                         key=lambda c: (c['quality']['maximum_violation'], c['quality']['mean_square_violation']))
        if not choices:
            raise RuntimeError(f'No finite candidate in time band {band}')
        band_selected = []
        for separation in (.25, 1e-5):
            for candidate in choices:
                arrivals = np.array(candidate['arrivals'])
                if all(np.max(abs(arrivals-np.array(old['arrivals']))/periods) >= separation for old in band_selected):
                    band_selected.append(candidate)
                if len(band_selected) == 2:
                    break
            if len(band_selected) == 2:
                break
        if len(band_selected) != 2:
            raise RuntimeError(f'Time band {band} has fewer than two distinct schedules')
        selected.extend(band_selected)
    return selected, len(pool)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cases', type=Path, required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--models', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    begin = time.perf_counter()
    protocol = dict(levels=['a50_f6', 'a75_f6'], independent_searches=16, workers=8,
                    generations=80, population=24, search_dimension=7, final_initials=8, target_time_fractions=[.65,.70,.75,.80], retained_per_time_band=2,
                    point_initialization='moving window center', states='native position-only MINCO actual VAJ',
                    selection='two low-violation distinct arrival schedules per short-total-time band; no global Pareto pruning',
                    fixed_quota_per_search=False, sampled_constraints='original frozen hard dynamics and safety nodes',
                    backend='unchanged original SLSQP; free-point eight-start validation',
                    seconds_per_backend_initial=300., geometry_assessment='unchanged original numeric and independent mesh checks',
                    search_objective='maximum original normalized violation at fixed initial total time', final_backend_total_time='fully free under original solver', normal_warm_model_generation_target='seconds; measured separately from one-time model export',
                    original_baseline=str(PACKAGE/'outputs/original_eight_ten_v1_20261112/run/SUMMARY.json'))
    save(args.out/'PROTOCOL.json', protocol)
    rows = []
    for level in protocol['levels']:
        setup = time.perf_counter()
        original_model = args.models/level
        model = args.out/'models'/level
        model.mkdir(parents=True)
        metadata = json.loads((original_model/'model.json').read_text())
        original_count = len(metadata['original_initials'])
        assert original_count == 8
        metadata['original_initials'] = metadata['original_initials']*2
        save(model/'model.json', metadata)
        os.link(original_model/'constraints.casadi', model/'constraints.casadi')
        generation = args.out/'generation'/level
        generation.mkdir(parents=True)
        runtime_begin = time.perf_counter()
        for wave in range(2):
            commands = [[str(args.binary), str(model), str(seed), '80', '24', str(generation/f'seed_{seed}.json')] for seed in range(8*wave, 8*(wave+1))]
            logs = generation/f'wave_{wave}'
            logs.mkdir()
            parallel(commands, logs, args.out/'status.json', f'CMA_pool_wave_{wave}', level)
        selected, pool_size = select_pool(generation, metadata['windows'])
        materialize_inputs(args.cases, args.out/'cases', level, selected)
        runtime_s = time.perf_counter()-runtime_begin
        setup_s = runtime_begin-setup
        export_s = metadata['export_wall_s']
        cold_generation_s = export_s+setup_s+runtime_s
        initial_result = dict(level=level, count=8, candidates=selected, pool_size=pool_size,
                              initial_generation_wall_s=cold_generation_s, runtime_generation_wall_s=runtime_s,
                              model_setup_s=setup_s, original_model_export_s=export_s,
                              sampled_feasible_count=sum(c['quality']['sampled_feasible'] for c in selected))
        save(generation/'INITIALS.json', initial_result)
        print('POOL_COMPLETE', level, json.dumps({k:v for k,v in initial_result.items() if k != 'candidates'}), flush=True)
        group = args.out/'groups'/level/'ours'
        group.mkdir(parents=True)
        commands = [[sys.executable, '-u', str(PACKAGE/'worker.py'), '--cases', str(args.out/'cases'), '--level', level,
                     '--method', 'ours', '--seed', str(seed), '--out', str(group/f'seed_{seed}')] for seed in range(8)]
        backend_s = parallel(commands, group, args.out/'status.json', 'free_point_backend', level)
        candidates = [json.loads((group/f'seed_{seed}'/'result.json').read_text()) for seed in range(8)]
        qualified = [r for r in candidates if r['passed']]
        best = min(qualified, key=lambda r:r['nominal_time_s']) if qualified else None
        if best:
            save(group/'BEST_RESULT.json', best)
        row = dict(level=level, method='ours', outcome='窗口复查通过' if best else '未找到',
                   time_s=best['nominal_time_s'] if best else None, qualified=len(qualified),
                   initial_generation_wall_s=cold_generation_s, runtime_generation_wall_s=runtime_s,
                   backend_wall_s=backend_s, total_wall_s=cold_generation_s+backend_s,
                   best_seed=best['seed'] if best else None, source=str(group))
        rows.append(row)
        save(args.out/'SUMMARY.json', rows)
        print('METHOD_COMPLETE', json.dumps(row, ensure_ascii=False), flush=True)
    save(args.out/'status.json', dict(state='complete', methods=len(rows), elapsed_s=time.perf_counter()-begin))


if __name__ == '__main__':
    main()
