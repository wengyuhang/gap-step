"""Validate independent repaired starts on specified frozen tracks, without baselines."""
import argparse
from collections import deque
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from bootstrap import HERE, load


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--levels', nargs='+', required=True)
    cli.add_argument('--outdir', type=Path, required=True)
    cli.add_argument('--concurrency', type=int, default=8)
    args=cli.parse_args()
    args.outdir.mkdir(parents=True, exist_ok=False)
    for name in ['initials', 'jobs', 'logs']:
        (args.outdir/name).mkdir()
    scene, _, _, _, _=load()
    from repair import Planner, explicit_signs
    began=time.perf_counter()
    queue=deque()
    initial_audits=[]
    for level in args.levels:
        track, _=scene.build_case(scene.Case(level))
        init_began=time.perf_counter()
        planner=Planner(track, True, pose_function=scene.pose)
        vectors=planner.seeds()
        audits=[]
        for seed, x in enumerate(vectors):
            g, _, _=planner.evaluate(x, planner.witnesses())
            audits.append(dict(seed=seed, minimum_finite=float(g.min()),
                objective=float(planner.objective(x)[0]),
                dynamic_soft_integral=float(planner.objective(x)[2])))
        signatures=[hashlib.sha256(np.asarray(x).tobytes()).hexdigest() for x in vectors]
        if len(set(signatures))!=8:
            raise ValueError('Initialization collapsed to duplicate vectors')
        initial_path=args.outdir/'initials'/f'{level}.json'
        save(initial_path, dict(level=level, explicit_signs=explicit_signs(track)[0].tolist(),
            seeds=[dict(variables=x.tolist()) for x in vectors],
            generation_s=time.perf_counter()-init_began, audits=audits,
            independent_variables_sha256=signatures, restoration=planner.initial_audits))
        initial_audits.append(dict(level=level, generation_s=time.perf_counter()-init_began,
            all_initial_finite_feasible=all(x['minimum_finite']>=-2e-6 for x in audits)))
        for method in ['ours','center']:
            for seed in range(8):
                job=dict(id=f'{level}_{method}_{seed}', level=level, method=method, seed=seed)
                save(args.outdir/'jobs'/f"{job['id']}.json", job)
                queue.append(dict(job=job, initial=initial_path))
    save(args.outdir/'protocol.json', dict(levels=args.levels, methods=['ours','center'],
        concurrency=args.concurrency, starts_per_method=8, initialization=initial_audits,
        original_frozen_geometry=True, unchanged_objective_and_limits=True,
        cross_start_or_method_warm_start=False, flights=False,
        purpose='Generalization check of direction and initialization repairs'))
    active=[]
    rows=[]
    while queue or active:
        for item in list(active):
            code=item['process'].poll()
            if code is None:
                continue
            item['log'].close()
            active.remove(item)
            job=item['job']
            file=args.outdir/'plans'/job['id']/'result.json'
            raw=json.loads(file.read_text()) if file.exists() else {}
            qualified=bool(code==0 and raw.get('found') and raw.get('independent_frame_clear')
                and raw.get('independent_task_passed'))
            row=dict(**job, exit_code=code, qualified=qualified,
                time_s=raw.get('nominal_time_s'), solve_s=raw.get('solve_s'),
                finite_statuses=[c['status'] for c in raw.get('optimizer_calls',[])], source=str(file))
            rows.append(row)
            save(args.outdir/'summary.json', rows)
            print(row, flush=True)
        while queue and len(active)<args.concurrency:
            item=queue.popleft()
            log=(args.outdir/'logs'/f"{item['job']['id']}.log").open('w')
            process=subprocess.Popen([sys.executable, '-u', str(HERE/'repair_worker.py'),
                '--outdir', str(args.outdir), '--job', item['job']['id'],
                '--initial', str(item['initial'])], stdout=log, stderr=subprocess.STDOUT,
                env=os.environ.copy())
            active.append(dict(**item, process=process, log=log))
        save(args.outdir/'status.json',dict(state='running', complete=len(rows),
            queued=len(queue), elapsed_s=time.perf_counter()-began,
            active=[dict(job=x['job']['id'],pid=x['process'].pid) for x in active]))
        time.sleep(1)
    save(args.outdir/'status.json',dict(state='complete',complete=len(rows),queued=0,
        elapsed_s=time.perf_counter()-began,active=[]))


if __name__=='__main__':
    main()
