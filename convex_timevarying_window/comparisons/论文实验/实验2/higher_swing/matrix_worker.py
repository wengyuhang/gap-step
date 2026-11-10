"""Plan one frozen-method job on the common higher-swing track."""
import argparse
import json
from pathlib import Path

from . import scenario
from . import run as higher
from ..matched_full_matrix import togt
from ..soft_dynamics import run as common
from ..wang2023 import rollout
from ..independent_repair.run import attitude_audit, frozen_plan

MATCHED_METHODS = ('ours', 'center', 'dynamic_togt', 'dynamic_togt_center')
METHODS = (*MATCHED_METHODS, 'wang2023')


def configure():
    common.scenario = scenario
    rollout.scenario = scenario


def solve_wang(out, job):
    """Keep Wang's own local solver and choose candidates by its own Eq. 31."""
    folder = out/'plans'/job['id']
    folder.mkdir(parents=True, exist_ok=False)
    with (folder/'progress.jsonl').open('w') as log:
        def progress(row):
            log.write(json.dumps(row, ensure_ascii=False)+'\n')
            log.flush()
        raw = rollout.solve(job['level'], progress=progress)
    # The flight adapter needs a radius field, and the shared Plan needs x.
    # Preserve every original Wang setting separately from these transport fields.
    original_config = raw['config']
    raw.update(job=job, method='wang2023', variables=[],
        wang_config=original_config, config=dict(original_config, radius=.3859447197013662),
        scenario_variant=scenario.VARIANT, swing_length_m=scenario.SWING_LENGTH,
        dynamic_limit_fraction=.9, number_initial_starts=4,
        optimized_initial_solution_used=False, external_solution_used=False,
        center_solution_used=False, all_starts_attempted=True)
    if raw['found']:
        common.save(folder/'attitude_audit.json', attitude_audit(frozen_plan(raw)))
    common.save(folder/'result.json', raw)
    common.save(folder/'scene.json', scenario.manifest(scenario.Case(level=job['level'])))
    print('WANG_RESULT', job['level'], raw['found'], raw['nominal_time_s'], raw['failure'], flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--job', required=True)
    args = parser.parse_args()
    out = args.outdir.resolve()
    job = common.read(out/'jobs'/f'{args.job}.json')
    configure()
    if job['method'] == 'wang2023':
        solve_wang(out, job)
    elif job['method'].startswith('dynamic_togt'):
        togt.worker(out, args.job)
        folder = out/'plans'/args.job
        raw = common.read(folder/'result.json')
        raw.update(scenario_variant=scenario.VARIANT, swing_length_m=scenario.SWING_LENGTH)
        common.save(folder/'result.json', raw)
        common.save(folder/'scene.json', scenario.manifest(scenario.Case(level=job['level'])))
    else:
        higher.worker(out, args.job)


if __name__ == '__main__':
    main()
