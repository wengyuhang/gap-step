"""Independent cold-start solves for the four methods with zero tilt weight."""
import argparse
from pathlib import Path
import sys

from ..basic_group.whole_course_difficulty import scenario
from ..basic_group.plan import nominal_audit
from ..independent_repair.run import frozen_plan, attitude_audit
from ..soft_dynamics import worker as soft_worker
from ..soft_dynamics.run import read, save
from ..static_calibration import planner as togt


METHODS = ('ours', 'center', 'dynamic_togt', 'dynamic_togt_center')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--level', choices=scenario.LEVELS, required=True)
    parser.add_argument('--method', choices=METHODS, required=True)
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--dynamic-limit-fraction', type=float, default=1.)
    parser.add_argument('--seconds-per-seed', type=float, default=180.)
    args = parser.parse_args()
    if args.method in ('ours', 'center'):
        sys.argv = [soft_worker.__name__, '--level', args.level, '--method', args.method,
            '--dynamic-limit-fraction', str(args.dynamic_limit_fraction), '--outdir', str(args.outdir)]
        soft_worker.main()
    else:
        togt.build_case, togt.manifest = scenario.build_case, scenario.manifest
        togt.nominal_audit = nominal_audit
        togt.solve(scenario.Case(level=args.level), args.method, args.outdir,
            dynamic_limit_fraction=args.dynamic_limit_fraction,seconds_per_seed=args.seconds_per_seed)
    raw = read(args.outdir/'result.json')
    raw.update(tilt_penalty_enabled=False, tilt_penalty_weight=0.,
        independent_cold_start=True, cross_condition_warm_start=False)
    if args.method.startswith('dynamic_togt'):
        raw['config'] = dict(radius=.3859447197013662)
        if raw['found']:
            raw['dynamic_soft_integral'] = raw['objective'] = None
            candidate = raw['candidates'][raw['selected_seed']]
            raw['objective'] = candidate['objective']
            raw['dynamic_soft_integral'] = raw['objective']-raw['nominal_time_s']
            save(args.outdir/'attitude_audit.json', attitude_audit(frozen_plan(raw)))
    save(args.outdir/'result.json', raw)
    save(args.outdir/'scene.json', scenario.manifest(scenario.Case(level=args.level)))


if __name__ == '__main__':
    main()
