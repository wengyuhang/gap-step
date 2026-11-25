"""Freeze and solve the single requested radius ablation independently."""
import argparse
import json
import os
from pathlib import Path
import shutil
import sys

import numpy as np

from .planner import Planner, ORIGINAL_RADIUS_M, SMALL_RADIUS_M
from .. import plan as entry
from ..basic_group.whole_course_difficulty import scenario
from ..gazebo_matrix import run as execution
from ..independent_repair.run import attitude_audit, frozen_plan

HERE = Path(__file__).resolve().parent
ROOT = scenario.ROOT


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    out = args.outdir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    frozen = execution.sources()
    for name in ('planner.py', 'plan.py', 'README.md', 'PROTOCOL.md'):
        path = HERE/name
        frozen[str(path.relative_to(ROOT))] = execution.digest(path)
    prior = HERE.parent/'results/gazebo_all_selected_v1_20261107/source_snapshot'
    for relative, expected in frozen.items():
        target = out/'source_snapshot'/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        source = prior/relative
        if source.is_file() and execution.digest(source) == expected:
            os.link(source, target)
        else:
            shutil.copyfile(ROOT/relative, target)
    save(out/'protocol.json', dict(level='a25_f6', radius_scale=.9,
        original_planning_radius_m=ORIGINAL_RADIUS_M, planning_radius_m=SMALL_RADIUS_M,
        passage_margin_m=.1, physical_x500_scaled=False, independent_cold_starts=True,
        original_seconds_per_seed=300., source_sha256=frozen, protocol=(HERE/'PROTOCOL.md').read_text()))
    save(out/'status.json', dict(state='planning', pid=os.getpid(), level='a25_f6'))
    entry.Case, entry.build_case, entry.manifest = scenario.Case, scenario.build_case, scenario.manifest
    entry.Planner = lambda track, free: Planner(track, free, pose_function=scenario.pose)
    sys.argv = [entry.__name__, '--layout', 'togt_whole', '--level', 'a25_f6', '--method', 'ours',
        '--outdir', str(out/'planning'), '--seconds-per-seed', '300']
    try:
        entry.main()
        raw = json.loads((out/'planning/result.json').read_text())
        checks = dict(passed=True, frozen_files_unchanged=True, all_four_cold_starts_attempted=True,
            selected_plan_found=raw['found'])
        assert raw['all_four_attempted'] and len(raw['candidates']) == 4
        track, _ = scenario.build_case(scenario.Case(level='a25_f6'))
        planner = Planner(track, pose_function=scenario.pose)
        for seed, candidate in zip(planner.seeds(), raw['candidates']):
            np.testing.assert_array_equal(seed, candidate['initial_variables'])
        if raw['found']:
            plan = frozen_plan(raw)
            g, _, curve = planner.evaluate(plan.x, planner.witnesses())
            np.testing.assert_allclose(curve, plan.coefficients, rtol=1e-10, atol=1e-7)
            assert g.min() >= -1e-5
            geometry, passed = planner.verify(plan)
            assert passed and len(geometry) == 56
            save(out/'planning/attitude_audit.json', attitude_audit(plan))
            checks.update(original_node_minimum=float(g.min()), all_56_geometry_passed=True)
        assert all(execution.digest(ROOT/relative) == expected for relative, expected in frozen.items())
        save(out/'planning_validation.json', checks)
        save(out/'status.json', dict(state='planned' if raw['found'] else 'planning_not_found',
            pid=os.getpid(), nominal_time_s=raw.get('nominal_time_s')))
    except BaseException as exc:
        save(out/'status.json', dict(state='stopped', pid=os.getpid(), failure=repr(exc)))
        raise


if __name__ == '__main__':
    main()
