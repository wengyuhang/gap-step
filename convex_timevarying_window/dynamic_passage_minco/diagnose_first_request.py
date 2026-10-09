"""Replay the recorded failed initial request, not the physical flight."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
from convex_timevarying_window.comparisons.single_gate_translation.scenario import Case, build_case
from .planner import Config, Planner


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args(); out = args.outdir.resolve(); out.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[2]
    original = root/'convex_timevarying_window/dynamic_passage_minco/results/single_gate_v1_20261009'
    raw = json.loads((original/'passage_ours_f3.5_a2_p000/result.json').read_text())
    request = raw['solves'][0]['request_head']
    source = original/'source_snapshot/convex_timevarying_window/dynamic_passage_minco/planner.py'
    name = 'convex_timevarying_window.dynamic_passage_minco._frozen_failed_request'
    spec = importlib.util.spec_from_file_location(name, source)
    frozen = importlib.util.module_from_spec(spec); sys.modules[name] = frozen; spec.loader.exec_module(frozen)
    safety_source = source.with_name('safety.py')
    safety_spec = importlib.util.spec_from_file_location(name+'_safety', safety_source)
    frozen_safety = importlib.util.module_from_spec(safety_spec)
    safety_spec.loader.exec_module(frozen_safety); frozen.passage = frozen_safety.passage
    track, _ = build_case(Case(3.5, 2., 0.)); rows = []
    for label, kind, cfg in (
        ('original_v1', frozen.Planner, frozen.Config(**raw['config'])),
        ('revised_v6', Planner, Config(free_points=True,
            workspace_lower=raw['config']['workspace_lower'], workspace_upper=raw['config']['workspace_upper']))):
        planner = kind(track, cfg)
        try:
            plan = planner.solve(request['pvaj'], request['made_at'], 0)
            row = dict(method=label, found=True, solve_ms=plan.solve_ms,
                       variables=plan.x.tolist(), durations=plan.durations.tolist())
        except Exception as exc:
            row = dict(method=label, found=False, error=repr(exc))
        row['stats'] = planner.solvers[0][0].stats(); rows.append(row)
        print(label, row['found'], row.get('solve_ms'), row.get('error'),
              [[s['status'] for s in attempt['rounds']] for attempt in row['stats']['attempts']], flush=True)
        (out/'replay.json').write_text(json.dumps(dict(request=request, results=rows,
            source_frozen=str(source), safety_source_frozen=str(safety_source), original_trial_not_reflown=True,
            caveat='Numerical request replay on current machine load; not identical runtime scheduling.'), indent=2)+'\n')


if __name__ == '__main__': main()
