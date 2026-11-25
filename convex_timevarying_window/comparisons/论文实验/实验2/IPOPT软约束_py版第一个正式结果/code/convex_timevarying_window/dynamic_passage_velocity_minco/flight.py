"""Frozen seven-variable, forward-tail variant; same physical flight task."""
import hashlib
import json
from pathlib import Path
import shutil
import sys
from convex_timevarying_window.simple_two_gate_soft_only_mpc import flight as existing
from convex_timevarying_window.comparisons.single_gate_translation import scenario
from .planner import Config
from .worker import Worker


def main():
    existing.Worker = Worker
    existing.variant_config = lambda variant: Config(free_points=variant == 'ours',
        initial_tail_scale=1., forward_tail=True, log_cost_scaling=True)
    existing.build_case = scenario.build_case
    existing.launch = scenario.launch
    existing.manifest = scenario.manifest
    existing.MAX_FLIGHT_TIME = 40
    try:
        existing.main()
    finally:
        if '--outdir' in sys.argv:
            out = Path(sys.argv[sys.argv.index('--outdir')+1]).resolve()
            if out.exists():
                root = Path(__file__).resolve().parents[2]; hashes = {}
                sources = list(Path(__file__).parent.glob('*.py'))
                sources += [root/'convex_timevarying_window/dynamic_passage_minco'/name
                            for name in ('planner.py', 'safety.py')]
                for source in sources:
                    target = out/'source_snapshot'/source.relative_to(root)
                    target.parent.mkdir(parents=True, exist_ok=True); shutil.copyfile(source, target)
                    hashes[str(source.relative_to(root))] = hashlib.sha256(source.read_bytes()).hexdigest()
                (out/'passage_sources.json').write_text(json.dumps(hashes, indent=2)+'\n')
                if (out/'result.json').exists():
                    raw = json.loads((out/'result.json').read_text())
                    raw.update(method='dynamic_passage_velocity_minco_'+raw['comparison_variant'],
                        trajectory_family='minimum_snap_with_crossing_velocity_C5_two_pieces',
                        safety_model='unchanged local adaptive moving-frame constraints; float bounds',
                        forward_tail_bernstein_constraints=True, post_trajectory_validation_enabled=True,
                        terminal_PVAJ_unchanged=True,
                        numerical_objective_transform='log(1+J); strictly increasing; original J logged')
                    (out/'result.json').write_text(json.dumps(raw, indent=2)+'\n')


if __name__ == '__main__': main()
