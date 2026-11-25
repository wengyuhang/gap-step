"""One frozen W1 flight: same x500/MPC/scenario, point-only MINCO planner."""
import hashlib
import json
from pathlib import Path
import shutil

from convex_timevarying_window.simple_two_gate_soft_only_mpc import flight as existing
from convex_timevarying_window.comparisons.single_gate_translation import scenario
from .planner import Config
from .worker import Worker


def main():
    existing.Worker = Worker
    existing.variant_config = lambda variant: Config(free_points=variant == 'ours')
    existing.build_case = scenario.build_case
    existing.launch = scenario.launch
    existing.manifest = scenario.manifest
    existing.MAX_FLIGHT_TIME = 40
    try:
        existing.main()
    finally:
        # Keep this new method's exact sources alongside the common snapshots.
        import sys
        if '--outdir' in sys.argv:
            out = Path(sys.argv[sys.argv.index('--outdir')+1]).resolve()
            if out.exists():
                root = Path(__file__).resolve().parents[2]
                hashes = {}
                for source in Path(__file__).parent.glob('*.py'):
                    target = out/'source_snapshot'/source.relative_to(root)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(source, target)
                    hashes[str(source.relative_to(root))] = hashlib.sha256(source.read_bytes()).hexdigest()
                (out/'point_only_sources.json').write_text(json.dumps(hashes, indent=2)+'\n')
                result = out/'result.json'
                if result.exists():
                    raw = json.loads(result.read_text())
                    raw['method'] = 'togt_point_only_minco_'+raw['comparison_variant']
                    raw['trajectory_family'] = 'global_minco_s4_one_piece_per_leg'
                    raw['control_nodes'] = 'start, moving gate crossing points, endpoint only'
                    result.write_text(json.dumps(raw, indent=2)+'\n')


if __name__ == '__main__':
    main()
