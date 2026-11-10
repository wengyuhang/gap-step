"""Same x500/PX4 flight and audit; only the planner worker changes."""
import json
from pathlib import Path
import sys
from convex_timevarying_window.comparisons.single_gate_swing import flight as original
from .worker import Worker


def main():
    output=Path(sys.argv[sys.argv.index('--outdir')+1]).resolve()
    original.Worker = Worker
    try:
        original.main()
    finally:
        if (output/'result.json').exists():
            raw=json.loads((output/'result.json').read_text())
            enabled=raw['comparison_variant']=='ours'
            raw.update(method='center_initialized_minco' if enabled else 'original_center_minco',
                initialization_policy='center_then_free_incumbent' if enabled else 'original_center_single_stage',
                both_stages_in_planning_time=enabled, equal_compute_comparison=False,
                nominal_guarantee_scope='same-request center trajectory; not separate closed-loop flight')
            (output/'result.json').write_text(json.dumps(raw,indent=2)+'\n')


if __name__ == '__main__':
    main()
