"""Same physical flight for both methods; only the multistart worker changes."""
import json
from pathlib import Path
import sys
from convex_timevarying_window.comparisons.single_gate_swing import flight as original
from .worker import Worker


def main():
    output=Path(sys.argv[sys.argv.index('--outdir')+1]).resolve()
    original.Worker=Worker
    try:
        original.main()
    finally:
        if (output/'result.json').exists():
            raw=json.loads((output/'result.json').read_text())
            raw.update(method='multistart_minco_'+raw['comparison_variant'],
                initialization_policy='four_time_starts_each_at_center',
                selection_policy='shortest_converged_and_admitted_nominal_duration',
                preliminary_center_solve=False, all_candidate_cost_in_planning_time=True,
                same_seed_protocol_both_methods=True, equal_wall_clock_compute_claim=False,
                nominal_global_optimality=False)
            (output/'result.json').write_text(json.dumps(raw,indent=2)+'\n')


if __name__=='__main__':main()
