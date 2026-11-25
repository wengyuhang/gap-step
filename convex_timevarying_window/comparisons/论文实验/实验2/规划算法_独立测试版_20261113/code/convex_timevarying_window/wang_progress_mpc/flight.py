"""Independent algorithm adapter; reuse frozen fixed-plane scene transport."""
import json
from pathlib import Path
import sys

from .reference import Reference
from .worker import Worker
from convex_timevarying_window.comparisons.checked_soft_racing.run import save


def main():
    from convex_timevarying_window.wang_free_state_mpc import px4
    from convex_timevarying_window.comparisons.fixed_plane_wang import flight
    original_argv=sys.argv.copy()
    out=Path(original_argv[original_argv.index('--outdir')+1]).resolve()
    references=[]
    def reference_factory(start):
        ref=Reference(start);references.append(ref);return ref
    px4.Reference=reference_factory
    px4.Worker=Worker
    sys.argv += ['--method','ours']
    try: flight.main()
    finally:
        if out.exists() and references:
            save(out/'shared_passage_contracts.json',references[0].contract_log)
        if (out/'result.json').exists():
            raw=json.loads((out/'result.json').read_text())
            raw.update(method='wang_progress_mpc',algorithm_adapter='common_passage_deadline_v1',
                source_provenance_note='Base source list inherited; full new algorithm manifest is in batch_source_reference.json')
            save(out/'result.json',raw)


if __name__=='__main__':main()
