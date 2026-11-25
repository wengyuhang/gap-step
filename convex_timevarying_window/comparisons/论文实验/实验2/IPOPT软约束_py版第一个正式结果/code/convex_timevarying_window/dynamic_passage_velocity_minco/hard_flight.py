"""Versioned hard nominal dynamics experiment; all scene/finish rules retained."""
from . import repaired_flight
from .hard_planner import Config
from .hard_worker import Worker
import json
import sys
from pathlib import Path


def main():
    repaired_flight.Config=Config;repaired_flight.Worker=Worker
    try:repaired_flight.main()
    finally:
        if '--outdir' in sys.argv:
            path=Path(sys.argv[sys.argv.index('--outdir')+1])/'result.json'
            if path.exists():
                raw=json.loads(path.read_text())
                raw.update(method='dynamic_passage_velocity_minco_hard_actuators',
                    forward_tail_bernstein_constraints=False,
                    exit_side_position_bernstein_constraints=True,
                    objective='total_time_only',
                    hard_nominal_dynamics='sampled per-rotor force and body rates; drag flatness',
                    closed_loop_safety_certificate=False)
                path.write_text(json.dumps(raw,indent=2)+'\n')


if __name__=='__main__':main()
