"""Versioned execution repair, preserving old flight entry and frozen results."""
import json
from pathlib import Path
import sys
from . import flight
from .repaired_planner import Config
from .repaired_worker import Worker
from .allocation_mpc import MPC


def main():
    from convex_timevarying_window.simple_two_gate_soft_only_mpc import px4
    flight.Worker=Worker;flight.Config=Config;px4.MPC=MPC
    try:flight.main()
    finally:
        if '--outdir' in sys.argv:
            path=Path(sys.argv[sys.argv.index('--outdir')+1])/'result.json'
            if path.exists():
                raw=json.loads(path.read_text());raw['method']='dynamic_passage_velocity_minco_execution_repair'
                raw['execution_repair']=True
                path.write_text(json.dumps(raw,indent=2)+'\n')


if __name__=='__main__':main()
