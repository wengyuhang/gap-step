"""Fly an existing higher-swing reference with scheduled MPC execution."""
import argparse
import json
from pathlib import Path

from ..higher_swing import flight, scenario
from ..radius_ablation import execution
from .scheduled_mpc import MPC, ScheduledSession


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--method', required=True)
    parser.add_argument('--outdir', required=True, type=Path)
    args = parser.parse_args()
    raw = json.loads(args.source.read_text())
    if raw['scenario_variant'] != scenario.VARIANT or raw['method'] != args.method:
        raise ValueError('Reference scenario or method mismatch')
    flight.configure()
    execution.RecordedSession = ScheduledSession
    execution.execute(args.source.resolve(), args.method, args.outdir.resolve(), tracker_factory=MPC)


if __name__ == '__main__':
    main()
