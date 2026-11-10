"""Execute a reference against the same shorter-pendulum model used to plan it."""
import argparse
from pathlib import Path

from ..radius_ablation import execution
from ..no_tilt_comparison.flight import RecordedSession
from ..soft_dynamics.run import read
from . import scenario


def configure():
    execution.scenario = scenario
    execution.world.launch = scenario.launch
    execution.RecordedSession = RecordedSession


def audit(folder):
    configure()
    return execution.audit(folder)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    source = args.source.resolve()
    raw = read(source)
    if raw.get('scenario_variant') != scenario.VARIANT:
        raise ValueError('Expected a reference planned for the shorter pendulum')
    configure()
    execution.execute(source, 'ours', args.outdir.resolve())


if __name__ == '__main__':
    main()
