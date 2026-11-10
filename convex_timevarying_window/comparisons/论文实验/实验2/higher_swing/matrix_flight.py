"""Execute all five exact references with the same MPC/PX4 and higher swing."""
import argparse
from pathlib import Path

from . import flight, scenario
from .matrix_worker import METHODS
from ..radius_ablation import execution
from ..soft_dynamics.run import read


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--method', choices=METHODS, required=True)
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    source = args.source.resolve()
    raw = read(source)
    if raw.get('scenario_variant') != scenario.VARIANT:
        raise ValueError('Expected the frozen higher-swing scenario')
    if raw['method'] != args.method:
        raise ValueError('Reference method disagrees with requested method')
    flight.configure()
    execution.execute(source, args.method, args.outdir.resolve())


if __name__ == '__main__':
    main()
