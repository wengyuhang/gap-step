"""Isolated single-start planning workers for the frozen matrix."""
import argparse
from pathlib import Path
from . import togt
from ..soft_dynamics import best_center_release, run


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--job', required=True)
    args = parser.parse_args()
    out = args.outdir.resolve()
    job = run.read(out/'jobs'/f'{args.job}.json')
    if job['method'].startswith('dynamic_togt'):
        togt.worker(out, args.job)
    else:
        best_center_release.worker(out, args.job)


if __name__ == '__main__':
    main()
