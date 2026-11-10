"""Use the common frozen-reference MPC and full-reference execution clock."""
import argparse
from pathlib import Path

from ..radius_ablation.execution import execute


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--method', choices=('ours', 'center'), required=True)
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    execute(args.source.resolve(), args.method, args.outdir.resolve())


if __name__ == '__main__':
    main()
