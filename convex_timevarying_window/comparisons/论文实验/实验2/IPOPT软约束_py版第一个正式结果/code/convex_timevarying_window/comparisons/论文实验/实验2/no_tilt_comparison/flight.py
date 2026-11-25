"""Execute every selected reference with the existing common MPC."""
import argparse
from pathlib import Path

from ..radius_ablation import execution
from ..soft_dynamics.run import save
from .worker import METHODS


class RecordedSession(execution.RecordedSession):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        save(self.out/'reference_activation.json', dict(activated=False))

    def prepare(self):
        super().prepare()
        save(self.out/'reference_activation.json', dict(activated=True,
            definition='preparation passed and full-reference execution entered'))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--method', choices=METHODS, required=True)
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    execution.RecordedSession = RecordedSession
    execution.execute(args.source.resolve(), args.method, args.outdir.resolve())


if __name__ == '__main__':
    main()
