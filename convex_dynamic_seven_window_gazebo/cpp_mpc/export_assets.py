#!/usr/bin/env python3
"""Export the frozen CasADi OCP and a plain-text reference for C++ runtime."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(ROOT))

from convex_dynamic_seven_window_gazebo.togt_nmpc import TOGTTrackingNMPC, flat_reference_ned


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=HERE / "generated")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    reference = np.load(args.reference.resolve())
    state, rotors = flat_reference_ned(reference)
    np.savetxt(output / "reference.csv",
               np.column_stack((reference["time"], state, rotors)),
               delimiter=",", fmt="%.17g")
    controller = TOGTTrackingNMPC()
    controller.solver.save(str(output / "solver.casadi"))
    controller.rollout.save(str(output / "rollout.casadi"))
    print(output)


if __name__ == "__main__":
    main()
