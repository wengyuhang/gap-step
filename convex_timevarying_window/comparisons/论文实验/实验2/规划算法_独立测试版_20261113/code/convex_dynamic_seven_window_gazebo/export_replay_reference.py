#!/usr/bin/env python3
"""Export a flat-output reference as a Gazebo kinematic replay CSV."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from convex_dynamic_seven_window_gazebo.togt_nmpc import flat_reference_ned


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    reference = np.load(args.reference.resolve())
    state_ned_frd, _ = flat_reference_ned(reference)

    # state is NED/FRD for PX4. Gazebo uses ENU and x500's body frame is FLU.
    c_ne = np.asarray(((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0)))
    c_flu_frd = np.diag((1.0, -1.0, -1.0))
    rotations_ned_frd = Rotation.from_quat(state_ned_frd[:, (7, 8, 9, 6)]).as_matrix()
    rotations_enu_flu = np.einsum("ij,njk,kl->nil", c_ne.T, rotations_ned_frd, c_flu_frd)
    quat_xyzw = Rotation.from_matrix(rotations_enu_flu).as_quat()
    body_rate_flu = state_ned_frd[:, 10:13] @ c_flu_frd
    # In the official x500 SDF, `base_link` (the PX4 local-position body)
    # is 0.24 m above the model root. Planning references describe base_link,
    # while Gazebo's WorldPoseCmd moves the root model.
    base_offset_body = np.asarray((0.0, 0.0, 0.24))
    base_offset_world = np.einsum("nij,j->ni", rotations_enu_flu, base_offset_body)
    world_omega = np.einsum("nij,nj->ni", rotations_enu_flu, body_rate_flu)
    model_position = reference["position_enu"] - base_offset_world
    model_velocity = reference["velocity_enu"] - np.cross(world_omega, base_offset_world)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time_s", "x_enu_m", "y_enu_m", "z_enu_m", "vx_enu_mps",
                         "vy_enu_mps", "vz_enu_mps", "qw", "qx", "qy", "qz",
                         "wx_flu_radps", "wy_flu_radps", "wz_flu_radps"))
        for time_s, position, velocity, quaternion, omega in zip(
                reference["time"], model_position, model_velocity,
                quat_xyzw, body_rate_flu):
            writer.writerow((float(time_s), *map(float, position), *map(float, velocity),
                             float(quaternion[3]), *map(float, quaternion[:3]), *map(float, omega)))


if __name__ == "__main__":
    main()
