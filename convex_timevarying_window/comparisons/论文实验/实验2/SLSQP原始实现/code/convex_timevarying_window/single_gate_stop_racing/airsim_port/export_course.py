"""Export exact mesh triangles and known motion; does not create an Unreal scene."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from .. import scenario
from .model import ENU_TO_NED, LOCAL_TO_UNREAL, GenericQuad, enu_to_ned


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', required=True)
    args = parser.parse_args()
    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[3]
    case = scenario.Case(4., 4., 0.)
    track, _ = scenario.build_case(case)
    # Map the unchanged Gazebo ground to AirSim NED z=0.
    # Actual PlayerStart/collision height must be calibrated separately.
    origin = np.r_[track.start[:2], -6.]
    gates = []
    for i, window in enumerate(track.windows):
        entries = {}
        for kind in ('sleeve', 'core'):
            source = root/'convex_dynamic_seven_window_gazebo'/'meshes'/f'gate_{i+1:02d}_{window.aperture.kind}_{kind}.stl'
            if not source.exists():
                # Names use shape labels (rectangle/pentagon/hexagon), not
                # necessarily the aperture class discriminator.
                matches = list(source.parent.glob(f'gate_{i+1:02d}_*_{kind}.stl'))
                if len(matches) != 1:
                    raise RuntimeError(f'Ambiguous original mesh for gate {i+1}/{kind}')
                source = matches[0]
            mesh = trimesh.load_mesh(source, process=False)
            transform = np.eye(4)
            transform[:3,:3] = 100.*LOCAL_TO_UNREAL
            mesh.apply_transform(transform)
            target = out/(source.stem+'.obj')
            mesh.export(target)
            shutil.copyfile(source, out/source.name)
            entries[kind] = dict(source=str(source.relative_to(root)),
                sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                original_stl=source.name, unreal_centimetre_obj=target.name,
                triangles=len(mesh.faces), ue_collision='complex_as_simple_required_not_yet_cooked')
        rotation = window.state_at(0.)[1]
        gates.append(dict(name=window.name, assets=entries,
            center0_ned_m=enu_to_ned(window.center0, origin).tolist(),
            rotation_ned_xyzw=Rotation.from_matrix(ENU_TO_NED@rotation).as_quat().tolist(),
            amplitude_ned_m=(ENU_TO_NED@window.motion.translation_amplitude).tolist(),
            frequency_hz=4., phase_rad=window.motion.phase))
    model = GenericQuad()
    manifest = dict(status='asset_export_only_not_cooked_not_flown',
        source_scenario=scenario.manifest(case), origin_enu_m=origin.tolist(),
        start_ned_m=enu_to_ned(track.start, origin).tolist(),
        goal_ned_m=enu_to_ned(track.goal, origin).tolist(), gates=gates,
        vehicle_source_parameters=asdict(model), inertia_diagonal=model.inertia.tolist(),
        max_rotor_thrust_n=model.max_rotor_thrust,
        max_rotor_torque_nm=model.max_rotor_torque,
        motion_clock='Required: engine simulation clock, not external wall-clock RPC',
        validation_pending=['Unreal importer scale/handedness','physical collision cooking',
            'PlayerStart datum','native inner-loop identification','closed-loop seven-gate race'])
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(dict(gates=len(gates),outdir=str(out),status=manifest['status'])))


if __name__ == '__main__':
    main()
