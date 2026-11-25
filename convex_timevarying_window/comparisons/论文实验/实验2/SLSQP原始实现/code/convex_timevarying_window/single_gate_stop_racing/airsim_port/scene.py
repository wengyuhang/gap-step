"""Native Blocks mesh approximation. No drone trajectory replay."""
import argparse
import json
import time
import uuid
from dataclasses import replace
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from ..scenario import Case, build_case, manifest
from .model import ENU_TO_NED, enu_to_ned
from .rpc import Client

FRAME_OUTWARD_OFFSET=.065
FRAME_RADIUS=.085


def scaled_track(scale=1.,spacing_scale=1.):
    if not np.isfinite(scale) or scale<=0:raise ValueError('aperture scale must be positive')
    if not np.isfinite(spacing_scale) or spacing_scale<=0:raise ValueError('spacing scale must be positive')
    track,_=build_case(Case(4.,4.,0.))
    for window in track.windows:
        window.center0=window.center0.copy()
        window.center0[:2]=track.start[:2]+spacing_scale*(window.center0[:2]-track.start[:2])
        a=window.aperture
        window.aperture=(replace(a,radius=a.radius*scale,margin=a.margin*scale)
            if a.kind=='circle' else replace(a,physical_vertices=a.physical_vertices*scale,
                safe_vertices=a.safe_vertices*scale,margin=a.margin*scale))
    return track


def vector(x):
    return dict(zip(('x_val', 'y_val', 'z_val'), map(float, x)))


def outward_offset_polygon(points, distance):
    points = np.asarray(points, dtype=float)
    edges = np.roll(points, -1, axis=0)-points
    tangents = edges/np.linalg.norm(edges, axis=1)[:, None]
    orientation = np.sign(np.sum(points[:, 0]*np.roll(points[:, 1], -1)
                                -np.roll(points[:, 0], -1)*points[:, 1]))
    normals = orientation*np.c_[tangents[:, 1], -tangents[:, 0]]
    directions = normals+np.roll(normals, 1, axis=0)
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    return points+distance*directions/np.sum(directions*normals, axis=1)[:, None]


def frame_vertices(aperture,offset=FRAME_OUTWARD_OFFSET):
    if aperture.kind=='circle':
        a=np.arange(24)*2*np.pi/24
        return (aperture.radius+offset)/np.cos(np.pi/24)*np.c_[np.cos(a),np.sin(a)]
    return outward_offset_polygon(aperture.physical_vertices,offset)


def pose(p, rotation=np.eye(3)):
    q = Rotation.from_matrix(rotation).as_quat()
    return dict(position=vector(p), orientation=dict(zip(
        ('x_val', 'y_val', 'z_val', 'w_val'), map(float, q))))


class Scene:
    def __init__(self, client, aperture_scale=1.,spacing_scale=1.):
        self.client = client
        self.track=scaled_track(aperture_scale,spacing_scale)
        self.origin = np.r_[self.track.start[:2], -6.]
        self.parts = []
        self.session = uuid.uuid4().hex[:8]

    def build(self):
        # Only dedicated-session scenery, never cameras/vehicle/ground.
        removed = []
        for pattern in ('TemplateCube.*', 'Cylinder.*', 'Cone_5', 'OrangeBall', 'RaceGate.*'):
            for name in self.client.call('simListSceneObjects', pattern):
                if self.client.call('simDestroyObject', name):
                    removed.append(name)
        for i, gate in enumerate(self.track.windows):
            aperture = gate.aperture
            points=frame_vertices(aperture)
            for j, (a, b) in enumerate(zip(points, np.roll(points, -1, axis=0))):
                # Window API uses local (u,v,normal), unlike STL export axes.
                local = np.r_[(a+b)/2, 0.]
                direction = np.r_[b-a, 0.]
                length = np.linalg.norm(direction)
                z = direction/length
                x = np.array([0., 0., 1.]); y = np.cross(z, x)
                rotation = np.column_stack((x, y, z))
                name = f'RaceGate_{i+1:02d}_{j:02d}_{self.session}'
                center, rg = gate.state_at(0.)[:2]
                actual = self.client.call('simSpawnObject', name, 'Cylinder',
                    pose(enu_to_ned(center+rg@local, self.origin), ENU_TO_NED@rg@rotation),
                    vector([2*FRAME_RADIUS,2*FRAME_RADIUS,length]), False, False)
                if not actual:
                    raise RuntimeError('Cannot spawn '+name)
                self.parts.append((actual, i, local, rotation))
        return removed

    def update(self, t, gate_indices=None):
        states = [gate.state_at(t) for gate in self.track.windows]
        requests = []
        for name, i, local, rotation in self.parts:
            if gate_indices is not None and i not in gate_indices:
                continue
            center, rg = states[i][:2]
            target = enu_to_ned(center+rg@local, self.origin)
            requests.append(('simSetObjectPose', (name,
                pose(target, ENU_TO_NED@rg@rotation), True)))
        # False is also returned for unchanged poses. Movement correctness is
        # established by readback tests, not by interpreting this flag alone.
        return self.client.batch(requests)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', required=True)
    args = parser.parse_args()
    out = Path(args.outdir); out.mkdir(parents=True, exist_ok=False)
    client = Client(timeout=30.)
    # Both physics-disabled and enabled spawned primitives are movable.
    # The native setter returns False for unchanged locations, not just errors.
    # Disable rigid-body simulation so externally prescribed gates cannot fall.
    client.call('simPause', True)
    scene = Scene(client)
    removed = scene.build()
    elapsed = []
    for t in (0., .01, .02):
        started = time.perf_counter(); scene.update(t)
        elapsed.append(time.perf_counter()-started)
    meshes = client.call('simGetMeshPositionVertexBuffers')
    gate_meshes = [m for m in meshes if 'RaceGate' in m.get('name', '')]
    result = dict(status='scene_only_not_flown_collision_unverified',
        scenario=manifest(Case(4., 4., 0.)), parts=len(scene.parts),
        removed_scenery=removed, update_wall_seconds=elapsed,
        mesh_names=[m.get('name') for m in gate_meshes],
        approximation='24-sided circumscribed circular frames; native Cylinder rods, assumed 1m base dimensions pending mesh measurement',
        collision_verified=False, native_meshes=meshes,
        paused=True, physics_enabled_parts=False,
        pending='Stepped motion driver and physical collision verification')
    (out/'scene.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ('native_meshes','scenario','removed_scenery','mesh_names')}))


if __name__ == '__main__':
    main()
