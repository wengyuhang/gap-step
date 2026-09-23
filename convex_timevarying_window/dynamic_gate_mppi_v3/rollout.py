"""Batched native x500 motor-input integration and hard screening."""
import ctypes
from dataclasses import dataclass
from pathlib import Path
import subprocess
import numpy as np
from .config import Config
from .geometry import Geometry, GateProgress

HERE = Path(__file__).resolve().parent
MASS = 2.0643076923076924
KF = 8.54858e-6
MIXER = np.array([[1., 1., 1., 1.], [-.174, .174, .174, -.174],
                  [-.174, .174, -.174, .174], [-.016, -.016, .016, .016]])


def native(controls=50,ctbr=False):
    source, library = HERE / "native/rollout.cpp", HERE / f"native/build/libv3_{'ctbr' if ctbr else 'rollout'}_h{controls}.so"
    if not library.exists() or library.stat().st_mtime < source.stat().st_mtime:
        library.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(["g++", "-std=c++17", "-O3", "-fPIC", "-shared", "-fopenmp",
                        f"-DV3_CONTROLS={controls}", f"-DV3_CTBR={int(ctbr)}",str(source), "-o", str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    ptr = ctypes.c_void_p
    lib.v3_integrate.argtypes = [ptr, ptr, ctypes.c_double, ctypes.c_int]
    lib.v3_integrate.restype = None
    lib.v3_guided.argtypes = [ctypes.c_int,ctypes.c_int]+[ptr]*4+[ctypes.c_int,ctypes.c_int,ptr,ptr]
    lib.v3_guided.restype = None
    lib.v3_rollout.argtypes = [ctypes.c_int]*3 + [ptr]*8 + [ctypes.c_int]*2 + [ptr]*7
    lib.v3_rollout.restype = None
    return lib


@dataclass
class Batch:
    states: np.ndarray
    clearances: np.ndarray
    valid: np.ndarray
    indices: np.ndarray
    race: np.ndarray
    phases: np.ndarray
    rejection_bits: np.ndarray | None = None


class Rollout:
    ctbr = False
    def __init__(self, track, config=Config()):
        if (config.dt, config.integration_dt, config.geometry_dt) != (.02, .005, .01):
            raise ValueError("native V3 kernel implements the PDF's 20/5/10 ms, 1 s schedule")
        self.track, self.config = track, config
        self.geometry = Geometry(track, config)
        self.nc = int(round(config.horizon/config.dt))
        self.ns = 2*self.nc
        if self.nc<50 or self.nc%5:raise ValueError("horizon must be >=1 s and a multiple of .1 s")
        self.lib = native(self.nc,self.ctbr)
        self.signs = np.ascontiguousarray(GateProgress(track, config).signs)
        self.geom = np.zeros((len(track.order), 32))
        for i, wi in enumerate(track.order):
            f = self.geometry.frames[wi]
            if f.kind == "circle": self.geom[i, 1] = f.radius
            else:
                if len(f.physical_vertices) > 14: raise ValueError("native frame supports at most 14 sides")
                self.geom[i, :3] = [1, 0, len(f.physical_vertices)]
                self.geom[i, 3:3+2*len(f.physical_vertices)] = f.physical_vertices.ravel()
        self.cfg = np.array([config.radius, config.error_margin, config.frame_thickness,
                             config.speed, config.acceleration, np.cos(np.deg2rad(config.tilt)),
                             *config.lower, *config.upper, *track.goal,config.feedback_geometry_reserve,float(config.staged_entry)])
        self.frame_speeds = np.ascontiguousarray(self.geometry.frame_speeds[list(track.order)])

    def prepare(self, now, local_points=None):
        times = now+np.arange(self.ns+1)*.01
        centers, rotations = self.geometry.poses(times)
        order = list(self.track.order)
        centers, rotations = centers[:, order], rotations[:, order]
        poses = np.ascontiguousarray(np.concatenate((centers, rotations.reshape(self.ns+1, len(order), 9)), axis=-1))
        guides = np.zeros((self.ns+1, len(order)+1, 3))
        if local_points is None: local_points = np.zeros((len(order), 2))
        guides[:, :-1] = (centers + np.einsum("tgij,gj->tgi", rotations[..., :2], local_points)
                         + rotations[..., 2]*self.signs[None, :, None]
                         *(self.config.radius+self.config.error_margin))
        guides[:, -1] = self.track.goal
        return poses, guides

    def __call__(self, initial, controls, prepared, first=0, crossing=False):
        x, u = [np.ascontiguousarray(a, dtype=np.float64) for a in (initial, controls)]
        if x.shape != (17,) or u.ndim != 3 or u.shape[2] != 4 or not 1<=u.shape[1]<=self.nc:
            raise ValueError("expected state[17], rotor thrust[batch,control_steps,4]")
        ns=2*u.shape[1]
        if not np.all(np.isfinite(x)) or abs(np.linalg.norm(x[6:10])-1) > 1e-5:
            raise ValueError("invalid initial state/quaternion")
        if not 0 <= first <= len(self.track.order): raise ValueError("invalid next gate")
        poses, guides = [np.ascontiguousarray(a) for a in prepared]
        gates = len(self.track.order)
        if poses.shape != (self.ns+1, gates, 12) or guides.shape != (self.ns+1, gates+1, 3):
            raise ValueError("prepared geometry/guide dimensions do not match track")
        poses,guides=poses[:ns+1],guides[:ns+1]
        n = len(u)
        states = np.empty((n, ns+1, 17)); clear = np.empty((n, ns))
        valid = np.empty(n, dtype=np.int32); indices = np.empty((n, ns+1), dtype=np.int32); race = np.empty(n)
        phases = np.empty((n, ns+1), dtype=np.int32)
        reasons = np.empty(n,dtype=np.int32)
        arrays = [x, u, poses, self.geom, self.frame_speeds, self.signs, guides, self.cfg]
        self.lib.v3_rollout(n, ns, len(self.track.order), *(a.ctypes.data for a in arrays),
                            first, int(crossing), states.ctypes.data, clear.ctypes.data,
                            valid.ctypes.data, indices.ctypes.data, race.ctypes.data, phases.ctypes.data,reasons.ctypes.data)
        return Batch(states, clear, valid.astype(bool), indices, race, phases,reasons)

    def integrate(self, state, controls, dt=.02):
        state = np.array(state, dtype=np.float64, copy=True)
        u = np.ascontiguousarray(controls, dtype=np.float64)
        if state.shape != (17,) or u.shape != (4,) or dt <= 0:
            raise ValueError("invalid integration input")
        self.lib.v3_integrate(state.ctypes.data, u.ctypes.data, dt, int(np.ceil(dt/.005)))
        return state

    def guided(self,state,prepared,first,phase,previous,count=24):
        if first>=len(self.track.order): return np.empty((0,self.nc,4))
        state,poses,previous=[np.ascontiguousarray(x,dtype=np.float64)
                              for x in (state,prepared[0],previous)]
        output=np.empty((count,self.nc,4))
        self.lib.v3_guided(count,len(self.track.order),state.ctypes.data,poses.ctypes.data,
            self.signs.ctypes.data,self.cfg.ctypes.data,first,int(phase),previous.ctypes.data,output.ctypes.data)
        return output


def hover_state(position):
    state = np.zeros(17)
    state[:3], state[6], state[13:] = position, 1., np.sqrt(MASS*9.8066/(4*KF))
    return state
