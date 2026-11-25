"""Variable-count frozen courses; motion is identical to the validated adapter."""
from dataclasses import dataclass
from pathlib import Path
import importlib.util,json,sys
import numpy as np
from scipy.spatial.transform import Rotation
from convex_timevarying_window.geometry import ConvexAperture
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from convex_timevarying_window.comparisons.论文实验.实验2.static_calibration.scenario import Aperture
from convex_timevarying_window.comparisons.论文实验.实验2 import scenario as mixed

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('validated_random_motion',HERE.parent/'scenario.py')
base=importlib.util.module_from_spec(spec);sys.modules[spec.name]=base;spec.loader.exec_module(base)
Motion=base.Motion;pose=base.pose
ROOT=base.ROOT;ASSETS=HERE/'results/assets_v1';SWING_LENGTH=2.2
VARIANT='expanded_paired_v1_20261114'
LEVELS=tuple(f'n{n}_{spacing}' for n in [10,14,18] for spacing in ['compact','long'])
@dataclass(frozen=True)
class Case:
    level:str
    layout:str='expanded_serpentine'
    def __post_init__(self):
        if self.level not in LEVELS:raise ValueError(self.level)

def manifest(case):return json.loads((HERE/'results/cases_v1'/f'{case.level}.json').read_text())

def aperture(row):
    if row['kind']=='circle':native=ConvexAperture('circle',radius=row['radius'],margin=0.)
    else:
        v=np.array(row['vertices']);native=ConvexAperture('polygon',physical_vertices=v,safe_vertices=v,margin=0.)
    return Aperture(native)

class Window(mixed.Window):
    def __init__(self,row):
        R=np.array(row['R']);super().__init__(row['index'],row['center'],R,row['frequency_hz'],row['amplitude_m'])
        self.name=row['name'];self.aperture=aperture(row['aperture']);self.shape_name=row['shape']
        self.motion=Motion(row['motion_kind'],R,row['frequency_hz'],row['amplitude_m'],phase=row['phase'],length=SWING_LENGTH,direction_local=tuple(row['direction_local']))
    def point_and_jacobians(self,d,t):
        q,J=self.aperture.point_and_jacobian(d);c,R,dc,dR=self.state_at(t)
        return c+R[:,:2]@q,q,R[:,:2]@J,dc+dR[:,:2]@q
    def to_point(self,d,t):return self.point_and_jacobians(d,t)[0]

def build_case(case):
    d=manifest(case);windows=tuple(Window(r) for r in d['windows'])
    return SCWindowTrack(case.level,np.array(d['start']),np.array(d['goal']),windows,tuple(range(len(windows)))),None
