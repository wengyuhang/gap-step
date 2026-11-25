"""Geometry-only redesign. Original apertures, safe maps and four planners stay."""
import copy
from dataclasses import dataclass, replace, asdict
import numpy as np
from scipy.spatial.transform import Rotation
from .. import scenario as previous

ROOT=previous.ROOT; ASSETS=previous.ASSETS
START=np.array([-16.,4.,3.2])
ANCHORS=np.array([[0.,0.,5.],[16.,-6.,5.],[32.,6.,5.],[48.,-6.,5.],
                  [64.,4.,5.],[44.,24.,5.],[12.,24.,5.]])
# Frequency Hz / actual horizontal amplitude m, per gate. All phases remain 0.
BASE=((.30,.75),(.40,1.),(.50,1.),(.45,.75),(.55,1.),(.60,1.),(.35,.75))
LEVELS=('static','mixed_base','translate_1','swing_1p5','translate_2',
        'swing_2p5','translate_3','swing_3p5','translate_4','swing_4')
UPGRADES=((2,1.,1.5),(5,1.5,1.5),(2,2.,2.),(5,2.5,2.),
          (2,3.,3.),(5,3.5,3.),(2,4.,4.),(5,4.,4.))
pose=previous.pose


@dataclass(frozen=True)
class Case:
    layout:str='separated'
    level:str='static'
    def __post_init__(self):
        if self.layout!='separated' or self.level not in LEVELS:
            raise ValueError('unknown separated-course condition')


def profile(level):
    if level not in LEVELS: raise ValueError(level)
    if level=='static': return tuple((f,0.) for f,_ in BASE)
    values=list(BASE)
    for gate,f,a in UPGRADES[:max(0,LEVELS.index(level)-1)]: values[gate]=(f,a)
    return tuple(values)


def build_case(case):
    # Reuse the exact historic openings and shape-specific inset metadata.
    base,_=previous.build_case(previous.Case('original','static'))
    nodes=np.vstack([START,ANCHORS,START]); windows=[]
    for i,(template,p,(f,a)) in enumerate(zip(base.windows,ANCHORS,profile(case.level))):
        incoming=(p-nodes[i]).copy();outgoing=(nodes[i+2]-p).copy()
        incoming[2]=outgoing[2]=0.
        normal=incoming/np.linalg.norm(incoming)+outgoing/np.linalg.norm(outgoing)
        normal/=np.linalg.norm(normal);up=np.array([0.,0.,1.])
        R=np.column_stack((up,np.cross(normal,up),normal))
        w=copy.deepcopy(template);w.center0=p.copy();w.R=R
        w.angles0_rpy=Rotation.from_matrix(R).as_euler('xyz')
        w.motion=previous.Motion('translation' if i%2==0 else 'swing',R,f,a)
        windows.append(w)
    return replace(base,name='separated_'+case.level,start=START.copy(),goal=START.copy(),windows=tuple(windows)),None


def manifest(case):
    track,_=build_case(case)
    return dict(case=asdict(case),start=START.tolist(),goal=START.tolist(),return_to_start=True,
        all_seven_joint=True,intermediate_stops=False,replanning=False,
        layout_rule='ordered non-self-intersecting slalom with outside return; chord-bisector vertical planes fixed before solving',
        anchors=ANCHORS.tolist(),inter_gate_distances_m=np.linalg.norm(np.diff(ANCHORS,axis=0),axis=1).tolist(),
        reference_polyline_length_m=float(np.linalg.norm(np.diff(np.vstack([START,ANCHORS,START]),axis=0),axis=1).sum()),
        translation_gates=[1,3,5,7],swing_gates=[2,4,6],phase=0.,
        escalation='only W3 translation and W6 swing are alternately upgraded; other five fixed',
        opening_source='unchanged compact_calibration original physical apertures and x500 shape-specific safe maps',
        mesh_directory=str(ASSETS/'meshes'),
        windows=[dict(name=w.name,anchor=w.center0.tolist(),R=w.R.tolist(),motion=w.motion.kind,
                      frequency_hz=w.motion.frequency,amplitude_m=w.motion.amplitude,phase=w.motion.phase,
                      swing_length_m=w.motion.length if w.motion.kind=='swing' else None,
                      swing_angle_deg=float(np.degrees(w.motion.angle)) if w.motion.kind=='swing' else None,
                      pivot=(w.center0+w.motion.length*w.R[:,0]).tolist() if w.motion.kind=='swing' else None,
                      physical_vertices=None if w.aperture.kind=='circle' else w.aperture.physical_vertices.tolist(),
                      physical_radius_m=w.aperture.radius if w.aperture.kind=='circle' else None,
                      safe_vertices=None if w.aperture.kind=='circle' else w.aperture.safe_vertices.tolist(),
                      margin_m=w.aperture.margin) for w in track.windows])
