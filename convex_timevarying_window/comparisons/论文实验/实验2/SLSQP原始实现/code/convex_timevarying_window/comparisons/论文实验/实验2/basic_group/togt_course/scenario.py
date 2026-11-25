"""TOGT's actual interlaced/stacked layout; no hand-drawn perimeter loop."""
import copy,json
from dataclasses import dataclass,replace,asdict
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .. import scenario as previous

HERE=Path(__file__).resolve().parent;ROOT=previous.ROOT;ASSETS=previous.ASSETS
UPSTREAM=json.loads((HERE/'upstream.json').read_text())
SCALE=2.2;OFFSET=np.array([0.,0.,2.])
START=np.array(UPSTREAM['start'])*SCALE+OFFSET
ANCHORS=np.array(UPSTREAM['positions'])*SCALE+OFFSET
BASE=((.30,.75),(.40,1.),(.50,1.),(.60,1.),(.55,1.),(.45,.75),(.35,.75))
LEVELS=('static','mixed_base','translate_1','swing_1p5','translate_2','swing_2p5','translate_3','swing_3p5','translate_4','swing_4')
UPGRADES=((2,1.,1.5),(3,1.5,1.5),(2,2.,2.),(3,2.5,2.),(2,3.,3.),(3,3.5,3.),(2,4.,4.),(3,4.,4.))
pose=previous.pose


@dataclass(frozen=True)
class Case:
    layout:str='togt'
    level:str='static'
    def __post_init__(self):
        if self.layout!='togt' or self.level not in LEVELS:raise ValueError('unknown TOGT-layout condition')


def profile(level):
    if level not in LEVELS:raise ValueError(level)
    if level=='static':return tuple((f,0.) for f,_ in BASE)
    values=list(BASE)
    for i,f,a in UPGRADES[:max(0,LEVELS.index(level)-1)]:values[i]=(f,a)
    return tuple(values)


def build_case(case):
    base,_=previous.build_case(previous.Case('original','static'));windows=[]
    # Upstream G7 is SingleBall, hence has no frame orientation. Reuse the
    # established seventh-window vertical yaw 200 degrees and disclose it.
    angles=UPSTREAM['rpy_degrees_first_six']+[[0.,-90.,200.]]
    for i,(template,p,rpy,(f,a)) in enumerate(zip(base.windows,ANCHORS,angles,profile(case.level))):
        w=copy.deepcopy(template);w.center0=p.copy();w.R=Rotation.from_euler('xyz',rpy,degrees=True).as_matrix()
        w.angles0_rpy=np.radians(rpy);w.motion=previous.Motion('translation' if i%2==0 else 'swing',w.R,f,a)
        windows.append(w)
    return replace(base,name='togt_layout_'+case.level,start=START.copy(),goal=START.copy(),windows=tuple(windows)),None


def manifest(case):
    tr,_=build_case(case)
    return dict(case=asdict(case),upstream=UPSTREAM,layout_rule='official TOGT positions/order and first-six RPY, uniformly scaled; stacked G4/G5 preserved',
        position_scale=SCALE,global_offset_m=OFFSET.tolist(),start=START.tolist(),goal=START.tolist(),return_to_start=True,
        original_end_replaced_by_closed_return=True,seventh_plane_adaptation='original SingleBall replaced by current rectangle at yaw 200deg',
        original_shapes_replaced_by_current_seven_physical_apertures=True,original_G3_corridor_not_reproduced=True,
        all_seven_joint=True,replanning=False,intermediate_stops=False,translation_gates=[1,3,5,7],swing_gates=[2,4,6],
        challenge_gates=[3,4],escalation='alternate W3 translation and W4 swing only; five other gates fixed, all phases zero',
        reference_polyline_length_m=float(np.linalg.norm(np.diff(np.vstack([START,ANCHORS,START]),axis=0),axis=1).sum()),
        inter_gate_distances_m=np.linalg.norm(np.diff(ANCHORS,axis=0),axis=1).tolist(),
        windows=[dict(name=w.name,anchor=w.center0.tolist(),R=w.R.tolist(),motion=w.motion.kind,
                      frequency_hz=w.motion.frequency,amplitude_m=w.motion.amplitude,phase=0.,
                      pivot=(w.center0+w.motion.length*w.R[:,0]).tolist() if w.motion.kind=='swing' else None,
                      margin_m=w.aperture.margin,physical_vertices=None if w.aperture.kind=='circle' else w.aperture.physical_vertices.tolist(),
                      physical_radius_m=w.aperture.radius if w.aperture.kind=='circle' else None) for w in tr.windows])
