"""One existing compact layout, with common horizontal translation/pendulum motion."""
from dataclasses import asdict
import xml.etree.ElementTree as ET
import casadi as ca
import numpy as np
from .. import scenario as mixed
from ..compact_calibration import scenario as compact
from ..static_calibration import scenario as historic

ROOT=compact.ROOT;ASSETS=compact.ASSETS;Case=compact.Case;LEVELS=mixed.LEVELS
prepare_assets=compact.prepare_assets
contact_topics=compact.contact_topics;contact_groups=compact.contact_groups
parse_all_contacts=compact.parse_all_contacts


class Motion(mixed.Motion):
    """Keep the aperture's old (up, horizontal, normal) coordinates unchanged."""
    @property
    def translation_amplitude(self):return -self.rotation_basis[:,1]*self.amplitude
    def local(self,t):
        p,v=super().local(t)
        return p[..., [1,0,2]]*[1,-1,1],v[..., [1,0,2]]*[1,-1,1]
    def local_bounds(self,a,b):
        lo,hi=super().local_bounds(a,b)
        return np.array([lo[1],-hi[0],lo[2]]),np.array([hi[1],-lo[0],hi[2]])


def build_case(case):
    if case.layout!='original':raise ValueError('This frozen experiment only evaluates the base layout.')
    track,_=compact.build_case(Case('original','static'))
    if case.level!='static':
        f,a=LEVELS[case.level]
        for w in track.windows:w.motion=Motion(w.motion.kind,w.R,f,a)
    return track,None


def pose(window,t):
    m=window.motion;z=2*np.pi*m.frequency*t+m.phase
    if m.kind=='translation':x=m.amplitude*ca.sin(z);y=0.
    else:
        angle=m.angle*ca.sin(z);x=m.length*ca.sin(angle);y=m.length*(1-ca.cos(angle))
    # Only the motion axes change; the physical frame/aperture rotation does not.
    d=ca.vertcat(y,-x,0.) if isinstance(m,Motion) else ca.vertcat(x,y,0.)
    return ca.DM(window.center0)+ca.DM(window.R)@d,ca.DM(window.R)


def manifest(case):
    result=compact.manifest(Case('original','static'))
    if case.level=='static':return result
    tr,_=build_case(case);f,a=LEVELS[case.level]
    result.update(case=asdict(case),purpose='frozen compact base-course four-method comparison',
        motion='fixed planes; W1/3/5/7 horizontal translation; W2/4/6 external-pivot pendulum centre motion',
        frequency_hz=f,amplitude_m=a,phase=0.,translation_gates=[1,3,5,7],swing_gates=[2,4,6],
        swing_length_m=mixed.SWING_LENGTH,swing_angle_deg=float(np.degrees(tr.windows[1].motion.angle)),
        motion_axes='axis0=-aperture.R[:,1] (horizontal), axis1=aperture.R[:,0] (up); aperture coordinates unchanged')
    for row,w in zip(result['windows'],tr.windows):
        row.update(motion=w.motion.kind,motion_axis0=(-w.R[:,1]).tolist(),motion_axis1=w.R[:,0].tolist(),
            pivot=(w.center0+mixed.SWING_LENGTH*w.R[:,0]).tolist() if w.motion.kind=='swing' else None)
    return result


def write_world(track,assets,out):
    target=historic.write_world(track,assets,out)
    if any(isinstance(w.motion,Motion) for w in track.windows):
        tree=ET.parse(target)
        models=[m for m in tree.findall('world/model') if m.get('name','').startswith('gate_')]
        for model,w in zip(models,track.windows):
            plugin=model.find("plugin[@name='paper_experiment::MixedGateMotion']")
            plugin.find('axis0').text=' '.join(map(str,-w.R[:,1]))
            plugin.find('axis1').text=' '.join(map(str,w.R[:,0]))
        tree.write(target,encoding='unicode')
    return target


def launch(track,assets,out):
    prepare_assets();common=historic.launch_common;previous=common.write_variant_world;common.write_variant_world=write_world
    try:return common.launch_variant(track,ASSETS,out)
    finally:common.write_variant_world=previous
